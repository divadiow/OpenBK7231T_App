#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../hal/hal_wifi.h"
#include "drv_local.h"
#include "drv_ssdp.h"
#include "../httpserver/new_http.h"

/* Single-switch, Tasmota-style Wemo emulation. SSDP must be started separately.
 * Keep the original identity, device name, URLs and POWER target selection:
 * changing these would silently remap devices already paired before an OTA.
 * SUBSCRIBE/NOTIFY is not implemented by the HTTP server; clients must poll.
 */
#define WEMO_SERVICE "urn:Belkin:service:basicevent:1"
#define WEMO_META_SERVICE "urn:Belkin:service:metainfo:1"
#define WEMO_SOAP_NS "http://schemas.xmlsoap.org/soap/envelope/"
#define WEMO_XML_DEPTH 8

static const char g_wemo_msearch[] =
    "HTTP/1.1 200 OK\r\n"
    "CACHE-CONTROL: max-age=86400\r\n"
    "DATE: Fri, 15 Apr 2016 04:56:29 GMT\r\n"
    "EXT:\r\n"
    "LOCATION: http://%s:80/setup.xml\r\n"
    "OPT: \"http://schemas.upnp.org/upnp/1/0/\"; ns=01\r\n"
    "01-NLS: b9200ebb-736d-4b93-bf03-835149d13983\r\n"
    "SERVER: Unspecified, UPnP/1.0, Unspecified\r\n"
    "ST: %s\r\n"
    "USN: uuid:%s::%s\r\n"
    "X-User-Agent: redsonic\r\n\r\n";

static const char g_wemo_eventService[] =
    "<?xml version=\"1.0\"?>"
    "<scpd xmlns=\"urn:Belkin:service-1-0\">"
    "<specVersion><major>1</major><minor>0</minor></specVersion>"
    "<actionList><action><name>SetBinaryState</name><argumentList><argument>"
    "<name>BinaryState</name><direction>in</direction>"
    "<relatedStateVariable>BinaryState</relatedStateVariable>"
    "</argument></argumentList></action>"
    "<action><name>GetBinaryState</name><argumentList><argument>"
    "<name>BinaryState</name><direction>out</direction>"
    "<relatedStateVariable>BinaryState</relatedStateVariable><retval/>"
    "</argument></argumentList></action></actionList>"
    "<serviceStateTable><stateVariable sendEvents=\"yes\">"
    "<name>BinaryState</name><dataType>bool</dataType><defaultValue>0</defaultValue>"
    "</stateVariable><stateVariable sendEvents=\"yes\">"
    "<name>level</name><dataType>string</dataType><defaultValue>0</defaultValue>"
    "</stateVariable></serviceStateTable></scpd>\r\n";

static const char g_wemo_metaService[] =
    "<?xml version=\"1.0\"?>"
    "<scpd xmlns=\"urn:Belkin:service-1-0\">"
    "<specVersion><major>1</major><minor>0</minor></specVersion>"
    "<actionList><action><name>GetMetaInfo</name><argumentList><argument>"
    "<name>MetaInfo</name><direction>out</direction>"
    "<relatedStateVariable>MetaInfo</relatedStateVariable><retval/>"
    "</argument></argumentList></action></actionList>"
    "<serviceStateTable><stateVariable sendEvents=\"no\">"
    "<name>MetaInfo</name><dataType>string</dataType>"
    "</stateVariable></serviceStateTable></scpd>\r\n";

static const char g_wemo_envelope[] =
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
    "<s:Envelope xmlns:s=\"" WEMO_SOAP_NS "\" "
    "s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\"><s:Body>";
static const char g_wemo_envelopeEnd[] = "</s:Body></s:Envelope>\r\n";

static int g_wemo_enabled;
static unsigned int g_wemo_registered;
static char *g_wemo_ssdpReply;
static int stat_searchesReceived, stat_setupXMLVisits, stat_eventsReceived;
static int stat_metaServiceXMLVisits, stat_eventServiceXMLVisits;

/* Generate an HTTP-local snapshot, rather than freeing shared identity strings
 * while an HTTP callback may still be using them. Preserve the old MAC format.
 */
static void WEMO_Identity(char serial[32], char uid[64], unsigned char mac[8]) {
    memset(mac, 0, 8);
    WiFI_GetMacAddress((char *)mac);
    snprintf(serial, 32, "201612%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
    snprintf(uid, 64, "Socket-1_0-%s", serial);
}

static int WEMO_FindMainChannel(void) {
    int i;
    for (i = 0; i < CHANNEL_MAX; i++) {
        if (h_isChannelRelay(i) || CHANNEL_GetType(i) == ChType_Toggle) return i;
    }
    return -1;
}

static int WEMO_GetMainPowerState(void) {
    int ch;
#if ENABLE_LED_BASIC
    if (LED_IsLEDRunning()) return LED_GetEnableAll() ? 1 : 0;
#endif
    ch = WEMO_FindMainChannel();
    return ch >= 0 && CHANNEL_Get(ch) ? 1 : 0;
}

static void WEMO_SetMainPowerState(int value) {
    /* Preserve the original command path, including user command aliases and
     * any platform-specific POWER handling, not just equivalent GPIO writes. */
    CMD_ExecuteCommand(value ? "POWER ON" : "POWER OFF", 0);
}

/* Small bounded SOAP reader, not a general-purpose XML implementation. It walks
 * real elements, validates nesting and resolves inherited namespace bindings.
 * No heap allocation, DTD expansion, substring action matching or atoi(). The
 * complete message is validated before any output changes. Unknown argument
 * elements remain ignorable, as in the original driver; BinaryState must be a
 * single direct argument. Get accepts the legacy redundant BinaryState argument.
 */
typedef struct {
    const char *name, *attrs, *end;
    int length;
} wemo_xml_element_t;

static int WEMO_Space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static int WEMO_NameChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_' || c == ':' || c == '-' || c == '.';
}

static int WEMO_ValidName(const char *p, int length) {
    int i, first = 1, colon = 0;
    for (i = 0; i < length; i++) {
        char c = p[i];
        if (c == ':') {
            if (first || colon++) return 0;
            first = 1;
        } else {
            if (first && !((c >= 'A' && c <= 'Z') ||
                (c >= 'a' && c <= 'z') || c == '_')) return 0;
            first = 0;
        }
    }
    return !first;
}

/* Detect duplicate attributes, including conflicting xmlns declarations.
 * The prefix of this opening tag has already been validated. */
static int WEMO_DuplicateAttribute(const char *p, const char *end, const char *name, int length) {
    while (p < end) {
        const char *key;
        int keyLen;
        char quote;
        while (p < end && WEMO_Space(*p)) p++;
        if (p == end) break;
        key = p;
        while (p < end && WEMO_NameChar(*p)) p++;
        keyLen = (int)(p - key);
        if (keyLen == length && !memcmp(key, name, length)) return 1;
        while (p < end && WEMO_Space(*p)) p++;
        p++;
        while (p < end && WEMO_Space(*p)) p++;
        quote = *p++;
        while (p < end && *p != quote) p++;
        p++;
    }
    return 0;
}

static int WEMO_Equal(const char *p, int len, const char *s) {
    return (int)strlen(s) == len && !memcmp(p, s, len);
}

static int WEMO_LocalName(const wemo_xml_element_t *e, const char *s) {
    const char *p = e->name;
    int i;
    for (i = 0; i < e->length; i++) if (e->name[i] == ':') p = e->name + i + 1;
    return WEMO_Equal(p, (int)(e->name + e->length - p), s);
}

/* The opening tag has already been syntactically checked when this is used. */
static int WEMO_Namespace(wemo_xml_element_t *stack, int depth, const char *expected) {
    const char *prefix = stack[depth - 1].name;
    int prefixLen = 0, i;
    for (i = 0; i < stack[depth - 1].length; i++) {
        if (prefix[i] == ':') { prefixLen = i; break; }
    }
    for (i = depth - 1; i >= 0; i--) {
        const char *p = stack[i].attrs;
        while (p < stack[i].end) {
            const char *name, *value;
            int len;
            char quote;
            while (p < stack[i].end && WEMO_Space(*p)) p++;
            if (p == stack[i].end || *p == '/') break;
            name = p;
            while (p < stack[i].end && WEMO_NameChar(*p)) p++;
            len = (int)(p - name);
            while (p < stack[i].end && WEMO_Space(*p)) p++;
            p++; /* '=' */
            while (p < stack[i].end && WEMO_Space(*p)) p++;
            quote = *p++;
            value = p;
            while (p < stack[i].end && *p != quote) p++;
            if ((!prefixLen && WEMO_Equal(name, len, "xmlns")) ||
                (prefixLen && len == prefixLen + 6 && !memcmp(name, "xmlns:", 6) &&
                    !memcmp(name + 6, prefix, prefixLen))) {
                return WEMO_Equal(value, (int)(p - value), expected);
            }
            p++;
        }
    }
    return !prefixLen && !*expected;
}

static const char *WEMO_FindEnd(const char *p, const char *end, const char *text, int len) {
    while (end - p >= len) {
        if (!memcmp(p, text, len)) return p;
        p++;
    }
    return NULL;
}

/* Returns an action initial (G/S/M), or negative UPnP error code. */
static int WEMO_ParseSOAP(const char *xml, int length, int meta, int *state) {
    wemo_xml_element_t stack[WEMO_XML_DEPTH];
    const char *p = xml, *end;
    int depth = 0, root = 0, body = 0, action = 0, binary = 0, value = -1;
    int bodyDepth = 0, actionDepth = 0, binaryDepth = 0, header = 0, headerDepth = 0;
    if (!xml || length <= 0 || memchr(xml, 0, length)) return -402;
    end = xml + length;
    if (length >= 3 && !memcmp(p, "\xef\xbb\xbf", 3)) p += 3;
    while (p < end) {
        const char *start, *textEnd;
        wemo_xml_element_t e;
        int closing, empty = 0;
        if (*p != '<') {
            if (binaryDepth == depth && binaryDepth) {
                if (!WEMO_Space(*p)) {
                    if (value != -1 || (*p != '0' && *p != '1')) return -402;
                    value = *p - '0';
                }
            } else if (!WEMO_Space(*p) && (!headerDepth || depth <= headerDepth) &&
                (!actionDepth || depth <= actionDepth)) {
                return -402;
            }
            p++;
            continue;
        }
        if (end - p >= 4 && !memcmp(p, "<!--", 4)) {
            textEnd = WEMO_FindEnd(p + 4, end, "-->", 3);
            if (!textEnd) return -402;
            p = textEnd + 3;
            continue;
        }
        if (end - p >= 2 && p[1] == '?') {
            textEnd = WEMO_FindEnd(p + 2, end, "?>", 2);
            if (!textEnd || depth || root) return -402;
            p = textEnd + 2;
            continue;
        }
        /* DTDs and entity declarations are deliberately unsupported. */
        if (++p == end) return -402;
        closing = *p == '/';
        if (closing && ++p == end) return -402;
        start = p;
        if (!((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') || *p == '_')) return -402;
        while (p < end && WEMO_NameChar(*p)) p++;
        e.name = start;
        e.length = (int)(p - start);
        if (!WEMO_ValidName(start, e.length)) return -402;
        e.attrs = p;
        if (closing) {
            while (p < end && WEMO_Space(*p)) p++;
            if (p == end || *p++ != '>' || !depth || e.length != stack[depth - 1].length ||
                memcmp(e.name, stack[depth - 1].name, e.length)) return -402;
        } else {
            while (p < end && *p != '>' && *p != '/') {
                const char *attribute;
                char quote;
                if (!WEMO_Space(*p)) return -402;
                while (p < end && WEMO_Space(*p)) p++;
                if (p == end || *p == '>' || *p == '/') break;
                attribute = p;
                while (p < end && WEMO_NameChar(*p)) p++;
                if (!WEMO_ValidName(attribute, (int)(p - attribute)) ||
                    WEMO_DuplicateAttribute(e.attrs, attribute, attribute, (int)(p - attribute))) return -402;
                while (p < end && WEMO_Space(*p)) p++;
                if (p == end || *p++ != '=') return -402;
                while (p < end && WEMO_Space(*p)) p++;
                if (p == end || (*p != '\'' && *p != '"')) return -402;
                quote = *p++;
                while (p < end && *p != quote) {
                    if (*p == '<') return -402;
                    p++;
                }
                if (p == end) return -402;
                p++;
            }
            e.end = p;
            if (p < end && *p == '/') { empty = 1; p++; }
            if (p == end || *p++ != '>' || depth == WEMO_XML_DEPTH) return -402;
            stack[depth++] = e;
            if (depth == 1) {
                if (root++ || !WEMO_LocalName(&e, "Envelope") ||
                    !WEMO_Namespace(stack, depth, WEMO_SOAP_NS)) return -402;
            } else if (depth == 2) {
                if (!WEMO_Namespace(stack, depth, WEMO_SOAP_NS)) return -402;
                if (WEMO_LocalName(&e, "Header")) {
                    if (header++ || body) return -402;
                    headerDepth = depth;
                } else if (WEMO_LocalName(&e, "Body")) {
                    if (body++) return -402;
                    bodyDepth = depth;
                } else return -402;
            } else if (bodyDepth && depth == bodyDepth + 1) {
                if (action) return -402;
                if (!WEMO_Namespace(stack, depth, meta ? WEMO_META_SERVICE : WEMO_SERVICE)) return -401;
                if (!meta && WEMO_LocalName(&e, "SetBinaryState")) action = 'S';
                else if (!meta && WEMO_LocalName(&e, "GetBinaryState")) action = 'G';
                else if (meta && WEMO_LocalName(&e, "GetMetaInfo")) action = 'M';
                else return -401;
                actionDepth = depth;
            } else if (actionDepth && depth == actionDepth + 1 && WEMO_LocalName(&e, "BinaryState")) {
                if (meta || binary++ || (!WEMO_Namespace(stack, depth, "") &&
                    !WEMO_Namespace(stack, depth, WEMO_SERVICE))) return -402;
                binaryDepth = depth;
            } else if (binaryDepth) return -402;
            if (!empty) continue;
        }
        if (depth == binaryDepth) {
            if (value < 0) return -402;
            binaryDepth = 0;
        }
        if (depth == actionDepth) actionDepth = 0;
        if (depth == bodyDepth) bodyDepth = 0;
        if (depth == headerDepth) headerDepth = 0;
        depth--;
    }
    if (depth || !body || !action || (action == 'S' && binary != 1)) return -402;
    *state = value;
    return action;
}

static int WEMO_RequestAction(http_request_t *request, int meta, int *state) {
    int i, count = 0, length, action;
    const char *expected, *service = meta ? WEMO_META_SERVICE : WEMO_SERVICE;
    if (!request->bodystart || request->bodylen <= 0 || request->contentLength < 0 ||
        request->contentLength > request->bodylen) return -402;
    length = request->contentLength ? request->contentLength : request->bodylen;
    action = WEMO_ParseSOAP(request->bodystart, length, meta, state);
    if (action < 0) return action;
    expected = action == 'S' ? "SetBinaryState" : action == 'G' ? "GetBinaryState" : "GetMetaInfo";
    for (i = 0; i < request->numheaders && i < MAX_HEADERS; i++) {
        const char *p = request->headers[i], *end;
        char quote = 0;
        int serviceLen = (int)strlen(service);
        if (!p || my_strnicmp(p, "SOAPACTION:", 11)) continue;
        if (count++) return -401;
        p += 11;
        while (WEMO_Space(*p)) p++;
        end = p + strlen(p);
        while (end > p && WEMO_Space(end[-1])) end--;
        if (p < end && (*p == '"' || *p == '\'')) {
            quote = *p++;
            if (end == p || end[-1] != quote) return -401;
            end--;
        }
        if (end - p <= serviceLen || memcmp(p, service, serviceLen) || p[serviceLen] != '#' ||
            !WEMO_Equal(p + serviceLen + 1, (int)(end - p - serviceLen - 1), expected)) return -401;
    }
    /* Headerless requests from existing OpenBeken clients remain supported. */
    return action;
}

static int WEMO_Fault(http_request_t *request, int code) {
    request->responseCode = 500;
    http_setup(request, httpMimeTypeXML);
    poststr(request, g_wemo_envelope);
    poststr(request, "<s:Fault><faultcode>s:Client</faultcode><faultstring>UPnPError</faultstring>"
        "<detail><UPnPError xmlns=\"urn:schemas-upnp-org:control-1-0\">");
    hprintf255(request, "<errorCode>%i</errorCode><errorDescription>%s</errorDescription>",
        code, code == 401 ? "Invalid Action" : "Invalid Args");
    poststr(request, "</UPnPError></detail></s:Fault>");
    poststr(request, g_wemo_envelopeEnd);
    poststr(request, NULL);
    return 0;
}

/* Return the legacy reply mode, or 3 for an explicit controllee search.
 * Parse ST rather than accepting a type mentioned in some unrelated header. */
int DRV_WEMO_GetSearchType(const char *packet) {
    int type = 0, seen = 0;
    const char *p = packet;
    if (!p) return 0;
    while (*p) {
        const char *end = strchr(p, '\n'), *value, *tail;
        int len;
        if (!end) end = p + strlen(p);
        if (end == p || (end == p + 1 && *p == '\r')) break;
        if (end - p >= 3 && !my_strnicmp(p, "ST:", 3)) {
            if (seen++) return 0;
            value = p + 3;
            while (value < end && WEMO_Space(*value)) value++;
            tail = end;
            while (tail > value && WEMO_Space(tail[-1])) tail--;
            len = (int)(tail - value);
            if (len == sizeof("urn:Belkin:device:**") - 1 && !my_strnicmp(value, "urn:Belkin:device:**", len)) type = 1;
            else if (len == sizeof("urn:Belkin:device:controllee:1") - 1 && !my_strnicmp(value, "urn:Belkin:device:controllee:1", len)) type = 3;
            else if ((len == sizeof("upnp:rootdevice") - 1 && !my_strnicmp(value, "upnp:rootdevice", len)) ||
                (len == sizeof("ssdp:all") - 1 && !my_strnicmp(value, "ssdp:all", len)) ||
                (len == sizeof("ssdpsearch:all") - 1 && !my_strnicmp(value, "ssdpsearch:all", len))) type = 2;
        }
        p = *end ? end + 1 : end;
    }
    return type;
}

void DRV_WEMO_Send_Advert_To(int mode, struct sockaddr_in *addr) {
    char serial[32], uid[64];
    unsigned char mac[8];
    const char *type;
    int length;
    const int capacity = sizeof(g_wemo_msearch) + 256;
    if (!g_wemo_enabled) return;
    type = mode == 1 ? "urn:Belkin:device:**" :
        mode == 3 ? "urn:Belkin:device:controllee:1" : "upnp:rootdevice";
    WEMO_Identity(serial, uid, mac);
    /* Called by the SSDP quick tick under the driver mutex, as is shutdown. */
    if (!g_wemo_ssdpReply) g_wemo_ssdpReply = (char *)malloc(capacity);
    if (!g_wemo_ssdpReply) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "WEMO: no memory for SSDP reply");
        return;
    }
    length = snprintf(g_wemo_ssdpReply, capacity, g_wemo_msearch, HAL_GetMyIPString(), type, uid, type);
    if (length < 0 || length >= capacity) return;
    DRV_SSDP_SendReply(addr, g_wemo_ssdpReply);
    stat_searchesReceived++;
}

static int WEMO_BasicEvent1(http_request_t *request) {
    int state, action;
    if (!g_wemo_enabled) return http_rest_error(request, 404, "Not Found");
    action = WEMO_RequestAction(request, 0, &state);
    if (action < 0) return WEMO_Fault(request, -action);
    if (action == 'S') WEMO_SetMainPowerState(state);
    http_setup(request, httpMimeTypeXML);
    poststr(request, g_wemo_envelope);
    hprintf255(request, "<u:%cetBinaryStateResponse xmlns:u=\"" WEMO_SERVICE "\">"
        "<BinaryState>%i</BinaryState></u:%cetBinaryStateResponse>", action, WEMO_GetMainPowerState(), action);
    poststr(request, g_wemo_envelopeEnd);
    poststr(request, NULL);
    stat_eventsReceived++;
    return 0;
}

static int WEMO_MetaInfo1(http_request_t *request) {
    char serial[32], uid[64];
    unsigned char mac[8];
    int state, action;
    if (!g_wemo_enabled) return http_rest_error(request, 404, "Not Found");
    action = WEMO_RequestAction(request, 1, &state);
    if (action < 0) return WEMO_Fault(request, -action);
    WEMO_Identity(serial, uid, mac);
    http_setup(request, httpMimeTypeXML);
    poststr(request, g_wemo_envelope);
    poststr(request, "<u:GetMetaInfoResponse xmlns:u=\"" WEMO_META_SERVICE "\"><MetaInfo>");
    /* Six fields consumed by pywemo.util.MetaInfo. Do not invent Belkin firmware
     * or remote-access metadata. The AP SSID is empty (not a Belkin setup AP).
     */
    hprintf255(request, "%02X%02X%02X%02X%02X%02X|%s|OpenBeken|",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], HAL_GetMyIPString());
    /* Some simulator/developer builds do not provide a firmware version.
     * Keep all six metadata fields, leaving only that optional field empty. */
#ifdef USER_SW_VER
    poststr_escaped(request, USER_SW_VER);
#endif
    poststr(request, "||Socket</MetaInfo></u:GetMetaInfoResponse>");
    poststr(request, g_wemo_envelopeEnd);
    poststr(request, NULL);
    return 0;
}

static int WEMO_EventService(http_request_t *request) {
    if (!g_wemo_enabled) return http_rest_error(request, 404, "Not Found");
    http_setup(request, httpMimeTypeXML);
    poststr(request, g_wemo_eventService);
    poststr(request, NULL);
    stat_eventServiceXMLVisits++;
    return 0;
}

static int WEMO_MetaInfoService(http_request_t *request) {
    if (!g_wemo_enabled) return http_rest_error(request, 404, "Not Found");
    http_setup(request, httpMimeTypeXML);
    poststr(request, g_wemo_metaService);
    poststr(request, NULL);
    stat_metaServiceXMLVisits++;
    return 0;
}

static int WEMO_Setup(http_request_t *request) {
    char serial[32], uid[64];
    unsigned char mac[8];
    if (!g_wemo_enabled) return http_rest_error(request, 404, "Not Found");
    WEMO_Identity(serial, uid, mac);
    http_setup(request, httpMimeTypeXML);
    poststr(request, "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
        "<root xmlns=\"urn:Belkin:device-1-0\"><device>"
        "<deviceType>urn:Belkin:device:controllee:1</deviceType><friendlyName>");
    poststr_escaped(request, (char *)CFG_GetDeviceName());
    poststr(request, "</friendlyName><manufacturer>Belkin International Inc.</manufacturer>"
        "<modelName>Socket</modelName><modelNumber>3.1415</modelNumber><UDN>uuid:");
    poststr(request, uid);
    poststr(request, "</UDN><serialNumber>");
    /* Upstream exposed the IP here. Home Assistant uses this field as its
     * entity unique ID, so correcting it during an OTA would create new entities.
     * Keep that legacy value; the UDN remains the stable MAC-derived identity.
     */
    poststr(request, HAL_GetMyIPString());
    poststr(request, "</serialNumber><presentationURL>http://");
    poststr(request, HAL_GetMyIPString());
    hprintf255(request, ":80/</presentationURL><binaryState>%i</binaryState>", WEMO_GetMainPowerState());
    poststr(request, "<serviceList><service>"
        "<serviceType>" WEMO_SERVICE "</serviceType>"
        "<serviceId>urn:Belkin:serviceId:basicevent1</serviceId>"
        "<controlURL>/upnp/control/basicevent1</controlURL>"
        "<eventSubURL>/upnp/event/basicevent1</eventSubURL>"
        "<SCPDURL>/eventservice.xml</SCPDURL></service><service>"
        "<serviceType>" WEMO_META_SERVICE "</serviceType>"
        "<serviceId>urn:Belkin:serviceId:metainfo1</serviceId>"
        "<controlURL>/upnp/control/metainfo1</controlURL>"
        "<eventSubURL>/upnp/event/metainfo1</eventSubURL>"
        "<SCPDURL>/metainfoservice.xml</SCPDURL>"
        "</service></serviceList></device></root>\r\n");
    poststr(request, NULL);
    stat_setupXMLVisits++;
    return 0;
}

void WEMO_AppendInformationToHTTPIndexPage(http_request_t *request, int pre) {
    if (pre) return;
    hprintf255(request, "<h4>WEMO: %s, searches %i, setup %i, events %i, mService %i, event %i</h4>",
        g_wemo_enabled ? "ready" : "initialisation failed", stat_searchesReceived,
        stat_setupXMLVisits, stat_eventsReceived, stat_metaServiceXMLVisits, stat_eventServiceXMLVisits);
}

void WEMO_Init(void) {
    static const struct {
        const char *url;
        int method;
        http_callback_fn callback;
    } routes[] = {
        { "/upnp/control/basicevent1", HTTP_POST, WEMO_BasicEvent1 },
        { "/eventservice.xml", HTTP_GET, WEMO_EventService },
        { "/metainfoservice.xml", HTTP_GET, WEMO_MetaInfoService },
        { "/setup.xml", HTTP_GET, WEMO_Setup },
        { "/upnp/control/metainfo1", HTTP_POST, WEMO_MetaInfo1 }
    };
    unsigned int i;
    g_wemo_enabled = 0;
    for (i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        if (!(g_wemo_registered & (1U << i))) {
            if (HTTP_RegisterCallback(routes[i].url, routes[i].method, routes[i].callback, 0) < 0) {
                addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "WEMO: cannot register %s; stop/start to retry", routes[i].url);
                /* Metadata is optional: the original four endpoints must still
                 * start on devices with only four free callback slots. */
                if (i == 4) break;
                return;
            }
            g_wemo_registered |= 1U << i;
        }
    }
    g_wemo_enabled = 1;
}

void WEMO_Shutdown(void) {
    /* HTTP callbacks cannot be unregistered. An already running HTTP request
     * owns its identity snapshot; new requests will receive 404.
     */
    g_wemo_enabled = 0;
    if (g_wemo_ssdpReply) {
        free(g_wemo_ssdpReply);
        g_wemo_ssdpReply = NULL;
    }
}
