#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "openopl1000_console.h"
#include "openopl1000_http_server.h"
#include "openopl1000_wifi_sta.h"
#include "sys_os_config.h"

#ifndef OPENOPL1000_HTTP_PORT
#define OPENOPL1000_HTTP_PORT 80
#endif

#define OPENOPL1000_HTTP_BACKLOG 2
#define OPENOPL1000_HTTP_STACK_SIZE 4096
#define OPENOPL1000_HTTP_RETRY_DELAY_MS 3000
#define OPENOPL1000_HTTP_REQUEST_SIZE 1024
#define OPENOPL1000_HTTP_RESPONSE_SIZE 8192
#define OPENOPL1000_CONSOLE_OUTPUT_SIZE 2048

static osThreadId s_httpThread;
static bool s_httpStarted;
static char s_response[OPENOPL1000_HTTP_RESPONSE_SIZE];
static char s_consoleOutput[OPENOPL1000_CONSOLE_OUTPUT_SIZE];

static int OpenOPL1000_HexValue(char c)
{
    if ((c >= '0') && (c <= '9'))
    {
        return c - '0';
    }
    if ((c >= 'a') && (c <= 'f'))
    {
        return c - 'a' + 10;
    }
    if ((c >= 'A') && (c <= 'F'))
    {
        return c - 'A' + 10;
    }
    return -1;
}

static void OpenOPL1000_UrlDecode(char *dst, size_t dstLen, const char *src)
{
    size_t used = 0;

    if ((dst == NULL) || (dstLen == 0))
    {
        return;
    }

    dst[0] = '\0';
    if (src == NULL)
    {
        return;
    }

    while ((*src != '\0') && (used + 1 < dstLen))
    {
        if (*src == '+')
        {
            dst[used++] = ' ';
            src++;
        }
        else if ((src[0] == '%') && (src[1] != '\0') && (src[2] != '\0'))
        {
            int hi = OpenOPL1000_HexValue(src[1]);
            int lo = OpenOPL1000_HexValue(src[2]);
            if ((hi >= 0) && (lo >= 0))
            {
                dst[used++] = (char)((hi << 4) | lo);
                src += 3;
            }
            else
            {
                dst[used++] = *src++;
            }
        }
        else
        {
            dst[used++] = *src++;
        }
    }

    dst[used] = '\0';
}

static void OpenOPL1000_ExtractPathAndQuery(const char *request, char *path, size_t pathLen, char *query, size_t queryLen)
{
    const char *start;
    const char *end;
    const char *qmark;
    size_t len;

    if ((path == NULL) || (pathLen == 0) || (query == NULL) || (queryLen == 0))
    {
        return;
    }

    snprintf(path, pathLen, "/");
    query[0] = '\0';

    if (request == NULL)
    {
        return;
    }

    start = strchr(request, ' ');
    if (start == NULL)
    {
        return;
    }
    start++;

    end = strchr(start, ' ');
    if (end == NULL)
    {
        return;
    }

    qmark = memchr(start, '?', (size_t)(end - start));
    if (qmark != NULL)
    {
        len = (size_t)(qmark - start);
        if (len >= pathLen)
        {
            len = pathLen - 1;
        }
        memcpy(path, start, len);
        path[len] = '\0';

        len = (size_t)(end - qmark - 1);
        if (len >= queryLen)
        {
            len = queryLen - 1;
        }
        memcpy(query, qmark + 1, len);
        query[len] = '\0';
    }
    else
    {
        len = (size_t)(end - start);
        if (len >= pathLen)
        {
            len = pathLen - 1;
        }
        memcpy(path, start, len);
        path[len] = '\0';
    }
}

static void OpenOPL1000_GetQueryValue(const char *query, const char *key, char *value, size_t valueLen)
{
    const char *p = query;
    size_t keyLen;

    if ((value == NULL) || (valueLen == 0))
    {
        return;
    }

    value[0] = '\0';
    if ((query == NULL) || (key == NULL))
    {
        return;
    }

    keyLen = strlen(key);
    while (*p != '\0')
    {
        const char *next = strchr(p, '&');
        size_t partLen = next ? (size_t)(next - p) : strlen(p);

        if ((partLen > keyLen) && (memcmp(p, key, keyLen) == 0) && (p[keyLen] == '='))
        {
            char encoded[256];
            size_t copyLen = partLen - keyLen - 1;
            if (copyLen >= sizeof(encoded))
            {
                copyLen = sizeof(encoded) - 1;
            }
            memcpy(encoded, p + keyLen + 1, copyLen);
            encoded[copyLen] = '\0';
            OpenOPL1000_UrlDecode(value, valueLen, encoded);
            return;
        }

        if (next == NULL)
        {
            break;
        }
        p = next + 1;
    }
}

static void OpenOPL1000_AppendRaw(char *dst, size_t dstLen, const char *text)
{
    size_t used;
    size_t remain;
    size_t addLen;

    if ((dst == NULL) || (dstLen == 0) || (text == NULL))
    {
        return;
    }

    used = strlen(dst);
    if (used >= dstLen - 1)
    {
        return;
    }

    remain = dstLen - used - 1;
    addLen = strlen(text);
    if (addLen > remain)
    {
        addLen = remain;
    }

    memcpy(dst + used, text, addLen);
    dst[used + addLen] = '\0';
}

static void OpenOPL1000_AppendFmt(char *dst, size_t dstLen, const char *fmt, ...)
{
    char temp[512];
    va_list args;

    if ((dst == NULL) || (dstLen == 0) || (fmt == NULL))
    {
        return;
    }

    va_start(args, fmt);
    vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);

    OpenOPL1000_AppendRaw(dst, dstLen, temp);
}

static void OpenOPL1000_HtmlEscapeAppend(char *dst, size_t dstLen, const char *src)
{
    if ((dst == NULL) || (dstLen == 0) || (src == NULL))
    {
        return;
    }

    while (*src != '\0')
    {
        switch (*src)
        {
            case '&':
                OpenOPL1000_AppendRaw(dst, dstLen, "&amp;");
                break;
            case '<':
                OpenOPL1000_AppendRaw(dst, dstLen, "&lt;");
                break;
            case '>':
                OpenOPL1000_AppendRaw(dst, dstLen, "&gt;");
                break;
            case '"':
                OpenOPL1000_AppendRaw(dst, dstLen, "&quot;");
                break;
            default:
            {
                char c[2];
                c[0] = *src;
                c[1] = '\0';
                OpenOPL1000_AppendRaw(dst, dstLen, c);
                break;
            }
        }
        src++;
    }
}

static void OpenOPL1000_JsonEscapeAppend(char *dst, size_t dstLen, const char *src)
{
    if ((dst == NULL) || (dstLen == 0) || (src == NULL))
    {
        return;
    }

    while (*src != '\0')
    {
        switch (*src)
        {
            case '\r':
                break;
            case '\n':
                OpenOPL1000_AppendRaw(dst, dstLen, "\\n");
                break;
            case '"':
                OpenOPL1000_AppendRaw(dst, dstLen, "\\\"");
                break;
            case '\\':
                OpenOPL1000_AppendRaw(dst, dstLen, "\\\\");
                break;
            default:
            {
                char c[2];
                c[0] = *src;
                c[1] = '\0';
                OpenOPL1000_AppendRaw(dst, dstLen, c);
                break;
            }
        }
        src++;
    }
}

static void OpenOPL1000_SendHttp(int clientSock, const char *status, const char *contentType, const char *body)
{
    char header[224];
    int bodyLen = body ? (int)strlen(body) : 0;
    int headerLen;

    headerLen = snprintf(header, sizeof(header),
                         "HTTP/1.0 %s\r\n"
                         "Content-Type: %s\r\n"
                         "Connection: close\r\n"
                         "Cache-Control: no-store\r\n"
                         "Content-Length: %d\r\n"
                         "\r\n",
                         status,
                         contentType,
                         bodyLen);

    lwip_send(clientSock, header, headerLen, 0);
    if ((body != NULL) && (bodyLen > 0))
    {
        lwip_send(clientSock, body, bodyLen, 0);
    }
}

static const char *OpenOPL1000_ActiveClass(const char *path, const char *item)
{
    return (strcmp(path, item) == 0) ? " class=\"active\"" : "";
}

static void OpenOPL1000_BuildPageHeader(char *out, size_t outLen, const char *title, const char *activePath)
{
    const char *pageTitle = title ? title : "OpenOPL1000";
    const char *active = activePath ? activePath : "/index";

    snprintf(out, outLen,
             "<!doctype html>"
             "<html><head>"
             "<meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             "<title>%s</title>"
             "<style>"
             ":root{--bg:#f3f5f7;--panel:#fff;--border:#d9dde2;--text:#111827;--muted:#606b7a;--accent:#1f6feb;--good:#0a7f28;--bad:#b42318}"
             "*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,Helvetica,sans-serif;font-size:15px;line-height:1.4}"
             "header{background:#20252b;color:#fff;padding:14px 18px;border-bottom:4px solid var(--accent)}"
             "header h1{margin:0;font-size:22px}header .sub{color:#c8d0dc;font-size:13px;margin-top:2px}"
             "nav{background:#fff;border-bottom:1px solid var(--border);padding:0 12px;display:flex;gap:4px;flex-wrap:wrap}"
             "nav a{display:block;padding:10px 12px;color:#1f2937;text-decoration:none;border-bottom:3px solid transparent}nav a.active{border-color:var(--accent);font-weight:bold}"
             "main{padding:16px;max-width:1120px;margin:0 auto}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}"
             ".card{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:12px;margin:0 0 12px 0;box-shadow:0 1px 2px rgba(0,0,0,.03)}"
             ".metric{font-size:24px;font-weight:bold}.label{color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:.04em}.ok{color:var(--good)}.bad{color:var(--bad)}"
             "input[type=text]{width:70%%;max-width:520px;padding:8px;border:1px solid #b9c1cc;border-radius:4px}button,input[type=submit]{padding:8px 12px;border:1px solid #1d5fbf;background:var(--accent);color:#fff;border-radius:4px}"
             "pre{white-space:pre-wrap;background:#101418;color:#e8edf2;padding:10px;border-radius:4px;overflow:auto}code{background:#eef1f5;padding:1px 4px;border-radius:3px}.muted{color:var(--muted)}"
             ".tiles a{display:inline-block;margin:3px 5px 3px 0;padding:7px 10px;background:#eef4ff;border:1px solid #bfd5ff;border-radius:4px;text-decoration:none;color:#164f9f}"
             "footer{color:var(--muted);font-size:12px;margin:16px 0 4px}"
             "</style>"
             "</head><body><header><h1>OpenOPL1000</h1><div class=\"sub\">OpenBeken-style OPL1000 station build</div></header>"
             "<nav>"
             "<a%s href=\"/index\">Home</a>"
             "<a%s href=\"/cfg\">Config</a>"
             "<a%s href=\"/cfg_wifi\">WiFi</a>"
             "<a%s href=\"/cfg_pins\">Pins</a>"
             "<a%s href=\"/logs\">Logs</a>"
             "<a%s href=\"/ota\">OTA</a>"
             "<a%s href=\"/about\">About</a>"
             "</nav><main>",
             pageTitle,
             OpenOPL1000_ActiveClass(active, "/index"),
             OpenOPL1000_ActiveClass(active, "/cfg"),
             OpenOPL1000_ActiveClass(active, "/cfg_wifi"),
             OpenOPL1000_ActiveClass(active, "/cfg_pins"),
             OpenOPL1000_ActiveClass(active, "/logs"),
             OpenOPL1000_ActiveClass(active, "/ota"),
             OpenOPL1000_ActiveClass(active, "/about"));
}

static void OpenOPL1000_BuildPageFooter(char *out, size_t outLen)
{
    OpenOPL1000_AppendRaw(out, outLen,
                          "<footer>OpenOPL1000 bring-up: STA Wi-Fi, DHCP, TCP/HTTP and command routing are active. "
                          "SoftAP is not exposed by the OPL1000 A2 SDK.</footer></main></body></html>");
}

static void OpenOPL1000_BuildCommandForm(char *out, size_t outLen, const char *placeholder)
{
    OpenOPL1000_AppendFmt(out, outLen,
                          "<form action=\"/cmd\" method=\"get\">"
                          "<input type=\"text\" name=\"cmnd\" placeholder=\"%s\" autofocus> "
                          "<input type=\"submit\" value=\"Run\">"
                          "</form>",
                          placeholder ? placeholder : "status");
}

static void OpenOPL1000_BuildCommandResult(char *out, size_t outLen, const char *command, const char *commandOutput)
{
    if ((command == NULL) || (command[0] == '\0'))
    {
        return;
    }

    OpenOPL1000_AppendRaw(out, outLen, "<div class=\"card\"><h2>Command result</h2><div class=\"label\">Command</div><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, command);
    OpenOPL1000_AppendRaw(out, outLen, "</pre><div class=\"label\">Output</div><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, commandOutput ? commandOutput : "");
    OpenOPL1000_AppendRaw(out, outLen, "</pre></div>");
}

static void OpenOPL1000_BuildHomePage(char *out, size_t outLen, const char *command, const char *commandOutput)
{
    char status[768];

    OpenOPL1000_WifiFormatStatus(status, sizeof(status));
    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000", "/index");

    OpenOPL1000_AppendRaw(out, outLen, "<div class=\"grid\">");
    OpenOPL1000_AppendFmt(out, outLen,
                          "<div class=\"card\"><div class=\"label\">WiFi</div><div class=\"metric %s\">%s</div><div class=\"muted\">SSID: <code>%s</code></div></div>",
                          OpenOPL1000_WifiHasIp() ? "ok" : "bad",
                          OpenOPL1000_WifiHasIp() ? "Online" : "Connecting",
                          OpenOPL1000_WifiGetTargetSsid());
    OpenOPL1000_AppendFmt(out, outLen,
                          "<div class=\"card\"><div class=\"label\">Uptime</div><div class=\"metric\">%lu s</div><div class=\"muted\">Since last boot</div></div>",
                          (unsigned long)OpenOPL1000_GetUptimeSeconds());
    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><div class=\"label\">Platform</div><div class=\"metric\">OPL1000 A2</div><div class=\"muted\">OpenOPL1000 early port</div></div>"
                          "<div class=\"card\"><div class=\"label\">HTTP</div><div class=\"metric ok\">Running</div><div class=\"muted\">Port 80</div></div>"
                          "</div>");

    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><h2>Home</h2>"
                          "<p>This is the main OpenOPL1000 home page. It is intentionally focused on the network/web layer first; "
                          "pin roles, PWM, MQTT, flash config and the rest of the OpenBeken runtime can be added after this page is stable.</p>"
                          "<div class=\"tiles\">"
                          "<a href=\"/cfg\">Configure</a>"
                          "<a href=\"/cfg_wifi\">WiFi status</a>"
                          "<a href=\"/logs\">Logs/status</a>"
                          "<a href=\"/api/status\">JSON status</a>"
                          "<a href=\"/cm?cmnd=status\">/cm?cmnd=status</a>"
                          "</div></div>");

    OpenOPL1000_AppendRaw(out, outLen, "<div class=\"card\"><h2>Command</h2>");
    OpenOPL1000_BuildCommandForm(out, outLen, "status, help, scan, scanresults, reconnect");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<p class=\"muted\">Quick commands: "
                          "<a href=\"/cmd?cmnd=status\">status</a> | "
                          "<a href=\"/cmd?cmnd=help\">help</a> | "
                          "<a href=\"/cmd?cmnd=scan\">scan</a> | "
                          "<a href=\"/cmd?cmnd=scanresults\">scanresults</a> | "
                          "<a href=\"/cmd?cmnd=reconnect\">reconnect</a></p></div>");

    OpenOPL1000_BuildCommandResult(out, outLen, command, commandOutput);

    OpenOPL1000_AppendRaw(out, outLen, "<div class=\"card\"><h2>Runtime status</h2><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, status);
    OpenOPL1000_AppendRaw(out, outLen, "</pre></div>");

    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildConfigPage(char *out, size_t outLen)
{
    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000 Config", "/cfg");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><h2>Configuration</h2>"
                          "<p>The OpenOPL1000 configuration shell is now routed like a normal device page, but persistent flash config is not wired yet.</p>"
                          "<div class=\"tiles\">"
                          "<a href=\"/cfg_wifi\">WiFi</a>"
                          "<a href=\"/cfg_pins\">Pins</a>"
                          "<a href=\"/ota\">OTA</a>"
                          "<a href=\"/about\">About</a>"
                          "</div></div>"
                          "<div class=\"card\"><h2>Available now</h2><pre>STA Wi-Fi\r\nDHCP\r\nHTTP server\r\n/cm?cmnd= command endpoint\r\nJSON status\r\nWi-Fi scan/reconnect commands</pre></div>"
                          "<div class=\"card\"><h2>Deferred</h2><pre>Persistent config\r\nPin role configuration\r\nPWM/channel model\r\nMQTT\r\nOTA update handler\r\nFull OBK command registry</pre></div>");
    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildWifiPage(char *out, size_t outLen)
{
    char status[768];
    char scans[1536];

    OpenOPL1000_WifiFormatStatus(status, sizeof(status));
    OpenOPL1000_WifiFormatScanResults(scans, sizeof(scans));

    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000 WiFi", "/cfg_wifi");
    OpenOPL1000_AppendFmt(out, outLen,
                          "<div class=\"card\"><h2>WiFi station</h2>"
                          "<p>This build connects as a station to <code>%s</code>. OPL1000 A2 SoftAP is not available in the SDK path used here.</p>"
                          "<p><a href=\"/cmd?cmnd=scan\">Start scan</a> | <a href=\"/cmd?cmnd=reconnect\">Reconnect</a></p></div>",
                          OpenOPL1000_WifiGetTargetSsid());
    OpenOPL1000_AppendRaw(out, outLen, "<div class=\"card\"><h2>Status</h2><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, status);
    OpenOPL1000_AppendRaw(out, outLen, "</pre></div><div class=\"card\"><h2>Last scan results</h2><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, scans);
    OpenOPL1000_AppendRaw(out, outLen, "</pre></div>");
    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildPinsPage(char *out, size_t outLen)
{
    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000 Pins", "/cfg_pins");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><h2>Pins</h2>"
                          "<p>Pin roles, PWM and channel mapping are deliberately deferred. This page is present so the web navigation is in place without pretending the HAL is complete.</p>"
                          "<p>For controlled board bring-up only, the command backend still has basic GPIO diagnostics.</p>"
                          "</div><div class=\"card\"><h2>GPIO diagnostic command</h2>");
    OpenOPL1000_BuildCommandForm(out, outLen, "gpio_read 16");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<p class=\"muted\">Default-safe GPIOs: 2-7, 10-11, 16-19, 22-23. Reserved: 0/1 UART1, 8/9 APS UART, 12-15 SPI0 flash, 20/21 ICE.</p></div>");
    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildLogsPage(char *out, size_t outLen)
{
    char status[768];
    char scans[1536];

    OpenOPL1000_WifiFormatStatus(status, sizeof(status));
    OpenOPL1000_WifiFormatScanResults(scans, sizeof(scans));

    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000 Logs", "/logs");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><h2>Runtime snapshot</h2><p>This is not a persistent log buffer yet; it exposes the current counters and cached Wi-Fi scan output.</p><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, status);
    OpenOPL1000_AppendRaw(out, outLen, "</pre></div><div class=\"card\"><h2>Cached scan results</h2><pre>");
    OpenOPL1000_HtmlEscapeAppend(out, outLen, scans);
    OpenOPL1000_AppendRaw(out, outLen, "</pre></div>");
    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildOtaPage(char *out, size_t outLen)
{
    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000 OTA", "/ota");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><h2>OTA</h2>"
                          "<p>The web OTA page is routed but intentionally inactive. The current output is still the vendor-style M3 patch image plus M0/loader blobs, not a proven OpenBeken OTA package.</p>"
                          "</div>");
    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildAboutPage(char *out, size_t outLen)
{
    OpenOPL1000_BuildPageHeader(out, outLen, "OpenOPL1000 About", "/about");
    OpenOPL1000_AppendRaw(out, outLen,
                          "<div class=\"card\"><h2>About OpenOPL1000</h2>"
                          "<pre>Platform: Opulinks OPL1000 A2\r\nSDK family: OPL1000A2 APS_PATCH\r\nMode: STA only\r\nWeb: OpenOPL1000 home page + command routes\r\nBuild stage: network/web bring-up</pre>"
                          "</div>");
    OpenOPL1000_BuildPageFooter(out, outLen);
}

static void OpenOPL1000_BuildJsonStatus(char *out, size_t outLen)
{
    char status[768];
    char escaped[1200];

    OpenOPL1000_WifiFormatStatus(status, sizeof(status));
    escaped[0] = '\0';
    OpenOPL1000_JsonEscapeAppend(escaped, sizeof(escaped), status);

    snprintf(out, outLen,
             "{\"name\":\"OpenOPL1000\",\"platform\":\"OPL1000\",\"target_ssid\":\"%s\",\"connected\":%s,\"got_ip\":%s,\"uptime_s\":%lu,\"status\":\"%s\"}",
             OpenOPL1000_WifiGetTargetSsid(),
             OpenOPL1000_WifiIsConnected() ? "true" : "false",
             OpenOPL1000_WifiHasIp() ? "true" : "false",
             (unsigned long)OpenOPL1000_GetUptimeSeconds(),
             escaped);
}

static bool OpenOPL1000_IsHomePath(const char *path)
{
    return (strcmp(path, "/") == 0) ||
           (strcmp(path, "/index") == 0) ||
           (strcmp(path, "/index.html") == 0);
}

static void OpenOPL1000_HttpServeClient(int clientSock)
{
    char request[OPENOPL1000_HTTP_REQUEST_SIZE];
    char path[96];
    char query[320];
    char command[224];
    int received;

    received = lwip_recv(clientSock, request, sizeof(request) - 1, 0);
    if (received > 0)
    {
        request[received] = '\0';
        printf("[OpenOPL1000] HTTP request received (%d bytes)\r\n", received);
    }
    else
    {
        request[0] = '\0';
        printf("[OpenOPL1000] HTTP request read returned %d\r\n", received);
    }

    OpenOPL1000_ExtractPathAndQuery(request, path, sizeof(path), query, sizeof(query));
    OpenOPL1000_GetQueryValue(query, "cmnd", command, sizeof(command));

    if ((strcmp(path, "/api/status") == 0) || (strcmp(path, "/status.json") == 0))
    {
        OpenOPL1000_BuildJsonStatus(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "application/json; charset=utf-8", s_response);
    }
    else if ((strcmp(path, "/api/cmd") == 0) || (strcmp(path, "/cm") == 0))
    {
        OpenOPL1000_ConsoleExecute(command, s_consoleOutput, sizeof(s_consoleOutput));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/plain; charset=utf-8", s_consoleOutput);
    }
    else if ((strcmp(path, "/cmd") == 0) || (strcmp(path, "/console") == 0) || OpenOPL1000_IsHomePath(path))
    {
        if (command[0] != '\0')
        {
            OpenOPL1000_ConsoleExecute(command, s_consoleOutput, sizeof(s_consoleOutput));
        }
        else
        {
            s_consoleOutput[0] = '\0';
        }

        s_response[0] = '\0';
        OpenOPL1000_BuildHomePage(s_response, sizeof(s_response), command, s_consoleOutput);
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/cfg") == 0)
    {
        s_response[0] = '\0';
        OpenOPL1000_BuildConfigPage(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/cfg_wifi") == 0)
    {
        s_response[0] = '\0';
        OpenOPL1000_BuildWifiPage(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/cfg_pins") == 0)
    {
        s_response[0] = '\0';
        OpenOPL1000_BuildPinsPage(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/logs") == 0)
    {
        s_response[0] = '\0';
        OpenOPL1000_BuildLogsPage(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/ota") == 0)
    {
        s_response[0] = '\0';
        OpenOPL1000_BuildOtaPage(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/about") == 0)
    {
        s_response[0] = '\0';
        OpenOPL1000_BuildAboutPage(s_response, sizeof(s_response));
        OpenOPL1000_SendHttp(clientSock, "200 OK", "text/html; charset=utf-8", s_response);
    }
    else if (strcmp(path, "/favicon.ico") == 0)
    {
        OpenOPL1000_SendHttp(clientSock, "204 No Content", "text/plain", "");
    }
    else
    {
        snprintf(s_response, sizeof(s_response), "Not found: %s\r\n", path);
        OpenOPL1000_SendHttp(clientSock, "404 Not Found", "text/plain; charset=utf-8", s_response);
    }

    lwip_close(clientSock);
}

static int OpenOPL1000_HttpCreateListenSocket(void)
{
    struct sockaddr_in serverAddr;
    int listenSock;
    int reuse = 1;

    listenSock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock < 0)
    {
        printf("[OpenOPL1000] HTTP socket create failed: %d\r\n", listenSock);
        return -1;
    }

    lwip_setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_len = sizeof(serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = lwip_htons(OPENOPL1000_HTTP_PORT);
    serverAddr.sin_addr.s_addr = lwip_htonl(IPADDR_ANY);

    if (lwip_bind(listenSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        printf("[OpenOPL1000] HTTP bind port %d failed\r\n", OPENOPL1000_HTTP_PORT);
        lwip_close(listenSock);
        return -1;
    }

    if (lwip_listen(listenSock, OPENOPL1000_HTTP_BACKLOG) < 0)
    {
        printf("[OpenOPL1000] HTTP listen failed\r\n");
        lwip_close(listenSock);
        return -1;
    }

    printf("[OpenOPL1000] HTTP home page listening on port %d\r\n", OPENOPL1000_HTTP_PORT);
    return listenSock;
}

static void OpenOPL1000_HttpThread(void *args)
{
    int listenSock;

    (void)args;

    while (1)
    {
        listenSock = OpenOPL1000_HttpCreateListenSocket();
        if (listenSock < 0)
        {
            osDelay(OPENOPL1000_HTTP_RETRY_DELAY_MS);
            continue;
        }

        while (1)
        {
            struct sockaddr_in clientAddr;
            socklen_t clientAddrLen = sizeof(clientAddr);
            int clientSock;

            clientSock = lwip_accept(listenSock, (struct sockaddr *)&clientAddr, &clientAddrLen);
            if (clientSock < 0)
            {
                printf("[OpenOPL1000] HTTP accept failed: %d\r\n", clientSock);
                osDelay(250);
                continue;
            }

            printf("[OpenOPL1000] HTTP client connected\r\n");
            OpenOPL1000_HttpServeClient(clientSock);
        }
    }
}

void OpenOPL1000_HttpServerStart(void)
{
    osThreadDef_t threadDef;

    if (s_httpStarted)
    {
        return;
    }

    s_httpStarted = true;

    threadDef.name = "openopl1000_http";
    threadDef.stacksize = OPENOPL1000_HTTP_STACK_SIZE;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_HttpThread;

    s_httpThread = osThreadCreate(&threadDef, NULL);
    if (s_httpThread == NULL)
    {
        s_httpStarted = false;
        printf("[OpenOPL1000] HTTP task create failed\r\n");
    }
    else
    {
        printf("[OpenOPL1000] HTTP task created\r\n");
    }
}
