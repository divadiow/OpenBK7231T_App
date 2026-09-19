#ifdef WINDOWS
#include "selftest_local.h"
#include "../hal/hal_wifi.h"

#if ENABLE_DRIVER_WEMO
int DRV_WEMO_GetSearchType(const char *packet);

#define WEMO_TEST_SOAP(action, args) \
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>" \
    "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\">" \
    "<s:Body><u:" action " xmlns:u=\"urn:Belkin:service:basicevent:1\">" \
    args "</u:" action "></s:Body></s:Envelope>"

static const char *wemo_on = WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>1</BinaryState>");
static const char *wemo_off = WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>0</BinaryState>");
static const char *wemo_get = WEMO_TEST_SOAP("GetBinaryState", "");

static void Test_Wemo_UpstreamOTA(void) {
    char expected[128];
    unsigned char mac[8];
    SIM_ClearOBK(0);
    CFG_SetDeviceName("Kitchen & <Plug> \"Main\"");
    PIN_SetPinRoleForPinIndex(7, IOR_Relay);
    PIN_SetPinChannelForPinIndex(7, 2);
    CHANNEL_SetLabel(2, "Not the paired device name", 1);
    CMD_ExecuteCommand("startDriver Wemo", 0);

    Test_FakeHTTPClientPacket_GET("setup.xml");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("Kitchen &amp; &lt;Plug&gt; &quot;Main&quot;");
    SELFTEST_ASSERT_HTML_REPLY_NOT_CONTAINS("Not the paired device name");
    WiFI_GetMacAddress((char *)mac);
    snprintf(expected, sizeof(expected), "<UDN>uuid:Socket-1_0-201612%02X%02X%02X%02X</UDN>", mac[2], mac[3], mac[4], mac[5]);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS(expected);
    /* Home Assistant uses this legacy field as its entity unique ID. */
    snprintf(expected, sizeof(expected), "<serialNumber>%s</serialNumber>", HAL_GetMyIPString());
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS(expected);
    snprintf(expected, sizeof(expected), "<presentationURL>http://%s:80/</presentationURL>", HAL_GetMyIPString());
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS(expected);

    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_on);
    SELFTEST_ASSERT_CHANNEL(2, 1);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<u:SetBinaryStateResponse");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<BinaryState>1</BinaryState>");
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_get);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<u:GetBinaryStateResponse");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<BinaryState>1</BinaryState>");
    Test_FakeHTTPClientPacket_GET("setup.xml");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<binaryState>1</binaryState>");

    /* Upstream POWER selects the first eligible channel, not relay-first. */
    CHANNEL_SetType(1, ChType_Toggle);
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_on);
    SELFTEST_ASSERT_CHANNEL(1, 1);
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_off);
    SELFTEST_ASSERT_CHANNEL(1, 0);
    SELFTEST_ASSERT_CHANNEL(2, 1);
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_get);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<BinaryState>0</BinaryState>");

    SIM_ClearOBK(0);
    PIN_SetPinRoleForPinIndex(7, IOR_Relay_n);
    PIN_SetPinChannelForPinIndex(7, 2);
    CMD_ExecuteCommand("startDriver Wemo", 0);
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_on);
    SELFTEST_ASSERT_CHANNEL(2, 1);
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_off);
    SELFTEST_ASSERT_CHANNEL(2, 0);
}

static void Test_Wemo_InvalidRequests(void) {
    static const char *bad[] = {
        WEMO_TEST_SOAP("SetBinaryState", ""),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState/>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState></BinaryState>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>invalid</BinaryState>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>-1</BinaryState>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>2</BinaryState>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>1xyz</BinaryState>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>1"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>0</Wrong>"),
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState>0</BinaryState><BinaryState>1</BinaryState>"),
        WEMO_TEST_SOAP("SetBinaryState", "<!-- <BinaryState>0</BinaryState> -->"),
        WEMO_TEST_SOAP("SetBinaryState", "<x><BinaryState>0</BinaryState></x>")
    };
    unsigned int i;
    SIM_ClearOBK(0);
    CHANNEL_SetType(5, ChType_Toggle);
    CMD_ExecuteCommand("startDriver Wemo", 0);
    for (i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        CHANNEL_Set(5, 1, 0);
        Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", bad[i]);
        SELFTEST_ASSERT_CHANNEL(5, 1);
        SELFTEST_ASSERT(strstr(Test_GetLastHTTPReply(), "HTTP/1.1 500"));
        SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<errorCode>402</errorCode>");
        CHANNEL_Set(5, 0, 0);
        Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", bad[i]);
        SELFTEST_ASSERT_CHANNEL(5, 0);
        SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<s:Fault>");
    }
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", WEMO_TEST_SOAP("UnknownAction", ""));
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<errorCode>401</errorCode>");
    SELFTEST_ASSERT_CHANNEL(5, 0);

    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1",
        WEMO_TEST_SOAP("SetBinaryState", "<BinaryState> \r\n1\t </BinaryState>"));
    SELFTEST_ASSERT_CHANNEL(5, 1);
    /* A redundant BinaryState in Get was accepted by the original driver. */
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1",
        WEMO_TEST_SOAP("GetBinaryState", "<BinaryState>0</BinaryState>"));
    SELFTEST_ASSERT_CHANNEL(5, 1);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<BinaryState>1</BinaryState>");
    Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1",
        "<Envelope xmlns=\"http://schemas.xmlsoap.org/soap/envelope/\" "
        "xmlns:b=\"urn:Belkin:service:basicevent:1\"><Body><b:SetBinaryState>"
        "<b:BinaryState>0</b:BinaryState></b:SetBinaryState></Body></Envelope>");
    SELFTEST_ASSERT_CHANNEL(5, 0);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<u:SetBinaryStateResponse");
}

static void Test_Wemo_MetadataAndRestart(void) {
    int i;
    SIM_ClearOBK(0);
    CFG_SetDeviceName("Toggle Device");
    CHANNEL_SetType(5, ChType_Toggle);
    CMD_ExecuteCommand("startDriver Wemo", 0);
    Test_FakeHTTPClientPacket_POST("upnp/control/metainfo1",
        "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\"><s:Body>"
        "<u:GetMetaInfo xmlns:u=\"urn:Belkin:service:metainfo:1\"/>"
        "</s:Body></s:Envelope>");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<u:GetMetaInfoResponse");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("|OpenBeken|");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("||Socket</MetaInfo>");
    Test_FakeHTTPClientPacket_POST("upnp/control/metainfo1", wemo_get);
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("<errorCode>401</errorCode>");
    Test_FakeHTTPClientPacket_GET("eventservice.xml");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("SetBinaryState");
    Test_FakeHTTPClientPacket_GET("metainfoservice.xml");
    SELFTEST_ASSERT_HTML_REPLY_CONTAINS("GetMetaInfo");
    SELFTEST_ASSERT_HTML_REPLY_NOT_CONTAINS("GetExtMetaInfo");
    for (i = 0; i < 5; i++) {
        CMD_ExecuteCommand("stopDriver Wemo", 0);
        Test_FakeHTTPClientPacket_GET("setup.xml");
        SELFTEST_ASSERT_HTML_REPLY_CONTAINS("\"error\":404");
        Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_on);
        SELFTEST_ASSERT_CHANNEL(5, 0);
        CMD_ExecuteCommand("startDriver Wemo", 0);
        Test_FakeHTTPClientPacket_GET("setup.xml");
        SELFTEST_ASSERT_HTML_REPLY_CONTAINS("Toggle Device");
        Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_on);
        SELFTEST_ASSERT_CHANNEL(5, 1);
        Test_FakeHTTPClientPacket_POST("upnp/control/basicevent1", wemo_off);
        SELFTEST_ASSERT_CHANNEL(5, 0);
    }
}

static void Test_Wemo_DiscoveryTypes(void) {
    SELFTEST_ASSERT(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: urn:Belkin:device:**\r\n\r\n") == 1);
    SELFTEST_ASSERT(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nsT: urn:belkin:device:controllee:1\r\n\r\n") == 3);
    SELFTEST_ASSERT(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: upnp:rootdevice\r\n\r\n") == 2);
    SELFTEST_ASSERT(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: ssdp:all\r\n\r\n") == 2);
    SELFTEST_ASSERT(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: ssdpsearch:all\r\n\r\n") == 2);
    SELFTEST_ASSERT(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: other\r\nX: urn:Belkin:device:**\r\n\r\n") == 0);
}
#endif

void Test_Wemo(void) {
#if ENABLE_DRIVER_WEMO
    Test_Wemo_UpstreamOTA();
    Test_Wemo_InvalidRequests();
    Test_Wemo_MetadataAndRestart();
    Test_Wemo_DiscoveryTypes();
#endif
}
#endif
