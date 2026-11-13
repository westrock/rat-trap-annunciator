#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <ArduinoGraphics.h>
// #include <ArduinoMDNS.h>
#include "Arduino_LED_Matrix.h" // Include the LED_Matrix library
#include "limits.h"
#include "animations.h"
#include "main.h"
#include "secrets.h"

//--------------------------------------------------
// CONFIGURATION
//--------------------------------------------------
#define MAX_REQUEST_LINE 256
#define MAX_STATE_LEN 16
#define MAX_RSSI_LEN 8
#define MAX_METHOD_LEN 8
#define MAX_URI_LEN 128

//--------------------------------------------------
// SEE secrets.h for your network credentials...
//
// const char *sta_ssid = "**********";
// const char *sta_pass = "**********";
// const char *ap_ssid = "**********";
// const char *ap_pass = "**********";
//--------------------------------------------------

const int WIFI_MODE_TEST_PIN = 7; // Pin that will set the level
const int WIFI_MODE_PIN = 8;
const int SPEAKER_PIN = 11;

const unsigned long NETWORK_CHECK_INTERVAL_MS = 10000UL;
const unsigned long HEARTBEAT_TIMEOUT_MS      = 450000UL;   // 7.5 minutes
const unsigned long ALERT_TONE_MS             = 60000UL;    // 60 sec
const unsigned long TIMEOUT_TONE_DURATION_MS  = 60000UL;    // 60 sec
const unsigned long CHIRP_MS = 5000UL;                 // 5 sec
const unsigned int TONE_A3 = 220;
const unsigned int TONE_A4 = 440;
const unsigned int TONE_A5 = 880;
const unsigned int TONE_A6 = 1760;

const char *REASON_PHRASE_200_OK = "OK";
const char *REASON_PHRASE_400_BadRequest = "Bad Request";
const char *REASON_PHRASE_405_MethodNotAllowed = "Method Not Allowed";
const char *REASON_PHRASE_408_RequestTimeout = "Request Timeout";
const char *REASON_PHRASE_414_URITooLong = "URI Too Long";
const char *REASON_PHRASE_500_InternalServerError = "Internal Server Error";
const char *REASON_PHRASE_404_NotFound = "Not Found";

const int RESPONSE_CODE_200 = 200;
const int RESPONSE_CODE_400 = 400;
const int RESPONSE_CODE_405 = 405;
const int RESPONSE_CODE_408 = 408;
const int RESPONSE_CODE_414 = 414;
const int RESPONSE_CODE_500 = 500;
const int RESPONSE_CODE_404 = 404;

//--------------------------------------------------
// GLOBALS
//--------------------------------------------------

ArduinoLEDMatrix g_matrix;
WiFiUDP g_udp;
// MDNS mdns(g_udp);
WiFiServer g_server(80);

CurrentState g_last_reported_trap_state = NONE;
unsigned long g_last_network_check_time = 0;

bool g_is_AP_mode = false;
bool g_is_network_initialized = false;

unsigned long g_last_update_millis = 0;

bool g_switch_is_open = false;
bool g_alarm_is_active = false;
bool g_timeout_is_active = false;

bool g_alert_tone_is_active = false;
unsigned long g_tone_stop_time = 0;
unsigned long g_tone_start_time = 0;
unsigned long g_tone_duration = 0;
unsigned int g_tone_frequency_01 = 0;
unsigned int g_tone_frequency_02 = 0;

unsigned long g_tone_segment_duration = 0;
unsigned long g_tone_last_segment_change = 0;

long g_rssi_min = LONG_MAX;
long g_rssi_max = LONG_MIN;
long g_rssi_latest = 0;

bool g_tone_playing_first_segment = true;

//--------------------------------------------------
// SETUP
//--------------------------------------------------
void setup()
{
    Serial.begin(115200);

    unsigned long startSerialTime = millis();
    while (!Serial && millis() - startSerialTime < 2000)
    {
        delay(50);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    digitalWrite(LED_BUILTIN, LOW);

    // Set pin 7 as output and Enable internal pull-up on pin 8 (INPUT_PULLUP)
    pinMode(WIFI_MODE_TEST_PIN, OUTPUT);
    pinMode(WIFI_MODE_PIN, INPUT_PULLUP);

    g_matrix.begin();
    g_matrix.loadSequence(STARTUP);
    g_matrix.play(true);

    digitalWrite(WIFI_MODE_TEST_PIN, LOW);
    delay(5);
    g_is_AP_mode = (digitalRead(WIFI_MODE_PIN) == HIGH);

    Serial.println("System Initializing...");

    handleNetworkTransition(g_is_AP_mode);
    g_last_update_millis = millis();
}

//--------------------------------------------------
// LOOP
// The main loop function is responsible for continuously processing network health, handling HTTP requests, managing timeouts, and playing audio sequences.
//--------------------------------------------------

void loop()
{
    processNetworkHealth();
    processHttpRequests();
    processTimeout();
    processAlertTone(SPEAKER_PIN);
}

//--------------------------------------------------
// NETWORK HEALTH
// The processNetworkHealth function is responsible for monitoring the network status and handling any necessary transitions between AP and STA modes.
// It periodically checks the network status and triggers appropriate actions if the network is not functioning correctly.
// This function should be called regularly in the main loop to ensure the network health is continuously monitored.
//--------------------------------------------------
void processNetworkHealth()
{
    // mdns.run();
    if (millis() - g_last_network_check_time < NETWORK_CHECK_INTERVAL_MS)
    {
        return;
    }

    g_last_network_check_time = millis();

    if (!g_is_network_initialized)
    {
        Serial.println("Network not initialized. Retrying...");
        handleNetworkTransition(g_is_AP_mode);
        return;
    }

    uint8_t status = WiFi.status();
    if (g_is_AP_mode)
    {
        if (status != WL_AP_LISTENING && status != WL_AP_CONNECTED)
        {
            Serial.print("AP ");
            dumpWifiStatus((wl_status_t)status);

            Serial.println("AP lost. Restarting...");
            handleNetworkTransition(true);
        }
    }
    else
    {
        if (status != WL_CONNECTED)
        {
            Serial.println("STA disconnected. Reconnecting...");
            handleNetworkTransition(false);
        }
    }
}

//--------------------------------------------------
// HTTP SERVER
// The HTTP server handles incoming resource requests and routes them to the appropriate handler functions based on the URI's resource.
// It supports endpoints with the resources status, alert, report, and other custom actions to be defined.
//--------------------------------------------------

//--------------------------------------------------
// 
//--------------------------------------------------
bool getResourceParameter(const char *uri, char *key, char *value, size_t value_size)
{
    const char *key_value_ptr = strstr(uri, key);

    if (key_value_ptr == NULL)
    {
        value[0] = '\0';
        return false;
    }

    key_value_ptr += strlen(key);

    size_t i = 0;

    while (*key_value_ptr &&
           *key_value_ptr != '&' &&
           i < value_size - 1)
    {
        value[i++] = *key_value_ptr++;
    }

    value[i] = '\0';

    return true;
}

//--------------------------------------------------
// REQUEST PROCESSING
// The processRequestURI function is responsible for handling the incoming HTTP requests and routing them to the appropriate handler functions based on the URI and state parameter.
// The getResourceParameter function is used to extract the state parameter from the URI string. It returns true if the state parameter is found and false otherwise.
// The MAX_STATE_LEN constant defines the maximum length of the state parameter that can be extracted from the URI.
// The processRequestURI function should be called for each incoming HTTP request to ensure proper routing and handling of the request.
// The getResourceParameter function should be used within processRequestURI to extract the state parameter from the URI before routing the request to the appropriate handler.
// The HTTP server should be configured to call processRequestURI for each incoming request to ensure proper handling and routing.
//--------------------------------------------------

void processRequestURI(WiFiClient &client, const char *uri)
{
    static char response_buffer[512];
    char client_IP_address[16];

                resetHeartbeatTimer();

    strncpy(client_IP_address, client.remoteIP().toString().c_str(), sizeof(client_IP_address));

    if (strncasecmp(uri, "/report", 7) == 0)
    {
        //--------------------------------------------------
        // REPORT
        // The report endpoint provides the current state of the system.
        // The response includes the current state as a string.
        // The g_last_reported_trap_state variable holds the current state of the system, which is then converted to a string for the response.
        // The currentStateText variable holds the string representation of the current state, which is then used to construct the HTTP response.
        // The response_buffer variable holds the complete HTTP response, including the current state, which is then sent to the client.
        // The sendHttpResponse function is used to send the HTTP response to the client.
        // The client.stop function is used to close the connection to the client after sending the response.
        // The handleServerError function is used to handle any errors that occur while preparing the HTTP response.
        // The resetHeartbeatTimer function WILL NOT reset the heartbeat timer after handling the report request.
        //--------------------------------------------------
        int written = -1;
        if (g_is_AP_mode)
        {
            written = snprintf(response_buffer,
                               sizeof(response_buffer),
                               "Current State: %s\r\nIP Address: %s\r\nSignal strength (RSSI): N.A.\r\nRemote IP Address: %s\r\nRemote (RSSI) current/min/max: %ld/%ld/%ld dBm",
                               currentTrapStateToString(g_last_reported_trap_state),
                               WiFi.localIP().toString().c_str(),
                               client_IP_address,
                               g_rssi_latest, g_rssi_min, g_rssi_max);
        }
        else
        {
            written = snprintf(response_buffer,
                               sizeof(response_buffer),
                               "Current State: %s\r\nIP Address: %s\r\nSignal strength (RSSI): %ld dBm\r\nRemote IP Address: %s\r\nRemote (RSSI) current/min/max: %ld/%ld/%ld dBm",
                               currentTrapStateToString(g_last_reported_trap_state),
                               WiFi.localIP().toString().c_str(),
                               WiFi.RSSI(),
                               client_IP_address,
                               g_rssi_latest, g_rssi_min, g_rssi_max);
        }

        if (written < 0 || written >= sizeof(response_buffer))
        {
            handleServerError(client, "Unexpected buffer overflow preparing HTTP response.");
            return;
        }
        Serial.println("HTTP REPORT request received. Sending response:");
        Serial.println(response_buffer);
        sendHttpResponse(client, 200, REASON_PHRASE_200_OK, response_buffer);
    }
    else
    {
        char state[MAX_STATE_LEN];
        char rssi[MAX_RSSI_LEN];

        if (!getResourceParameter(uri, "state=", state, sizeof(state)))
        {
            Serial.print("Missing state parameter. URI: ");
            Serial.println(uri);
            sendHttpResponse(client, 400, REASON_PHRASE_400_BadRequest, "Missing Status/Alert State");
            return;
        }

        getResourceParameter(uri, "rssi=", rssi, sizeof(rssi));

        if (strlen(rssi) > 0)
        {
            long rssiValue;
            if (parseLong(rssi, &rssiValue))
            {
                updateRssiStats(rssiValue);
            }
            else
            {
                Serial.print("Invalid RSSI value: ");
                Serial.println(rssi);
            }
        }

        CurrentState the_incoming_state = (strcasecmp(state, "tripped") == 0) ? TRIPPED : (strcasecmp(state, "set") == 0 ? SET : NONE);
        bool resource_is_status = strncasecmp(uri, "/status", 7) == 0;
        bool resource_is_alert = strncasecmp(uri, "/alert", 6) == 0;

        if (resource_is_status || resource_is_alert)
        {
            switch (the_incoming_state)
            {
            case TRIPPED:
                handleStatusTripped();
                sendHttpResponse(client, RESPONSE_CODE_200, REASON_PHRASE_200_OK, resource_is_status ? "Status Tripped OK" : "Alert Tripped OK");
                break;
            case SET:
                handleStatusSet();
                sendHttpResponse(client, RESPONSE_CODE_200, REASON_PHRASE_200_OK, resource_is_status ?" Status Set OK" : "Alert Set OK");
                break;
            case NONE:
            default:
                Serial.println(resource_is_status ? "Invalid Status State" : "Invalid Alert State");
                sendHttpResponse(client, RESPONSE_CODE_400, REASON_PHRASE_400_BadRequest, resource_is_status ? "Invalid Status State" : "Invalid Alert State");
                break;
            }
        }
        else
        {
            //--------------------------------------------------
            // UNKNOWN ENDPOINT
            //--------------------------------------------------

            Serial.print("Unknown URI: ");
            Serial.println(uri);

            sendHttpResponse(client, RESPONSE_CODE_404, REASON_PHRASE_404_NotFound, "Unknown URI");
        }
    }
}

//--------------------------------------------------
// HTTP REQUEST HANDLING
// The processHttpRequests function is responsible for handling incoming HTTP requests, parsing the request line, and routing the request
// to the appropriate handler function.
//--------------------------------------------------
void processHttpRequests()
{
    WiFiClient client = g_server.available();

    if (!client)
    {
        return;
    }

    //--------------------------------------------------
    // Configure socket timeout
    //--------------------------------------------------


    //--------------------------------------------------
    // Read first request line
    //--------------------------------------------------

    client.setTimeout(1000);

    unsigned long timer_start_millis = millis();

    while (!client.available())
    {
        if (millis() - timer_start_millis > 1000)
        {
            Serial.println("Client data timeout");
            client.stop();
            return;
        }

        delay(1);
    }

    char request_line[MAX_REQUEST_LINE];

    size_t len = client.readBytesUntil(
        '\n',
        request_line,
        sizeof(request_line) - 1);
        
    request_line[len] = '\0';

    //--------------------------------------------------
    // Trim trailing CR if present
    //--------------------------------------------------

    if (len > 0 &&
        request_line[len - 1] == '\r')
    {
        request_line[len - 1] = '\0';
        --len;
    }

    //--------------------------------------------------
    // Timeout / empty request
    //--------------------------------------------------

    if (len == 0)
    {
        Serial.println("HTTP request timeout");
        sendHttpResponse(client, RESPONSE_CODE_408, REASON_PHRASE_408_RequestTimeout, "Request Timeout");

        return;
    }

    //--------------------------------------------------
    // Detect possible truncation
    //--------------------------------------------------

    if (len >= sizeof(request_line) - 1)
    {
        Serial.println("HTTP request line too long");
        sendHttpResponse(client, RESPONSE_CODE_414, REASON_PHRASE_414_URITooLong, "URI Too Long");
    
        return;
    }

    //--------------------------------------------------
    // Parse request line
    //--------------------------------------------------

    char method[MAX_METHOD_LEN];
    char uri[MAX_URI_LEN];

    int fields = sscanf(request_line, "%7s %127s", method, uri);

    if (fields != 2)
    {
        Serial.println("Malformed HTTP request");
        sendHttpResponse(client, RESPONSE_CODE_400, REASON_PHRASE_400_BadRequest, "Malformed request");

        return;
    }

    //--------------------------------------------------
    // Verify HTTP method
    //--------------------------------------------------

    if (strcasecmp(method, "GET") != 0)
    {
        Serial.print("Unsupported method: ");
        Serial.println(method);

        sendHttpResponse(client, RESPONSE_CODE_405, REASON_PHRASE_405_MethodNotAllowed, "HTTP method not supported");
 
        return;
    }

    //--------------------------------------------------
    // Ignore browser favicon requests
    //--------------------------------------------------

    if (strncasecmp(uri, "/favicon.ico", 12) == 0)
    {
        sendHttpResponse(client, RESPONSE_CODE_404, REASON_PHRASE_404_NotFound, "Not Found");
        return;
    }

    //--------------------------------------------------
    // Application request processing
    //--------------------------------------------------

    processRequestURI(client, uri);

    if (client.connected())
    {
        client.flush();
        delay(10);
        client.stop();
    }
}

//--------------------------------------------------
// SERVER ERROR HANDLING
// The handleServerError function is responsible for sending a 500 Internal Server Error response to the client.
//--------------------------------------------------
void handleServerError(WiFiClient &client, const char *message)
{
    sendHttpResponse(client, RESPONSE_CODE_500, REASON_PHRASE_500_InternalServerError, message);
}

//--------------------------------------------------
// HTTP RESPONSE SENDING
// The sendHttpResponse function is responsible for sending an HTTP response to the client with the specified status code, reason phrase, and message.
//--------------------------------------------------
void sendHttpResponse(WiFiClient &client, int status_code, const char *reason_phrase, const char *message)
{
    static char response_buffer[1024];

    int written = snprintf(response_buffer, sizeof(response_buffer), "HTTP/1.1 %d %s", status_code, reason_phrase);

    if (written < 0 || written >= sizeof(response_buffer))
    {
        handleServerError(client, "Unexpected buffer overflow preparing HTTP response.");
        return;
    }

    size_t body_length = (message != nullptr) ? strlen(message) : 0;

    client.println(response_buffer);
    client.println("Content-Type: text/plain");

    client.print("Content-Length: ");
    client.println(body_length);

    client.println("Connection: close");
    client.println();

    if (message != nullptr)
    {
        client.print(message);
    }
    client.flush();
    delay(10);
    client.stop();
}

//--------------------------------------------------
// NETWORK TRANSITION
// The handleNetworkTransition function is responsible for switching the network mode between Access Point (AP) and Station (STA) modes.
// The function takes a boolean parameter 'make_AP' to determine whether to switch to Access Point mode or Station mode.
// The function also updates the 'g_is_network_initialized' flag to reflect the current network state.
// The function handles both the AP and STA modes, including establishing the connection, setting up the server, and updating the network state.
// The function ensures that the network transition is handled gracefully, with appropriate error handling and status reporting.
//--------------------------------------------------
void handleNetworkTransition(bool make_AP)
{
    Serial.println("\n--- Setting Network Mode ---");

    // 1. Cleanup existing state to prevent memory leaks and conflicts
    WiFi.disconnect();
    // server.stop(); // If your server library supports a stop method

    g_is_network_initialized = false;

    if (make_AP)
    {
        Serial.println("Mode: Access Point (AP)");
        Serial.print("Firmware version: ");
        Serial.println(WiFi.firmwareVersion());
        int rc = WiFi.beginAP(ap_ssid, ap_pass);

        Serial.print("beginAP returned: ");
        Serial.println(rc);

        delay(2000);

        wl_status_t status = (wl_status_t)WiFi.status();
        dumpWifiStatus(status);

        if (status == WL_NO_MODULE)
        {
            Serial.println("No WiFi module found!");
        }

        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        if (status == WL_AP_LISTENING || status == WL_AP_CONNECTED)
        {
            Serial.print("AP established. IP address: ");
            Serial.println(WiFi.localIP());

            g_server.begin();

            g_is_network_initialized = true;
        }
        else
        {
            Serial.println("Error: Failed to establish Access Point.");
        }
    }
    else
    {
        Serial.print("Mode: Station (STA) - Connecting to ");
        Serial.println(sta_ssid);

        if (WiFi.begin(sta_ssid, sta_pass))
        {
            // Wait for connection (with timeout)
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 200)
            {
                delay(50);
                Serial.print(".");
                attempts++;
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                IPAddress ip;
                bool haveValidIP = false;

                for (int i = 0; i < 200; i++)
                {
                    ip = WiFi.localIP();

                    if (ip != IPAddress(0, 0, 0, 0))
                    {
                        haveValidIP = true;
                        break;
                    }
                    delay(50);
                }

                if (haveValidIP)
                {
                    Serial.println("\nConnected successfully!");
                    Serial.print("IP Address: ");
                    Serial.println(ip);

                    // print the received signal strength:
                    long rssi = WiFi.RSSI();
                    Serial.print("signal strength (RSSI):");
                    Serial.print(rssi);
                    Serial.println(" dBm");
                
                    g_server.begin();

                    g_is_network_initialized = true;
                }
                else
                {
                    Serial.println("\nError: Failed to obtain a valid IP address.");
                }
            }
            else
            {
                Serial.println("\nError: Failed to connect to WiFi.");
            }
        }
        else
        {
            Serial.println("Error: WiFi hardware not responding.");
        }
    }
}

void handleNetworkTransition_new(bool make_AP)
{
    Serial.println("\n--- Transitioning Network Mode ---");

    // 1. Cleanup existing state to prevent memory leaks and conflicts
    WiFi.disconnect();
    // server.stop(); // If your server library supports a stop method

    g_is_network_initialized = false;

    if (make_AP)
    {
        Serial.println("Mode: Access Point (AP)");
        if (WiFi.beginAP(ap_ssid, ap_pass) != 0)
        {
            Serial.println("Error: beginAP failed to initialize hardware.");
            return;
        }

        // Poll for AP status instead of hard delay
        int attempts = 0;
        while (WiFi.status() != WL_AP_LISTENING && attempts < 20)
        {
            delay(200);
            attempts++;
        }

        if (WiFi.status() == WL_AP_LISTENING)
        {
            Serial.print("AP established. IP: ");
            Serial.println(WiFi.localIP());
            g_is_network_initialized = true;
        }
        else
        {
            Serial.println("Error: Failed to establish Access Point.");
        }
    }
    else
    {
        Serial.print("Mode: Station (STA) - Connecting to ");
        Serial.println(sta_ssid);

        if (WiFi.begin(sta_ssid, sta_pass))
        {
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20)
            {
                delay(500);
                Serial.print(".");
                attempts++;
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.println("\nConnected successfully!");
                Serial.print("IP Address: ");
                Serial.println(WiFi.localIP());
                Serial.print("RSSI: ");
                Serial.println(WiFi.RSSI());
                g_is_network_initialized = true;
            }
            else
            {
                Serial.println("\nError: Connection timeout.");
            }
        }
        else
        {
            Serial.println("Error: WiFi hardware not responding.");
        }
    }
}

//--------------------------------------------------
// TIMER MANAGEMENT
//--------------------------------------------------
// HEARTBEAT MANAGEMENT
// The resetHeartbeatTimer function resets the heartbeat timer, indicating that the system is active.
//--------------------------------------------------
void resetHeartbeatTimer()
{
    g_last_update_millis = millis();

    if (g_timeout_is_active)
    {
        g_timeout_is_active = false;

        Serial.println("Heartbeat restored");

        clearTimeoutIndication();
    }
}

//--------------------------------------------------
// The processTimeout function is responsible for handling the heartbeat timeout.
// It checks if the time since the last update has exceeded the heartbeat timeout and triggers the appropriate actions if necessary.
//--------------------------------------------------
void processTimeout()
{
    // Check if the heartbeat timeout has been reached, even if we are already in a timeout state.
    if (millis() - g_last_update_millis >= HEARTBEAT_TIMEOUT_MS)
    {
        g_timeout_is_active = true;
        g_last_update_millis = millis();
        handleTimeout();
    }
}

//--------------------------------------------------
// AUDIO
//--------------------------------------------------
// AUDIO MANAGEMENT
// The startToneFor function initiates a tone sequence on the specified pin with the given frequencies and durations.
// The processAlertTone function handles the progression of the tone sequence, switching between segments and stopping the tone when the total duration is reached.
//--------------------------------------------------
void startToneFor(uint8_t tone_pin, unsigned int frequency1, unsigned int frequency2, unsigned long segment_duration_millis, unsigned long duration_millis)
{
    g_tone_frequency_01 = frequency1;
    g_tone_frequency_02 = frequency2;

    g_tone_segment_duration = segment_duration_millis;

    g_tone_duration = duration_millis;
    g_tone_start_time = millis();
    g_tone_last_segment_change = g_tone_start_time;

    g_tone_playing_first_segment = true;
    g_alert_tone_is_active = true;

    if (frequency1 > 0)
    {
        tone(tone_pin, frequency1);
    }
    else
    {
        noTone(tone_pin);
    }
}

//--------------------------------------------------
// AUDIO PROCESSING
// The processAlertTone function is responsible for managing the audio output, including switching between tone segments and stopping the tone when the total duration is reached.
// It should be called regularly in the main loop to ensure the audio sequence progresses correctly.
//--------------------------------------------------
void processAlertTone(uint8_t tone_pin)
{
    if (!g_alert_tone_is_active)
    {
        // If additional audio types are added in the future, handle them here.
        // For now, just return.
        return;
    }

    unsigned long now = millis();

    //--------------------------------------------------
    // Check total runtime
    //--------------------------------------------------
    if ((g_tone_duration > 0) &&
        (now - g_tone_start_time >= g_tone_duration))
    {
        forceQuitTone(tone_pin);
        return;
    }

    //--------------------------------------------------
    // Time to switch segments?
    //--------------------------------------------------
    if (g_tone_segment_duration > 0 && now - g_tone_last_segment_change >= g_tone_segment_duration)
    {
        g_tone_last_segment_change = now;

        g_tone_playing_first_segment = !g_tone_playing_first_segment;

        // If both toneFrequency1 and g_tone_frequency_02 are zero, noTone will be called in both segments.
        if (g_tone_playing_first_segment)
        {
            if (g_tone_frequency_01 > 0)
            {
                tone(tone_pin, g_tone_frequency_01);
            }
            else
            {
                noTone(tone_pin);
            }
        }
        else
        {
            if (g_tone_frequency_02 > 0)
            {
                tone(tone_pin, g_tone_frequency_02);
            }
            else
            {
                noTone(tone_pin);
            }
        }
    }
}

//--------------------------------------------------
// FORCE QUIT TONE
// The forceQuitTone function is responsible for immediately stopping the tone and resetting the audio state.
// It ensures that the speaker is turned off and all relevant variables are cleared.
//--------------------------------------------------
void forceQuitTone(uint8_t tone_pin)
{
    g_alert_tone_is_active = false;

    g_tone_start_time = 0;
    g_tone_duration = 0;

    g_tone_frequency_01 = 0;
    g_tone_frequency_02 = 0;

    g_tone_segment_duration = 0;
    g_tone_last_segment_change = 0;

    g_tone_playing_first_segment = true;

    noTone(tone_pin);

    Serial.println("Speaker OFF");
}

//--------------------------------------------------
// ALERT PROCESSING
//--------------------------------------------------
// ALERT TRIPPED
// The handleAlertTripped function is responsible for managing the alert tripped event by printing an "ALERT TRIPPED" message to the serial output.
// It updates the switch and alarm states, starts a tone on the specified speaker pin, and calls the displayAlertTripped function to update the display.
//--------------------------------------------------
void handleAlertTripped()
{
    Serial.println("ALERT TRIPPED");

    g_last_reported_trap_state = TRIPPED;
    g_switch_is_open = true;
    g_alarm_is_active = true;

    startToneFor(SPEAKER_PIN,TONE_A4, TONE_A5, 300, 0);

    displayAlertTripped();
}

//--------------------------------------------------
// ALERT SET
// The handleAlertSet function is responsible for managing the alert set event by printing an "ALERT SET" message to the serial output.
// It updates the switch and alarm states, stops any active alert tone, and calls the displaySet function to update the display.
//--------------------------------------------------
void handleAlertSet()
{
    Serial.println("ALERT SET");

    g_last_reported_trap_state = SET;
    g_switch_is_open = false;
    g_alarm_is_active = false;

    if (g_alert_tone_is_active)
    {
        forceQuitTone(SPEAKER_PIN);
    }
    displaySet();
}

//--------------------------------------------------
// STATUS PROCESSING
//--------------------------------------------------

//--------------------------------------------------
// STATUS TRIPPED
// The handleStatusTripped function is responsible for managing the status tripped event by printing a "STATUS TRIPPED" message to the serial output.
// It updates the switch state, starts a tone on the specified speaker pin, and calls the displayTripped function to update the display.
//--------------------------------------------------
void handleStatusTripped()
{
    Serial.println("STATUS TRIPPED");

    g_last_reported_trap_state = TRIPPED;
    g_switch_is_open = true;

    startToneFor(SPEAKER_PIN, TONE_A4, TONE_A5, 300, 0);

    displayTripped();
}


//--------------------------------------------------
// STATUS SET
// The handleStatusSet function is responsible for managing the status set event by printing a "STATUS SET" message to the serial output.
// It updates the switch and alarm states, stops any active alert tone, and calls the displaySet function to update the display.
//--------------------------------------------------
void handleStatusSet()
{
    Serial.println("STATUS SET");

    g_last_reported_trap_state = SET;
    g_switch_is_open = false;
    g_alarm_is_active = false;

    if (g_alert_tone_is_active)
    {
        forceQuitTone(SPEAKER_PIN);
    }
    displaySet();
}

//--------------------------------------------------
// TIMEOUT
// The handleTimeout function is responsible for managing timeout events by printing a "HEARTBEAT TIMEOUT" message to the serial output.
// If an alarm is not active, it initiates a tone on a specified speaker pin and subsequently calls the displayTimeout function to update the display.
//--------------------------------------------------
void handleTimeout()
{
    Serial.println("HEARTBEAT TIMEOUT");

    switch (g_last_reported_trap_state)
    {
        case TRIPPED:
        case TRIPPED_TIMEOUT:
            g_last_reported_trap_state = TRIPPED_TIMEOUT;
            break;
        case SET:
        case SET_TIMEOUT:
            g_last_reported_trap_state = SET_TIMEOUT;
            break;
        default:
            g_last_reported_trap_state = TIMEOUT;
            break;
    }

    if (!g_alarm_is_active)
    {
        startToneFor(SPEAKER_PIN, TONE_A3, 0, 300, TIMEOUT_TONE_DURATION_MS);
    }

    displayTimeout();
}

//--------------------------------------------------
// Function to display text on the LED matrix
// The displayMatrixText function is responsible for rendering text on the LED matrix display.
// Parameters:
//   matrix - The LED matrix object on which to display the text.
//   text   - The text to be displayed.
// Returns:
//   None.
//--------------------------------------------------
void displayMatrixText(ArduinoLEDMatrix &matrix, const char *text)
{
    // Begin drawing on the LED matrix
    matrix.beginDraw();

    // Set the stroke color to white (0xFFFFFFFF)
    matrix.stroke(0xFFFFFFFF);

    // Set the font for the text
    matrix.textFont(Font_4x6);

    // Begin text rendering at position (0, 1) with a foreground color of white (0xFFFFFF)
    matrix.beginText(0, 1, 0xFFFFFF);

    // Print the provided text
    matrix.println(text);

    // End text rendering
    matrix.endText();

    // Finish drawing and update the display
    matrix.endDraw();
}

//--------------------------------------------------
// DISPLAY TRIPPED
// The displayTripped function is responsible for updating the LED matrix display to indicate that the status is tripped.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displayTripped()
{
    Serial.println("Display Tripped Status");
    if (g_is_AP_mode)
    {
        g_matrix.loadSequence(LIVE_CAUGHT);
    }
    else
    {
        g_matrix.loadSequence(DEBUG_CAUGHT);
    }
    g_matrix.play(true);
}

//--------------------------------------------------
// DISPLAY SET
// The displaySet function is responsible for updating the LED matrix display to indicate that the status is set.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displaySet()
{
    Serial.println("Display Set Status");
    if (g_is_AP_mode)
    {
        g_matrix.loadSequence(LIVE_HEARTBEAT);
    }
    else
    {
        g_matrix.loadSequence(DEBUG_HEARTBEAT);
    }
    g_matrix.play(true);
}

//--------------------------------------------------
// DISPLAY ALERT TRIPPED
// The displayAlertTripped function is responsible for updating the LED matrix display to indicate that an alert has been tripped.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displayAlertTripped()
{
    Serial.println("Display Tripped Alert");
    if (g_is_AP_mode)
    {
        g_matrix.loadSequence(LIVE_CAUGHT);
    }
    else
    {
        g_matrix.loadSequence(DEBUG_CAUGHT);
    }
    g_matrix.play(true);
}

//--------------------------------------------------
// DISPLAY TIMEOUT
// The displayTimeout function is responsible for updating the LED matrix display to indicate that a timeout has occurred.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displayTimeout()
{
    Serial.println("Display timeout");
    if (g_is_AP_mode)
    {
        g_matrix.loadSequence(LIVE_NO_HEARTBEAT);
    }
    else
    {
        g_matrix.loadSequence(DEBUG_NO_HEARTBEAT);
    }
    g_matrix.play(true);
}

//--------------------------------------------------
// CLEAR TIMEOUT INDICATION
// The clearTimeoutIndication function is responsible for clearing the timeout indication on the LED matrix display.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void clearTimeoutIndication()
{
    Serial.println("Clear timeout indication");
    #if false
    if (isAPMode)
    {
        matrix.loadSequence(LIVE_NO_HEARTBEAT);
    }
    else
    {
        matrix.loadSequence(DEBUG_NO_HEARTBEAT);
    }
    matrix.play(true);
    #endif
}

//--------------------------------------------------
// WIFI STATUS TO STRING
// The wifiStatusToString function converts a WiFi status code to a human-readable string.
//--------------------------------------------------
const char *wifiStatusToString(wl_status_t status)
{
    switch (status)
    {
    case WL_NO_SHIELD:
        return "WL_NO_SHIELD";
    case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
        return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    case WL_AP_LISTENING:
        return "WL_AP_LISTENING";
    case WL_AP_CONNECTED:
        return "WL_AP_CONNECTED";
    case WL_AP_FAILED:
        return "WL_AP_FAILED";
    default:
        return "UNKNOWN";
    }
}

//--------------------------------------------------
// CURRENT STATE TO STRING
// The currentTrapStateToString function converts a current state code to a human-readable string.
//--------------------------------------------------
const char *currentTrapStateToString(CurrentState state)
{
    switch (state)
    {
    case NONE:
        return "NONE";
    case TRIPPED:
        return "TRIPPED";
    case SET:
        return "SET";
    case TIMEOUT:
        return "TIMEOUT";
    case TRIPPED_TIMEOUT:
        return "TRIPPED_TIMEOUT";
    case SET_TIMEOUT:
        return "SET_TIMEOUT";
    default:
        return "UNKNOWN";
    }
}

//--------------------------------------------------
// DUMP WIFI STATUS
// The dumpWifiStatus function prints the current WiFi status to the serial monitor.
//--------------------------------------------------
void dumpWifiStatus(wl_status_t status)
{
    Serial.print("WiFi.status() = ");
    Serial.print((int)status);
    Serial.print(" (");
    Serial.print(wifiStatusToString(status));
    Serial.println(")");
}

void updateRssiStats(long rssi)
{
    g_rssi_latest = rssi;

    if (rssi < g_rssi_min)
    {
        g_rssi_min = rssi;
    }
    else
    {
        if (rssi > g_rssi_max)
        {
            g_rssi_max = rssi;
        }
    }
}

//-------------------------------------------------------------
// The parseLong function attempts to convert a string to a long integer, storing the result in the provided pointer.
// It returns true if the conversion is successful and valid, and false if the input string is empty, contains invalid
// characters, or results in an overflow or underflow.
//-------------------------------------------------------------
bool parseLong(const char *str, long *result)
{
    char *endptr;

    // Check for empty string
    if (!str || *str == '\0')
    {
        return false;
    }

    errno = 0;
    *result = strtol(str, &endptr, 10);

    // Check for conversion errors
    if (errno == ERANGE)
    {
        return false; // Overflow or underflow
    }

    // Check if any conversion occurred
    if (endptr == str)
    {
        return false; // No digits found
    }

    // Check for trailing garbage (only allow whitespace/null after number)
    // Note: *endptr == '\0' ensures no trailing letters/digits
    if (*endptr != '\0')
    {
        return false; // Trailing garbage found
    }

    return true;
}
