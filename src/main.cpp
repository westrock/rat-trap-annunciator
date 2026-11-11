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

unsigned long lastNetworkCheck = 0;
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

currentState_t currentState = NONE;

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

    handle_network_transition(g_is_AP_mode);
    g_last_update_millis = millis();
}

//--------------------------------------------------
// LOOP
// The main loop function is responsible for continuously processing network health, handling HTTP requests, managing timeouts, and playing audio sequences.
//--------------------------------------------------

void loop()
{
    process_network_health();
    process_http_requests();
    process_timeout();
    process_alert_tone(SPEAKER_PIN);
}

//--------------------------------------------------
// NETWORK HEALTH
// The process_network_health function is responsible for monitoring the network status and handling any necessary transitions between AP and STA modes.
// It periodically checks the network status and triggers appropriate actions if the network is not functioning correctly.
// This function should be called regularly in the main loop to ensure the network health is continuously monitored.
//--------------------------------------------------
void process_network_health()
{
    // mdns.run();
    if (millis() - lastNetworkCheck < NETWORK_CHECK_INTERVAL_MS)
    {
        return;
    }

    lastNetworkCheck = millis();

    if (!g_is_network_initialized)
    {
        Serial.println("Network not initialized. Retrying...");
        handle_network_transition(g_is_AP_mode);
        return;
    }

    uint8_t status = WiFi.status();
    if (g_is_AP_mode)
    {
        if (status != WL_AP_LISTENING && status != WL_AP_CONNECTED)
        {
            Serial.print("AP ");
            dump_wifi_status((wl_status_t)status);

            Serial.println("AP lost. Restarting...");
            handle_network_transition(true);
        }
    }
    else
    {
        if (status != WL_CONNECTED)
        {
            Serial.println("STA disconnected. Reconnecting...");
            handle_network_transition(false);
        }
    }
}

//--------------------------------------------------
// HTTP SERVER
// The HTTP server handles incoming requests and routes them to the appropriate handler functions based on the URI.
// It supports endpoints for status, alert, and other custom actions.
//--------------------------------------------------
bool get_state_parameter(const char *uri, char *key, char *value, size_t valueSize)
{
    const char *keyValuePtr = strstr(uri, key);

    if (keyValuePtr == NULL)
    {
        value[0] = '\0';
        return false;
    }

    keyValuePtr += strlen(key);

    size_t i = 0;

    while (*keyValuePtr &&
           *keyValuePtr != '&' &&
           i < valueSize - 1)
    {
        value[i++] = *keyValuePtr++;
    }

    value[i] = '\0';

    return true;
}

//--------------------------------------------------
// REQUEST PROCESSING
// The process_request_uri function is responsible for handling the incoming HTTP requests and routing them to the appropriate handler functions based on the URI and state parameter.
// The get_state_parameter function is used to extract the state parameter from the URI string. It returns true if the state parameter is found and false otherwise.
// The MAX_STATE_LEN constant defines the maximum length of the state parameter that can be extracted from the URI.
// The process_request_uri function should be called for each incoming HTTP request to ensure proper routing and handling of the request.
// The get_state_parameter function should be used within process_request_uri to extract the state parameter from the URI before routing the request to the appropriate handler.
// The HTTP server should be configured to call process_request_uri for each incoming request to ensure proper handling and routing.
//--------------------------------------------------

void process_request_uri(WiFiClient &client, const char *uri)
{
    static char responseBuffer[512];
    char clienIPAddress[16];

                reset_heartbeat_timer();

    strncpy(clienIPAddress, client.remoteIP().toString().c_str(), sizeof(clienIPAddress));

    if (strncasecmp(uri, "/report", 7) == 0)
    {
        //--------------------------------------------------
        // REPORT
        // The report endpoint provides the current state of the system.
        // The response includes the current state as a string.
        // The currentState variable holds the current state of the system, which is then converted to a string for the response.
        // The currentStateText variable holds the string representation of the current state, which is then used to construct the HTTP response.
        // The responseBuffer variable holds the complete HTTP response, including the current state, which is then sent to the client.
        // The send_http_response function is used to send the HTTP response to the client.
        // The client.stop function is used to close the connection to the client after sending the response.
        // The handle_server_error function is used to handle any errors that occur while preparing the HTTP response.
        // The reset_heartbeat_timer function WILL NOT reset the heartbeat timer after handling the report request.
        //--------------------------------------------------
        int written = -1;
        if (g_is_AP_mode)
        {
            written = snprintf(responseBuffer,
                               sizeof(responseBuffer),
                               "Current State: %s\r\nIP Address: %s\r\nSignal strength (RSSI): N.A.\r\nRemote IP Address: %s\r\nRemote (RSSI) current/min/max: %ld/%ld/%ld dBm",
                               current_state_to_string(currentState),
                               WiFi.localIP().toString().c_str(),
                               clienIPAddress,
                               g_rssi_latest, g_rssi_min, g_rssi_max);
        }
        else
        {
            written = snprintf(responseBuffer,
                               sizeof(responseBuffer),
                               "Current State: %s\r\nIP Address: %s\r\nSignal strength (RSSI): %ld dBm\r\nRemote IP Address: %s\r\nRemote (RSSI) current/min/max: %ld/%ld/%ld dBm",
                               current_state_to_string(currentState),
                               WiFi.localIP().toString().c_str(),
                               WiFi.RSSI(),
                               clienIPAddress,
                               g_rssi_latest, g_rssi_min, g_rssi_max);
        }

        if (written < 0 || written >= sizeof(responseBuffer))
        {
            handle_server_error(client, "Unexpected buffer overflow preparing HTTP response.");
            return;
        }
        Serial.println("HTTP REPORT request received. Sending response:");
        Serial.println(responseBuffer);
        send_http_response(client, 200, REASON_PHRASE_200_OK, responseBuffer);
    }
    else
    {
        char state[MAX_STATE_LEN];
        char rssi[MAX_RSSI_LEN];

        if (!get_state_parameter(uri, "state=", state, sizeof(state)))
        {
            Serial.print("Missing state parameter. URI: ");
            Serial.println(uri);
            send_http_response(client, 400, REASON_PHRASE_400_BadRequest, "Missing Status/Alert State");
            return;
        }

        get_state_parameter(uri, "rssi=", rssi, sizeof(rssi));

        if (strlen(rssi) > 0)
        {
            long rssiValue;
            if (parse_long(rssi, &rssiValue))
            {
                update_rssi_stats(rssiValue);
            }
            else
            {
                Serial.print("Invalid RSSI value: ");
                Serial.println(rssi);
            }
        }

        currentState_t theState = (strcasecmp(state, "tripped") == 0) ? TRIPPED : (strcasecmp(state, "set") == 0 ? SET : NONE);
        bool isStatus = strncasecmp(uri, "/status", 7) == 0;
        bool isAlert = strncasecmp(uri, "/alert", 6) == 0;

        if (isStatus || isAlert)
        {
            switch (theState)
            {
            case TRIPPED:
                handle_status_tripped();
                send_http_response(client, RESPONSE_CODE_200, REASON_PHRASE_200_OK, isStatus ? "Status Tripped OK" : "Alert Tripped OK");
                break;
            case SET:
                handle_status_set();
                send_http_response(client, RESPONSE_CODE_200, REASON_PHRASE_200_OK, isStatus ?" Status Set OK" : "Alert Set OK");
                break;
            case NONE:
            default:
                Serial.println(isStatus ? "Invalid Status State" : "Invalid Alert State");
                send_http_response(client, RESPONSE_CODE_400, REASON_PHRASE_400_BadRequest, isStatus ? "Invalid Status State" : "Invalid Alert State");
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

            send_http_response(client, RESPONSE_CODE_404, REASON_PHRASE_404_NotFound, "Unknown URI");
        }
    }
}

//--------------------------------------------------
// HTTP REQUEST HANDLING
// The process_http_requests function is responsible for handling incoming HTTP requests, parsing the request line, and routing the request
// to the appropriate handler function.
//--------------------------------------------------
void process_http_requests()
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

    unsigned long startTime = millis();

    while (!client.available())
    {
        if (millis() - startTime > 1000)
        {
            Serial.println("Client data timeout");
            client.stop();
            return;
        }

        delay(1);
    }

    char requestLine[MAX_REQUEST_LINE];

    size_t len = client.readBytesUntil(
        '\n',
        requestLine,
        sizeof(requestLine) - 1);
        
    requestLine[len] = '\0';

    //--------------------------------------------------
    // Trim trailing CR if present
    //--------------------------------------------------

    if (len > 0 &&
        requestLine[len - 1] == '\r')
    {
        requestLine[len - 1] = '\0';
        --len;
    }

    //--------------------------------------------------
    // Timeout / empty request
    //--------------------------------------------------

    if (len == 0)
    {
        Serial.println("HTTP request timeout");
        send_http_response(client, RESPONSE_CODE_408, REASON_PHRASE_408_RequestTimeout, "Request Timeout");

        return;
    }

    //--------------------------------------------------
    // Detect possible truncation
    //--------------------------------------------------

    if (len >= sizeof(requestLine) - 1)
    {
        Serial.println("HTTP request line too long");
        send_http_response(client, RESPONSE_CODE_414, REASON_PHRASE_414_URITooLong, "URI Too Long");
    
        return;
    }

    //--------------------------------------------------
    // Parse request line
    //--------------------------------------------------

    char method[MAX_METHOD_LEN];
    char uri[MAX_URI_LEN];

    int fields = sscanf(requestLine, "%7s %127s", method, uri);

    if (fields != 2)
    {
        Serial.println("Malformed HTTP request");
        send_http_response(client, RESPONSE_CODE_400, REASON_PHRASE_400_BadRequest, "Malformed request");

        return;
    }

    //--------------------------------------------------
    // Verify HTTP method
    //--------------------------------------------------

    if (strcasecmp(method, "GET") != 0)
    {
        Serial.print("Unsupported method: ");
        Serial.println(method);

        send_http_response(client, RESPONSE_CODE_405, REASON_PHRASE_405_MethodNotAllowed, "HTTP method not supported");
 
        return;
    }

    //--------------------------------------------------
    // Ignore browser favicon requests
    //--------------------------------------------------

    if (strncasecmp(uri, "/favicon.ico", 12) == 0)
    {
        send_http_response(client, RESPONSE_CODE_404, REASON_PHRASE_404_NotFound, "Not Found");
        return;
    }

    //--------------------------------------------------
    // Application request processing
    //--------------------------------------------------

    process_request_uri(client, uri);

    if (client.connected())
    {
        client.flush();
        delay(10);
        client.stop();
    }
}

//--------------------------------------------------
// SERVER ERROR HANDLING
// The handle_server_error function is responsible for sending a 500 Internal Server Error response to the client.
//--------------------------------------------------
void handle_server_error(WiFiClient &client, const char *message)
{
    send_http_response(client, RESPONSE_CODE_500, REASON_PHRASE_500_InternalServerError, message);
}

//--------------------------------------------------
// HTTP RESPONSE SENDING
// The send_http_response function is responsible for sending an HTTP response to the client with the specified status code, reason phrase, and message.
//--------------------------------------------------
void send_http_response(WiFiClient &client, int statusCode, const char *reasonPhrase, const char *message)
{
    static char responseBuffer[1024];

    int written = snprintf(responseBuffer, sizeof(responseBuffer), "HTTP/1.1 %d %s", statusCode, reasonPhrase);

    if (written < 0 || written >= sizeof(responseBuffer))
    {
        handle_server_error(client, "Unexpected buffer overflow preparing HTTP response.");
        return;
    }

    size_t bodyLength = (message != nullptr) ? strlen(message) : 0;

    client.println(responseBuffer);
    client.println("Content-Type: text/plain");

    client.print("Content-Length: ");
    client.println(bodyLength);

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
// The handle_network_transition function is responsible for switching the network mode between Access Point (AP) and Station (STA) modes.
// The function takes a boolean parameter 'makeAP' to determine whether to switch to Access Point mode or Station mode.
// The function also updates the 'g_is_network_initialized' flag to reflect the current network state.
// The function handles both the AP and STA modes, including establishing the connection, setting up the server, and updating the network state.
// The function ensures that the network transition is handled gracefully, with appropriate error handling and status reporting.
//--------------------------------------------------
void handle_network_transition(bool makeAP)
{
    Serial.println("\n--- Setting Network Mode ---");

    // 1. Cleanup existing state to prevent memory leaks and conflicts
    WiFi.disconnect();
    // server.stop(); // If your server library supports a stop method

    g_is_network_initialized = false;

    if (makeAP)
    {
        Serial.println("Mode: Access Point (AP)");
        Serial.print("Firmware version: ");
        Serial.println(WiFi.firmwareVersion());
        int rc = WiFi.beginAP(ap_ssid, ap_pass);

        Serial.print("beginAP returned: ");
        Serial.println(rc);

        delay(2000);

        wl_status_t status = (wl_status_t)WiFi.status();
        dump_wifi_status(status);

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

void handleNetworkTransition_new(bool makeAP)
{
    Serial.println("\n--- Transitioning Network Mode ---");

    // 1. Cleanup existing state to prevent memory leaks and conflicts
    WiFi.disconnect();
    // server.stop(); // If your server library supports a stop method

    g_is_network_initialized = false;

    if (makeAP)
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
// The reset_heartbeat_timer function resets the heartbeat timer, indicating that the system is active.
//--------------------------------------------------
void reset_heartbeat_timer()
{
    g_last_update_millis = millis();

    if (g_timeout_is_active)
    {
        g_timeout_is_active = false;

        Serial.println("Heartbeat restored");

        clear_timeout_indication();
    }
}

//--------------------------------------------------
// The process_timeout function is responsible for handling the heartbeat timeout.
// It checks if the time since the last update has exceeded the heartbeat timeout and triggers the appropriate actions if necessary.
//--------------------------------------------------
void process_timeout()
{
    // Check if the heartbeat timeout has been reached, even if we are already in a timeout state.
    if (millis() - g_last_update_millis >= HEARTBEAT_TIMEOUT_MS)
    {
        g_timeout_is_active = true;
        g_last_update_millis = millis();
        handle_timeout();
    }
}

//--------------------------------------------------
// AUDIO
//--------------------------------------------------
// AUDIO MANAGEMENT
// The start_tone_for function initiates a tone sequence on the specified pin with the given frequencies and durations.
// The process_alert_tone function handles the progression of the tone sequence, switching between segments and stopping the tone when the total duration is reached.
//--------------------------------------------------
void start_tone_for(uint8_t tonePin, unsigned int frequency1, unsigned int frequency2, unsigned long segmentDurationMs, unsigned long durationMs)
{
    g_tone_frequency_01 = frequency1;
    g_tone_frequency_02 = frequency2;

    g_tone_segment_duration = segmentDurationMs;

    g_tone_duration = durationMs;
    g_tone_start_time = millis();
    g_tone_last_segment_change = g_tone_start_time;

    g_tone_playing_first_segment = true;
    g_alert_tone_is_active = true;

    if (frequency1 > 0)
    {
        tone(tonePin, frequency1);
    }
    else
    {
        noTone(tonePin);
    }
}

//--------------------------------------------------
// AUDIO PROCESSING
// The process_alert_tone function is responsible for managing the audio output, including switching between tone segments and stopping the tone when the total duration is reached.
// It should be called regularly in the main loop to ensure the audio sequence progresses correctly.
//--------------------------------------------------
void process_alert_tone(uint8_t tonePin)
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
        force_quit_tone(tonePin);
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
                tone(tonePin, g_tone_frequency_01);
            }
            else
            {
                noTone(tonePin);
            }
        }
        else
        {
            if (g_tone_frequency_02 > 0)
            {
                tone(tonePin, g_tone_frequency_02);
            }
            else
            {
                noTone(tonePin);
            }
        }
    }
}

//--------------------------------------------------
// FORCE QUIT TONE
// The force_quit_tone function is responsible for immediately stopping the tone and resetting the audio state.
// It ensures that the speaker is turned off and all relevant variables are cleared.
//--------------------------------------------------
void force_quit_tone(uint8_t tonePin)
{
    g_alert_tone_is_active = false;

    g_tone_start_time = 0;
    g_tone_duration = 0;

    g_tone_frequency_01 = 0;
    g_tone_frequency_02 = 0;

    g_tone_segment_duration = 0;
    g_tone_last_segment_change = 0;

    g_tone_playing_first_segment = true;

    noTone(tonePin);

    Serial.println("Speaker OFF");
}

//--------------------------------------------------
// ALERT PROCESSING
//--------------------------------------------------
// ALERT TRIPPED
// The handle_alert_tripped function is responsible for managing the alert tripped event by printing an "ALERT TRIPPED" message to the serial output.
// It updates the switch and alarm states, starts a tone on the specified speaker pin, and calls the display_alert_tripped function to update the display.
//--------------------------------------------------
void handle_alert_tripped()
{
    Serial.println("ALERT TRIPPED");

    currentState = TRIPPED;
    g_switch_is_open = true;
    g_alarm_is_active = true;

    start_tone_for(SPEAKER_PIN,TONE_A4, TONE_A5, 300, 0);

    display_alert_tripped();
}

//--------------------------------------------------
// ALERT SET
// The handle_alert_set function is responsible for managing the alert set event by printing an "ALERT SET" message to the serial output.
// It updates the switch and alarm states, stops any active alert tone, and calls the display_set function to update the display.
//--------------------------------------------------
void handle_alert_set()
{
    Serial.println("ALERT SET");

    currentState = SET;
    g_switch_is_open = false;
    g_alarm_is_active = false;

    if (g_alert_tone_is_active)
    {
        force_quit_tone(SPEAKER_PIN);
    }
    display_set();
}

//--------------------------------------------------
// STATUS PROCESSING
//--------------------------------------------------

//--------------------------------------------------
// STATUS TRIPPED
// The handle_status_tripped function is responsible for managing the status tripped event by printing a "STATUS TRIPPED" message to the serial output.
// It updates the switch state, starts a tone on the specified speaker pin, and calls the display_tripped function to update the display.
//--------------------------------------------------
void handle_status_tripped()
{
    Serial.println("STATUS TRIPPED");

    currentState = TRIPPED;
    g_switch_is_open = true;

    start_tone_for(SPEAKER_PIN, TONE_A4, TONE_A5, 300, 0);

    display_tripped();
}


//--------------------------------------------------
// STATUS SET
// The handle_status_set function is responsible for managing the status set event by printing a "STATUS SET" message to the serial output.
// It updates the switch and alarm states, stops any active alert tone, and calls the display_set function to update the display.
//--------------------------------------------------
void handle_status_set()
{
    Serial.println("STATUS SET");

    currentState = SET;
    g_switch_is_open = false;
    g_alarm_is_active = false;

    if (g_alert_tone_is_active)
    {
        force_quit_tone(SPEAKER_PIN);
    }
    display_set();
}

//--------------------------------------------------
// TIMEOUT
// The handle_timeout function is responsible for managing timeout events by printing a "HEARTBEAT TIMEOUT" message to the serial output.
// If an alarm is not active, it initiates a tone on a specified speaker pin and subsequently calls the display_timeout function to update the display.
//--------------------------------------------------
void handle_timeout()
{
    Serial.println("HEARTBEAT TIMEOUT");

    switch (currentState)
    {
        case TRIPPED:
        case TRIPPED_TIMEOUT:
            currentState = TRIPPED_TIMEOUT;
            break;
        case SET:
        case SET_TIMEOUT:
            currentState = SET_TIMEOUT;
            break;
        default:
            currentState = TIMEOUT;
            break;
    }

    if (!g_alarm_is_active)
    {
        start_tone_for(SPEAKER_PIN, TONE_A3, 0, 300, TIMEOUT_TONE_DURATION_MS);
    }

    display_timeout();
}

//--------------------------------------------------
// Function to display text on the LED matrix
// The display_text function is responsible for rendering text on the LED matrix display.
// Parameters:
//   matrix - The LED matrix object on which to display the text.
//   text   - The text to be displayed.
// Returns:
//   None.
//--------------------------------------------------
void display_text(ArduinoLEDMatrix &matrix, const char *text)
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
// The display_tripped function is responsible for updating the LED matrix display to indicate that the status is tripped.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void display_tripped()
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
// The display_set function is responsible for updating the LED matrix display to indicate that the status is set.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void display_set()
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
// The display_alert_tripped function is responsible for updating the LED matrix display to indicate that an alert has been tripped.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void display_alert_tripped()
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
// The display_timeout function is responsible for updating the LED matrix display to indicate that a timeout has occurred.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void display_timeout()
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
// The clear_timeout_indication function is responsible for clearing the timeout indication on the LED matrix display.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void clear_timeout_indication()
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
// The wifi_status_to_string function converts a WiFi status code to a human-readable string.
//--------------------------------------------------
const char *wifi_status_to_string(wl_status_t status)
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
// The current_state_to_string function converts a current state code to a human-readable string.
//--------------------------------------------------
const char *current_state_to_string(currentState_t state)
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
// The dump_wifi_status function prints the current WiFi status to the serial monitor.
//--------------------------------------------------
void dump_wifi_status(wl_status_t status)
{
    Serial.print("WiFi.status() = ");
    Serial.print((int)status);
    Serial.print(" (");
    Serial.print(wifi_status_to_string(status));
    Serial.println(")");
}

void update_rssi_stats(long rssi)
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

bool parse_long(const char *str, long *result)
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
