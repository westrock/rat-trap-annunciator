#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <ArduinoGraphics.h>
#include <ArduinoMDNS.h>
#include "Arduino_LED_Matrix.h" // Include the LED_Matrix library
#include "animations.h"
#include "main.h"
#include "secrets.h"

//--------------------------------------------------
// CONFIGURATION
//--------------------------------------------------
#define MAX_REQUEST_LINE 256
#define MAX_STATE_LEN 16
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

const int MODE_TEST_PIN = 7; // Pin that will set the level
const int MODE_PIN = 8;
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

//--------------------------------------------------
// GLOBALS
//--------------------------------------------------

ArduinoLEDMatrix matrix;
WiFiUDP udp;
MDNS mdns(udp);
WiFiServer server(80);

currentState_t currentState = NONE;

bool isAPMode = false;
bool networkInitialized = false;

unsigned long lastUpdateMillis = 0;

bool switchOpen = false;
bool alarmActive = false;
bool timeoutActive = false;

bool alertToneActive = false;
unsigned long toneStopTime = 0;
unsigned long toneStartTime = 0;
unsigned long toneDuration = 0;
unsigned int toneFrequency1 = 0;
unsigned int toneFrequency2 = 0;

unsigned long toneSegmentDuration = 0;
unsigned long toneLastSegmentChange = 0;

bool tonePlayingFirstSegment = true;
//--------------------------------------------------
// SETUP
//--------------------------------------------------
void setup()
{
    Serial.begin(115200);
    while (!Serial);
    
    // Set pin 7 as output and Enable internal pull-up on pin 8 (INPUT_PULLUP)
    pinMode(MODE_TEST_PIN, OUTPUT);
    pinMode(MODE_PIN, INPUT_PULLUP);

    matrix.begin();
    matrix.loadSequence(STARTUP);
    matrix.play(true);

    digitalWrite(MODE_TEST_PIN, LOW);
    delay(5);
    isAPMode = (digitalRead(MODE_PIN) == HIGH);

    Serial.println("System Initializing...");

    handleNetworkTransition(isAPMode);
    lastUpdateMillis = millis();
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
    processAudio(SPEAKER_PIN);
}

//--------------------------------------------------
// NETWORK HEALTH
// The processNetworkHealth function is responsible for monitoring the network status and handling any necessary transitions between AP and STA modes.
// It periodically checks the network status and triggers appropriate actions if the network is not functioning correctly.
// This function should be called regularly in the main loop to ensure the network health is continuously monitored.
//--------------------------------------------------
void processNetworkHealth()
{
    mdns.run();
    if (millis() - lastNetworkCheck < NETWORK_CHECK_INTERVAL_MS)
    {
        return;
    }

    lastNetworkCheck = millis();

    if (!networkInitialized)
    {
        Serial.println("Network not initialized. Retrying...");
        handleNetworkTransition(isAPMode);
        return;
    }

    uint8_t status = WiFi.status();
    if (isAPMode)
    {
        if (status != WL_AP_LISTENING && status != WL_AP_CONNECTED)
        {
            Serial.print("AP ");
            dumpWiFiStatus((wl_status_t)status);

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
// The HTTP server handles incoming requests and routes them to the appropriate handler functions based on the URI.
// It supports endpoints for status, alert, and other custom actions.
//--------------------------------------------------
bool getStateParameter(const char *uri,
                       char *state,
                       size_t stateSize)
{
    const char *p = strstr(uri, "state=");

    if (p == NULL)
    {
        state[0] = '\0';
        return false;
    }

    p += strlen("state=");

    size_t i = 0;

    while (*p &&
           *p != '&' &&
           i < stateSize - 1)
    {
        state[i++] = *p++;
    }

    state[i] = '\0';

    return true;
}

//--------------------------------------------------
// REQUEST PROCESSING
// The processRequest function is responsible for handling the incoming HTTP requests and routing them to the appropriate handler functions based on the URI and state parameter.
// The getStateParameter function is used to extract the state parameter from the URI string. It returns true if the state parameter is found and false otherwise.
// The MAX_STATE_LEN constant defines the maximum length of the state parameter that can be extracted from the URI.
// The processRequest function should be called for each incoming HTTP request to ensure proper routing and handling of the request.
// The getStateParameter function should be used within processRequest to extract the state parameter from the URI before routing the request to the appropriate handler.
// The HTTP server should be configured to call processRequest for each incoming request to ensure proper handling and routing.
//--------------------------------------------------

void processRequest(WiFiClient &client, const char *uri)
{
    static char responseBuffer[512];

    if (strncmp(uri, "/report", 7) == 0)
    {
        //--------------------------------------------------
        // REPORT
        // The report endpoint provides the current state of the system.
        // The response includes the current state as a string.
        // The currentState variable holds the current state of the system, which is then converted to a string for the response.
        // The currentStateText variable holds the string representation of the current state, which is then used to construct the HTTP response.
        // The responseBuffer variable holds the complete HTTP response, including the current state, which is then sent to the client.
        // The sendHttpResponse function is used to send the HTTP response to the client.
        // The client.stop function is used to close the connection to the client after sending the response.
        // The handleServerError function is used to handle any errors that occur while preparing the HTTP response.
        // The resetHeartbeatTimer function WILL NOT reset the heartbeat timer after handling the report request.
        //--------------------------------------------------
        int written = snprintf(responseBuffer, 
            sizeof(responseBuffer), 
            "Current State: %s\r\nIP Address: %s\r\nRemote IP Address: %s\r\nSignal strength (RSSI): %ld dBm", 
            currentStateToString(currentState), 
            WiFi.localIP().toString().c_str(),
            client.remoteIP().toString().c_str(),
            WiFi.RSSI());

        if (written < 0 || written >= sizeof(responseBuffer))
        {
            handleServerError(client, "Unexpected buffer overflow preparing HTTP response.");
            return;
        }

        sendHttpResponse(client, 200, "OK", responseBuffer);
        client.stop();
    }
    else
    {
        char state[MAX_STATE_LEN];

        if (!getStateParameter(uri,
                               state,
                               sizeof(state)))
        {
            Serial.print("Missing state parameter. URI: ");
            Serial.println(uri);
            sendHttpResponse(client, 400, "Bad Request", "Missing Status/Alert State");
            client.stop();
            return;
        }

        if (strncmp(uri, "/status", 7) == 0)
        {
            //--------------------------------------------------
            // STATUS
            //--------------------------------------------------
            if (strcmp(state, "tripped") == 0)
            {
                handleStatusTripped();
                resetHeartbeatTimer();

                sendHttpResponse(client, 200, "OK", "Status Tripped OK");
                client.stop();
            }
            else if (strcmp(state, "set") == 0)
            {
                handleStatusSet();
                resetHeartbeatTimer();

                sendHttpResponse(client, 200, "OK", "Status Set OK");
                client.stop();
            }
            else
            {
                Serial.println("Invalid STATUS state");

                sendHttpResponse(client, 400, "Bad Request", "Invalid Status State");
                client.stop();
            }
        }
        else if (strncmp(uri, "/alert", 6) == 0)
        {
            //--------------------------------------------------
            // ALERT
            //--------------------------------------------------

            if (strcmp(state, "tripped") == 0)
            {
                handleAlertTripped();
                resetHeartbeatTimer();

                sendHttpResponse(client, 200, "OK", "Alert Tripped OK");
                client.stop();
            }
            else if (strcmp(state, "set") == 0)
            {
                handleAlertSet();
                resetHeartbeatTimer();

                sendHttpResponse(client, 200, "OK", "Alert Set OK");
                client.stop();
            }
            else
            {
                Serial.println("Invalid ALERT state");

                sendHttpResponse(client, 400, "Bad Request", "Invalid Alert State");
                client.stop();
            }
        } 
        else
        {
            //--------------------------------------------------
            // UNKNOWN ENDPOINT
            //--------------------------------------------------

            Serial.print("Unknown URI: ");
            Serial.println(uri);

            sendHttpResponse(client, 404, "Not Found", "Unknown URI");
            client.stop();
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
    WiFiClient client = server.available();

    if (!client)
    {
        return;
    }

    //--------------------------------------------------
    // Configure socket timeout
    //--------------------------------------------------

    client.setTimeout(250);

    //--------------------------------------------------
    // Read first request line
    //--------------------------------------------------

    char requestLine[MAX_REQUEST_LINE];

    size_t len = client.readBytesUntil('\n', requestLine, sizeof(requestLine) - 1);

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
        sendHttpResponse(client, 408, "Request Timeout", "Request Timeout");
        client.stop();

        return;
    }

    //--------------------------------------------------
    // Detect possible truncation
    //--------------------------------------------------

    if (len >= sizeof(requestLine) - 1)
    {
        Serial.println("HTTP request line too long");
        sendHttpResponse(client, 414, "URI Too Long", "URI Too Long");
        client.stop();
    
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
        sendHttpResponse(client, 400, "Bad Request", "Malformed request");
        client.stop();

        return;
    }

    //--------------------------------------------------
    // Verify HTTP method
    //--------------------------------------------------

    if (strcmp(method, "GET") != 0)
    {
        Serial.print("Unsupported method: ");
        Serial.println(method);

        sendHttpResponse(client, 405, "Method Not Allowed", "HTTP method not supported");
        client.stop();

        return;
    }

    //--------------------------------------------------
    // Ignore browser favicon requests
    //--------------------------------------------------

    if (strncmp(uri, "/favicon.ico", 12) == 0)
    {
        sendHttpResponse(client, 404, "Not Found", "Not Found");
        client.stop();
        return;
    }

    //--------------------------------------------------
    // Application request processing
    //--------------------------------------------------

    processRequest(client, uri);
}

//--------------------------------------------------
// SERVER ERROR HANDLING
// The handleServerError function is responsible for sending a 500 Internal Server Error response to the client.
//--------------------------------------------------
void handleServerError(WiFiClient &client, const char *message)
{
    sendHttpResponse(client, 500, "Internal Server Error", message);
}

//--------------------------------------------------
// HTTP RESPONSE SENDING
// The sendHttpResponse function is responsible for sending an HTTP response to the client with the specified status code, reason phrase, and message.
//--------------------------------------------------
void sendHttpResponse(WiFiClient &client, int statusCode, const char *reasonPhrase, const char *message)
{
    static char responseBuffer[256];

    int written = snprintf(responseBuffer, sizeof(responseBuffer), "HTTP/1.1 %d %s", statusCode, reasonPhrase);

    if (written < 0 || written >= sizeof(responseBuffer))
    {
        handleServerError(client, "Unexpected buffer overflow preparing HTTP response.");
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
}

//--------------------------------------------------
// NETWORK TRANSITION
// The handleNetworkTransition function is responsible for switching the network mode between Access Point (AP) and Station (STA) modes.
// The function takes a boolean parameter 'makeAP' to determine whether to switch to Access Point mode or Station mode.
// The function also updates the 'networkInitialized' flag to reflect the current network state.
// The function handles both the AP and STA modes, including establishing the connection, setting up the server, and updating the network state.
// The function ensures that the network transition is handled gracefully, with appropriate error handling and status reporting.
//--------------------------------------------------
void handleNetworkTransition(bool makeAP)
{
    Serial.println("\n--- Setting Network Mode ---");

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
        dumpWiFiStatus(status);

        if (status == WL_NO_MODULE)
        {
            Serial.println("No WiFi module found!");
        }

        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        // if (WiFi.beginAP(ap_ssid, ap_pass) == WL_CONNECTED)
        if (status == WL_AP_LISTENING || status == WL_AP_CONNECTED)
        {
            Serial.print("AP established. IP address: ");
            Serial.println(WiFi.localIP());

            server.begin();
            Serial.println("Starting mDNS...");
            bool ok = mdns.begin(WiFi.localIP(), "ratdetector");
            Serial.println(ok ? "mDNS OK" : "mDNS FAIL");

            //isAPMode = true;
            networkInitialized = true;
        }
        else
        {
            Serial.println("Error: Failed to establish Access Point.");
            networkInitialized = false;
        }
    }
    else
    {
        Serial.println("Mode: Station (STA)");
        Serial.print("Connecting to ");
        Serial.println(sta_ssid);

        if (WiFi.begin(sta_ssid, sta_pass))
        {
            // Wait for connection (with timeout)
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20)
            {
                delay(500);
                Serial.print(".");
                attempts++;
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                IPAddress ip;
                bool haveValidIP = false;

                for (int i = 0; i < 20; i++)
                {
                    ip = WiFi.localIP();

                    if (ip != IPAddress(0, 0, 0, 0))
                    {
                        haveValidIP = true;
                        break;
                    }

                    delay(500);
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
                
                    server.begin();
                    Serial.println("Starting mDNS...");
                    bool ok = mdns.begin(WiFi.localIP(), "ratdetector");
                    Serial.println(ok ? "mDNS OK" : "mDNS FAIL");

                    //isAPMode = false;
                    networkInitialized = true;
                }
                else
                {
                    Serial.println("\nError: Failed to obtain a valid IP address.");
                    networkInitialized = false;
                }
            }
            else
            {
                Serial.println("\nError: Failed to connect to WiFi.");
                networkInitialized = false;
            }
        }
        else
        {
            Serial.println("Error: WiFi hardware not responding.");
            networkInitialized = false;
        }
    }
}

//--------------------------------------------------
// TIMER MANAGEMENT
//--------------------------------------------------
// HEARTBEAT MANAGEMENT
// The resetHeartbeatTimer function resets the heartbeat timer, indicating that the system is active.
// The processTimeout function checks if the heartbeat timeout has been reached and handles the timeout if necessary.
//--------------------------------------------------
void resetHeartbeatTimer()
{
    lastUpdateMillis = millis();

    if (timeoutActive)
    {
        timeoutActive = false;

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
    if (millis() - lastUpdateMillis >= HEARTBEAT_TIMEOUT_MS)
    {
        timeoutActive = true;
        lastUpdateMillis = millis();
        handleTimeout();
    }
}

//--------------------------------------------------
// AUDIO
//--------------------------------------------------
// AUDIO MANAGEMENT
// The startToneFor function initiates a tone sequence on the specified pin with the given frequencies and durations.
// The processAudio function handles the progression of the tone sequence, switching between segments and stopping the tone when the total duration is reached.
//--------------------------------------------------
void startToneFor(uint8_t tonePin, unsigned int frequency1, unsigned int frequency2, unsigned long segmentDurationMs, unsigned long durationMs)
{
    toneFrequency1 = frequency1;
    toneFrequency2 = frequency2;

    toneSegmentDuration = segmentDurationMs;

    toneDuration = durationMs;
    toneStartTime = millis();
    toneLastSegmentChange = toneStartTime;

    tonePlayingFirstSegment = true;
    alertToneActive = true;

    Serial.print("Speaker ON. F1=");
    Serial.print(frequency1);
    Serial.print(" F2=");
    Serial.print(frequency2);
    Serial.print(" Segment=");
    Serial.print(segmentDurationMs);
    Serial.print(" Total=");
    Serial.println(durationMs);

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
// The processAudio function is responsible for managing the audio output, including switching between tone segments and stopping the tone when the total duration is reached.
// It should be called regularly in the main loop to ensure the audio sequence progresses correctly.
//--------------------------------------------------
void processAudio(uint8_t tonePin)
{
    if (!alertToneActive)
    {
        return;
    }

    unsigned long now = millis();

    //--------------------------------------------------
    // Check total runtime
    //--------------------------------------------------
    if ((toneDuration > 0) &&
        (now - toneStartTime >= toneDuration))
    {
        forceQuitTone(tonePin);
        return;
    }

    //--------------------------------------------------
    // Time to switch segments?
    //--------------------------------------------------
    if (now - toneLastSegmentChange >= toneSegmentDuration)
    {
        toneLastSegmentChange = now;

        tonePlayingFirstSegment = !tonePlayingFirstSegment;

        if (tonePlayingFirstSegment)
        {
            if (toneFrequency1 > 0)
            {
                tone(tonePin, toneFrequency1);
            }
            else
            {
                noTone(tonePin);
            }
        }
        else
        {
            if (toneFrequency2 > 0)
            {
                tone(tonePin, toneFrequency2);
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
// The forceQuitTone function is responsible for immediately stopping the tone and resetting the audio state.
// It ensures that the speaker is turned off and all relevant variables are cleared.
//--------------------------------------------------
void forceQuitTone(uint8_t tonePin)
{
    alertToneActive = false;

    toneStartTime = 0;
    toneDuration = 0;

    toneFrequency1 = 0;
    toneFrequency2 = 0;

    toneSegmentDuration = 0;
    toneLastSegmentChange = 0;

    tonePlayingFirstSegment = true;

    noTone(tonePin);

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

    currentState = TRIPPED;
    switchOpen = true;
    alarmActive = true;

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

    currentState = SET;
    switchOpen = false;
    alarmActive = false;

    if (alertToneActive)
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

    currentState = TRIPPED;
    switchOpen = true;

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

    currentState = SET;
    switchOpen = false;
    alarmActive = false;

    if (alertToneActive)
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

    if (!alarmActive)
    {
        startToneFor(SPEAKER_PIN, TONE_A3, 0, 300, TIMEOUT_TONE_DURATION_MS);
    }

    displayTimeout();
}

//--------------------------------------------------
// Function to display text on the LED matrix
// The displayText function is responsible for rendering text on the LED matrix display.
// Parameters:
//   matrix - The LED matrix object on which to display the text.
//   text   - The text to be displayed.
// Returns:
//   None.
//--------------------------------------------------
void displayText(ArduinoLEDMatrix &matrix, const char *text)
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
    if (isAPMode)
    {
        matrix.loadSequence(LIVE_CAUGHT);
    }
    else
    {
        matrix.loadSequence(DEBUG_CAUGHT);
    }
    matrix.play(true);
}

//--------------------------------------------------
// DISPLAY SET
// The displaySet function is responsible for updating the LED matrix display to indicate that the status is set.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displaySet()
{
    Serial.println("Display Set Status");
    if (isAPMode)
    {
        matrix.loadSequence(LIVE_HEARTBEAT);
    }
    else
    {
        matrix.loadSequence(DEBUG_HEARTBEAT);
    }
    matrix.play(true);
}

//--------------------------------------------------
// DISPLAY ALERT TRIPPED
// The displayAlertTripped function is responsible for updating the LED matrix display to indicate that an alert has been tripped.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displayAlertTripped()
{
    Serial.println("Display Tripped Alert");
    if (isAPMode)
    {
        matrix.loadSequence(LIVE_CAUGHT);
    }
    else
    {
        matrix.loadSequence(DEBUG_CAUGHT);
    }
    matrix.play(true);
}

//--------------------------------------------------
// DISPLAY TIMEOUT
// The displayTimeout function is responsible for updating the LED matrix display to indicate that a timeout has occurred.
// It loads the appropriate sequence based on whether the device is in AP mode or not and plays the sequence.
//--------------------------------------------------
void displayTimeout()
{
    Serial.println("Display timeout");
    if (isAPMode)
    {
        matrix.loadSequence(LIVE_NO_HEARTBEAT);
    }
    else
    {
        matrix.loadSequence(DEBUG_NO_HEARTBEAT);
    }
    matrix.play(true);
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
// The currentStateToString function converts a current state code to a human-readable string.
//--------------------------------------------------
const char *currentStateToString(currentState_t state)
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
// The dumpWiFiStatus function prints the current WiFi status to the serial monitor.
//--------------------------------------------------
void dumpWiFiStatus(wl_status_t status)
{
    Serial.print("WiFi.status() = ");
    Serial.print((int)status);
    Serial.print(" (");
    Serial.print(wifiStatusToString(status));
    Serial.println(")");
}