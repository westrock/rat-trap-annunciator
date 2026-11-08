#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

typedef enum
{
    NONE,
    TRIPPED,
    SET,
    TIMEOUT,
    TRIPPED_TIMEOUT,
    SET_TIMEOUT,
} currentState_t;

void processNetworkHealth();
void processRequest(WiFiClient &client, const char *uri);
void processHttpRequests();
void handleServerError(WiFiClient &client, const char *message);
void sendHttpResponse(WiFiClient &client, int statusCode, const char *reasonPhrase, const char *message);
void handleNetworkTransition(bool makeAP);
void resetHeartbeatTimer();
void processTimeout();
void startToneFor(uint8_t tonePin, unsigned int frequency1, unsigned int frequency2, unsigned long segmentDurationMs, unsigned long durationMs);
void processAlertTone(uint8_t tonePin);
void forceQuitTone(uint8_t tonePin);
void handleAlertTripped();
void handleAlertSet();
void handleStatusTripped();
void handleStatusSet();
void handleTimeout();
void displayText(ArduinoLEDMatrix &matrix, const char *text);
void displayTripped();
void displaySet();
void displayAlertTripped();
void displayTimeout();
void clearTimeoutIndication();
void dumpWiFiStatus(wl_status_t status);
const char *currentStateToString(currentState_t state);
const char *wifiStatusToString(wl_status_t status);
bool parse_long(const char *str, long *result);
void updateRssiStats(long rssi);

#endif // MAIN_H