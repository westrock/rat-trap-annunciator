#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

typedef enum TrapState
{
    NONE,
    TRIPPED,
    SET,
    TIMEOUT,
    TRIPPED_TIMEOUT,
    SET_TIMEOUT,
} CurrentState;

void processNetworkHealth();
bool getResourceParameter(const char *uri, char *key, char *value, size_t valueSize);
void processRequestURI(WiFiClient &client, const char *uri);
void processHttpRequests();
void handleServerError(WiFiClient &client, const char *message);
void sendHttpResponse(WiFiClient &client, int statusCode, const char *reasonPhrase, const char *message);
bool handleNetworkTransition(bool makeAP);
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
void displayMatrixText(ArduinoLEDMatrix &matrix, const char *text);
void displayTripped();
void displaySet();
void displayAlertTripped();
void displayTimeout();
void clearTimeoutIndication();
void dumpWifiStatus(wl_status_t status);
const char *currentTrapStateToString(CurrentState state);
const char *wifiStatusToString(wl_status_t status);
bool parseLong(const char *str, long *result);
void updateRssiStats(long rssi);

// Returns true if the given UTC epoch is inside US Central Daylight Time
bool isChicagoDST(unsigned long utcEpoch);

// Returns the correct Chicago offset in seconds (negative)
long getChicagoOffsetSeconds(unsigned long utcEpoch);

bool syncRtcFromNtp();

void processRtcMaintenance();

#endif // MAIN_H