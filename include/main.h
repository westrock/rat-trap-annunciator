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

void process_network_health();
void process_request_uri(WiFiClient &client, const char *uri);
void process_http_requests();
void handle_server_error(WiFiClient &client, const char *message);
void send_http_response(WiFiClient &client, int statusCode, const char *reasonPhrase, const char *message);
void handle_network_transition(bool makeAP);
void reset_heartbeat_timer();
void process_timeout();
void start_tone_for(uint8_t tonePin, unsigned int frequency1, unsigned int frequency2, unsigned long segmentDurationMs, unsigned long durationMs);
void process_alert_tone(uint8_t tonePin);
void force_quit_tone(uint8_t tonePin);
void handle_alert_tripped();
void handle_alert_set();
void handle_status_tripped();
void handle_status_set();
void handle_timeout();
void display_text(ArduinoLEDMatrix &matrix, const char *text);
void display_tripped();
void display_set();
void display_alert_tripped();
void display_timeout();
void clear_timeout_indication();
void dump_wifi_status(wl_status_t status);
const char *current_state_to_string(currentState_t state);
const char *wifi_status_to_string(wl_status_t status);
bool parse_long(const char *str, long *result);
void update_rssi_stats(long rssi);

#endif // MAIN_H