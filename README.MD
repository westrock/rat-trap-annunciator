# rat-trap-annunciator
This Arduino Sketch is targeted to run on an Arduino UNO R4 Wifi and serves as a listener/annunciator for remote rat-trap monitors.

The rat-trap-annunciation uses WiFi to receive status and alert updates from a rat-trap-monitor.  It can be configured to run as a station, connecting to a larger wireless network, or as an access point, spinning up its own network.

## RTC calibration

The UNO R4 WiFi firmware uses an RA4M1 LOCO frequency comparison value of `260` (the Arduino core default is `255`). This initial calibration was calculated from a measured gain of about 69.9 seconds per hour. In the first hardware check, after a little over 61 minutes, the reported RTC drift was 0 seconds. This is encouraging but is only one interval; collect additional sync measurements before treating the value as validated across longer runs or different boards.
