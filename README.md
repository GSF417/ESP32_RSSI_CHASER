# ESP32_RSSI_CHASER

Embedded systems program to coordinate a cart-like robot to reach a certain RSSI value in comparison to to a given "AP_SSID".

Another micro controller must be the AP_SSID

Must include a "wifi_details.h" file in the root folder, preferably the same as AP_SSID micro controller.

# Template for wifi_details.h

#define SECRET_SSID [Your Wi-fi SSID here]
#define SECRET_PASS [Your Wi-fi password here]

#define AP_SSID [Access Point SSID/Target SSID here]
#define AP_PASS [Access Point password here]