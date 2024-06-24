#include "stdint.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "math.h"
#include "WiFi.h"
#include "ArduinoMqttClient.h"
#include "wifi_details.h"

const char* wifi_ssid = SECRET_SSID;        // your network SSID (name)
const char* wifi_pass = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

const char* ap_ssid = AP_SSID;
const char* ap_pass = AP_PASS;

void setup() {
  // put your setup code here, to run once:
  WiFi.mode(WIFI_AP_STA);

  WiFi.softAP(ap_ssid, ap_pass);

  WiFi.begin(wifi_ssid, wifi_pass);

  while(WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
