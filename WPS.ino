#include "WiFi.h"
#include "esp_wps.h"
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ESP32_Servo.h>
int button = 12;
int servoPin = 18;
Servo myservo;

#define AIO_SERVER      "test.mosquitto.org"
#define AIO_SERVERPORT  1883

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
Adafruit_MQTT_Subscribe lockStatus = Adafruit_MQTT_Subscribe(&mqtt, "ESP32/lock/lockstate");



#define ESP_WPS_MODE      WPS_TYPE_PBC
#define ESP_MANUFACTURER  "ESPRESSIF"
#define ESP_MODEL_NUMBER  "ESP32"
#define ESP_MODEL_NAME    "ESPRESSIF IOT"
#define ESP_DEVICE_NAME   "ESP STATION"

static esp_wps_config_t config;

void wpsInitConfig() {
  config.crypto_funcs = &g_wifi_default_wps_crypto_funcs;
  config.wps_type = ESP_WPS_MODE;
  strcpy(config.factory_info.manufacturer, ESP_MANUFACTURER);
  strcpy(config.factory_info.model_number, ESP_MODEL_NUMBER);
  strcpy(config.factory_info.model_name, ESP_MODEL_NAME);
  strcpy(config.factory_info.device_name, ESP_DEVICE_NAME);
}

String wpspin2string(uint8_t a[]) {
  char wps_pin[9];
  for (int i = 0; i < 8; i++) {
    wps_pin[i] = a[i];
  }
  wps_pin[8] = '\0';
  return (String)wps_pin;
}

void WiFiEvent(WiFiEvent_t event, system_event_info_t info) {
  switch (event) {
    case SYSTEM_EVENT_STA_START:
      Serial.println("Station Mode Started");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to :" + String(WiFi.SSID()));
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from station, attempting reconnection");
      WiFi.reconnect();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WPS Successfull, stopping WPS and connecting to: " + String(WiFi.SSID()));
      esp_wifi_wps_disable();
      delay(10);
      WiFi.begin();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WPS Failed, retrying");
      esp_wifi_wps_disable();
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WPS Timedout, retrying");
      esp_wifi_wps_disable();
      esp_wifi_wps_enable(&config);
      esp_wifi_wps_start(0);
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WPS_PIN = " + wpspin2string(info.sta_er_pin.pin_code));
      break;
    default:
      break;
  }
}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void closeDoor() {



}

bool doorOpen = true;
void controlDoor() {

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &lockStatus) {
      String data = (char *)lockStatus.lastread;
      Serial.print(F("Got: "));
      Serial.println(data);

      if (doorOpen = false) {
        if (data == "OPEN DOOR") {
          myservo.write(200);//DIT IS SLOT IN
          doorOpen = true;
        }
      }

      else if (doorOpen = true) {
        if (data == "CLOSE DOOR") {
          myservo.write(0); // DIT IS SLOT UIT
          doorOpen = false;
        }
      }
    }
  }
}

void setup() {
  pinMode(button, INPUT_PULLUP)
  myservo.attach(servoPin);
  myservo.write(200);//DIT IS SLOT IN
  Serial.begin(115200);
  delay(10);
  Serial.println();
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println("Starting WPS");
  wpsInitConfig();
  esp_wifi_wps_enable(&config);
  esp_wifi_wps_start(0);
  mqtt.subscribe(&lockStatus);
}

void loop() {
  //connect to MQTT broker
  MQTT_connect();


  //check if the server is still running by pinging it
  if (! mqtt.ping()) {
    mqtt.disconnect();
  }
  // Listen to broker for commands
  controlDoor();
}
