#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32_Servo.h>
#include "Wire.h"

//pins
int Led = 17;
int servoPin = 18;
Servo myservo;
int wait = 0;


// MOU-6050 values
const uint8_t MPU_addr = 0x68; // I2C address of the MPU-6050
const float MPU_GYRO_250_SCALE = 131.0;
const float MPU_GYRO_500_SCALE = 65.5;
const float MPU_GYRO_1000_SCALE = 32.8;
const float MPU_GYRO_2000_SCALE = 16.4;
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
const float MPU_ACCL_16_SCALE = 2048.0;


//values to determine movement
bool previous = false;
float changeOld;
float changeNew;


//netwerok detail
const char* ssid = "Mi Phone";
const char* password = "Ricardo1";


//security cert
const char* ca_cert = \
                      "-----BEGIN CERTIFICATE-----\n" \
                      "MIIEKjCCAxKgAwIBAgIJAIQuL0aM6TEqMA0GCSqGSIb3DQEBCwUAMIGpMQswCQYD\n" \
                      "VQQGEwJOTDEWMBQGA1UECAwNTm9vcmQtQnJhYmFudDETMBEGA1UEBwwKRGludGVs\n" \
                      "b29yZDERMA8GA1UECgwIUmVkYWN0ZWQxFTATBgNVBAsMDEJyb2tlckNvbmZpZzEY\n" \
                      "MBYGA1UEAwwPQnl0ZWdyb2VwMTMtNy84MSkwJwYJKoZIhvcNAQkBFhpyaWNhcmRv\n" \
                      "LnN0ZWlqbjk3QGdtYWlsLmNvbTAeFw0xOTAzMTMxNzUwNDBaFw0yNDAzMTIxNzUw\n" \
                      "NDBaMIGpMQswCQYDVQQGEwJOTDEWMBQGA1UECAwNTm9vcmQtQnJhYmFudDETMBEG\n" \
                      "A1UEBwwKRGludGVsb29yZDERMA8GA1UECgwIUmVkYWN0ZWQxFTATBgNVBAsMDEJy\n" \
                      "b2tlckNvbmZpZzEYMBYGA1UEAwwPQnl0ZWdyb2VwMTMtNy84MSkwJwYJKoZIhvcN\n" \
                      "AQkBFhpyaWNhcmRvLnN0ZWlqbjk3QGdtYWlsLmNvbTCCASIwDQYJKoZIhvcNAQEB\n" \
                      "BQADggEPADCCAQoCggEBAKiRR91+w5lvtte4QUi3vAkOxNktHMshTB6gP0l1pfsx\n" \
                      "C2dyCnz4Ro6LMKR5HBynws7XdF1Jz++/ebOw7pOMLEYD+rsnO1xYU9tG1cRqVi52\n" \
                      "Msg6UiA5dlwJShVeOE9OdouR4FAiePYM4Tr4qQeVAqZABuV/aodyxIzZMsebxBlm\n" \
                      "5qcXM9pDoQkPqBcFxdzJeaV4zJqRQMqbkrSNZ/vpQGiGxi1pw3/WFq/j217jcLVg\n" \
                      "i+BwvRAWRDB+rQStQwponbfw3/L8JlfSXYbOLS0+ylMdCs0ZV8ii41KAUTl7YXVD\n" \
                      "zzy0uCqrsKbBKGJlSad5kPgwruv6UTlO3O5deC3Rl60CAwEAAaNTMFEwHQYDVR0O\n" \
                      "BBYEFIRzsPOKap/bdRoC++5vMVeDi+SyMB8GA1UdIwQYMBaAFIRzsPOKap/bdRoC\n" \
                      "++5vMVeDi+SyMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQELBQADggEBAD4m\n" \
                      "yEGc8CkpDPeIi+yRNOiRGz9IHs5jqhyWGGzK32wAohrK+3h0TUGbwrJghh+oylr0\n" \
                      "Nz062/EyXYvB3DDnnC3HjyfTjp5HFTxP6Kg7GeGA7J5QUeLT/90fCfhujFyFiHsD\n" \
                      "D7dkSiehccuEjzdjsRA53QWSZ27lVs7xjt8cWqk+9hckE6QGWeF1WmFw8IG+jf2W\n" \
                      "1atqcCHnh5wGJtKOkYuXV5zfn4NnDw0LokB6851EJkm92Dr7m2TPxBrKF+jSrRpJ\n" \
                      "p+YZdJB6JzNVWVDjOKrI1UfY8CinlSOjuUVhTRDSzdRlm5aGzP/ap/4LnHbhuYHd\n" \
                      "Z4yS/7+DVI+pzfFhy5A=\n" \
                      "-----END CERTIFICATE-----\n";


// create an instance of WiFiClientSecure 
WiFiClientSecure espClient;
PubSubClient client(espClient);

// topics 
#define TOPIC "ESP32/locks/front door"

long lastMsg = 0;
char msg[20];
int counter = 0;
bool doorOpen = true;



//MQTT Receieve + servo control
void receivedCallback(char* topic, byte* payload, unsigned int length) {
  String receivedMessage = "";
  Serial.print("Message received: ");
  Serial.println(topic);

  Serial.print("payload: ");
  //stuur motor aan
  for (int i = 0; i < length; i++) {
    receivedMessage += (char)payload[i];
  }

  Serial.println(receivedMessage);
  Serial.println();

    if (receivedMessage == "OPEN") {
      myservo.write(180);//DIT IS SLOT IN
      doorOpen = true;
  }

    if (receivedMessage == "CLOSE") {
      myservo.write(0); // DIT IS SLOT UIT
      doorOpen = false;
    }
}


//MQTT setup
void mqttconnect() {
  /* Loop until reconnected */
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic */
      client.subscribe(TOPIC);
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

struct rawdata {
  int16_t AcX;
  int16_t AcY;
  int16_t AcZ;
  int16_t Tmp;
  int16_t GyX;
  int16_t GyY;
  int16_t GyZ;
};

struct scaleddata {
  float AcX;
  float AcY;
  float AcZ;
  float Tmp;
  float GyX;
  float GyY;
  float GyZ;
};

bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr, bool Debug);
void setMPU6050scales(byte addr, uint8_t Gyro, uint8_t Accl);
void getMPU6050scales(byte addr, uint8_t &Gyro, uint8_t &Accl);
scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug);

void setup() {
  //servo setup
  myservo.attach(servoPin);
  myservo.write(180);//open lock

  Wire.begin();
  pinMode(Led, OUTPUT);

  mpu6050Begin(MPU_addr);

  Serial.begin(115200);
  // wifi connection
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // set SSL/TLS certificate 
  espClient.setCACert(ca_cert);
  // configure MQTT server
  client.setServer("192.168.43.143", 8884);

  client.setCallback(receivedCallback);

}
void loop() {

  //Gyro 
  rawdata next_sample;
  setMPU6050scales(MPU_addr, 0b00000000, 0b00010000);
  next_sample = mpu6050Read(MPU_addr, true);
  convertRawToScaled(MPU_addr, next_sample, true);

  //MQTT
  // if client was disconnected then try to reconnect again 
  if (!client.connected()) {
    mqttconnect();
  }

  client.loop();

}


// GYRO init
void mpu6050Begin(byte addr) {

  if (checkI2c(addr)) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    delay(30); // Ensure gyro has enough time to power up
  }
}

//IC2 check
bool checkI2c(byte addr) {

  Serial.println(" ");
  Wire.beginTransmission(addr);

  if (Wire.endTransmission() == 0)
  {
    Serial.print(" Device Found at 0x");
    Serial.println(addr, HEX);
    return true;
  }
  else
  {
    Serial.print(" No Device Found at 0x");
    Serial.println(addr, HEX);
    return false;
  }
}


// ump read
rawdata mpu6050Read(byte addr, bool Debug) {

  rawdata values;

  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 14, true); // request a total of 14 registers
  values.AcX = Wire.read() << 8 | Wire.read(); 
  values.AcY = Wire.read() << 8 | Wire.read(); 
  values.AcZ = Wire.read() << 8 | Wire.read(); 
  return values;
}

void setMPU6050scales(byte addr, uint8_t Gyro, uint8_t Accl) {
  Wire.beginTransmission(addr);
  Wire.write(0x1B); 
  Wire.write(Gyro); 
  Wire.write(Accl); 
  Wire.endTransmission(true);
}

void getMPU6050scales(byte addr, uint8_t &Gyro, uint8_t &Accl) {
  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, true); // request a total of 14 registers
  Gyro = (Wire.read() & (bit(3) | bit(4))) >> 3;
  Accl = (Wire.read() & (bit(3) | bit(4))) >> 3;
}

scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug) {

  scaleddata values;
  float scale_value = 0.0;
  byte Gyro, Accl;

  getMPU6050scales(MPU_addr, Gyro, Accl);

  values.GyX = (float) data_in.GyX / scale_value;
  values.GyY = (float) data_in.GyY / scale_value;
  values.GyZ = (float) data_in.GyZ / scale_value;

  scale_value = 0.0;
  if (Debug) {

  }
  switch (Accl) {
    case 0:
      scale_value = MPU_ACCL_2_SCALE;
      break;
    case 1:
      scale_value = MPU_ACCL_4_SCALE;
      break;
    case 2:
      scale_value = MPU_ACCL_8_SCALE;
      break;
    case 3:
      scale_value = MPU_ACCL_16_SCALE;
      break;
    default:
      break;
  }
  values.AcX = (float) data_in.AcX / scale_value;
  values.AcY = (float) data_in.AcY / scale_value;
  values.AcZ = (float) data_in.AcZ / scale_value;


  if (previous == true) {
    previous = false;
    changeOld = values.AcY;
  }
  else if (previous == false) {
    previous = true;
    changeNew = values.AcY;
  }

  if (wait < 1){
    
  
  if  (changeNew + -changeOld > 0.05) {
    Serial.println("HELP");
    client.publish("ESP32/warning", "ALERT");
    digitalWrite(Led, HIGH);
    wait = 1000;
  }
  if  (changeOld + -changeNew > 0.05) {
    Serial.println("HELP") ;
    client.publish("ESP32/warning", "ALERT");
    digitalWrite(Led, HIGH);
    wait = 1000;
  }
  }
  if ( wait > 0) wait--;
  if (wait == 1) digitalWrite (Led, LOW);

  return values;
}
