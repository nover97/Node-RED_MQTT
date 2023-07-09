#include <Arduino.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>

#define livingRoom_LED 16   // Living Room LED
#define bedRoom_1_LED 17    // Bedroom 1 LED
#define bedRoom_2_LED 18    // Bedroom 2 LED
#define kitchen_LED 19      // Kitchen LED
#define restRoom_LED 21     // Restroom LED
#define outside_LED 22      // Outside LED
#define dhtPin 23           // DHT11
#define motionSensorPin 4   // Motion Sensor
#define photoResistorPin 33 // Photo Resistor - Analog
#define restroom_VENT 32    // Restroom Ventilator

#define rainSensorPin 34       // Rain Sensor - Analog Input Only
#define rainSensorActionPin 25 // Rain Sensor Action
#define gasSensorPin 35        // Gas Sensor - Analog Input Only
#define gasSensorActionPin 26  // Gas Sensor Action
#define fireSensorPin 13       // Fire Sensor - Analog Input Only
#define fireSensorActionPin 14 // Fire Sensor Action

#define dhtTYPE DHT11

DHT dht(dhtPin, dhtTYPE);
WiFiClient client;
PubSubClient mqttClient(client);

// const String ssid = "UIT BaoVe";
// const String pass = "uitlaso1";

// const String ssid = "MA VI_2.4G";
// const String pass = "17121980";

// const String ssid = "My Home 1";
// const String pass = "1234@67890";

// const String ssid = "Thanh_Cong";
// const String pass = "48484848";

const String ssid = "UIT P. CTSV";
const String pass = "khcobiet";

// const char *serverMQTT = "192.168.26.225";          // Local VM
const char *serverMQTT = "34.125.252.87"; // GCP
const int portMQTT = 1883;
const char *idMQTT = "esp32";
const char *userMQTT = "NT114";
const char *passMQTT = "NT114iot";

const char *topicSub_LivingRoom_Light = "livingroom/light";
const char *topicSub_BedRoom1_Light = "bedroom1/light";
const char *topicSub_BedRoom2_Light = "bedroom2/light";
const char *topicSub_Kitchen_Light = "kitchen/light";
const char *topicSub_Kitchen_GasSensor = "kitchen/gasSensor";
const char *topicSub_Kitchen_FireSensor = "kitchen/fireSensor";
const char *topicSub_RestRoom_Light = "restroom/light";
const char *topicSub_RestRoom_Vent = "restroom/vent";
const char *topicSub_Outside_Light = "outside/light";
const char *topicSub_Outside_RainSensor = "outside/rainSensor";

const char *topicPub = "from-esp32";
const char *topicPub_dhtHumid = "general/dht11/humidity";
const char *topicPub_dhtTemp = "general/dht11/temperature";
const char *topicPub_gasSensor = "general/gasSensor";
const char *topicPub_fireSensor = "general/fireSensor";
const char *topicPub_rainSensor = "general/rainSensor";

unsigned long dhtLastTime = 0;
unsigned long dhtInterval = 2000; // 2 seconds
unsigned long rainSensorLastTime = 0;
unsigned long rainSensorInterval = 500; // 3 minutes
unsigned long gasSensorLastTime = 0;
unsigned long gasSensorInterval = 500; // 3 minutes
unsigned long fireSensorLastTime = 0;
unsigned long fireSensorInterval = 500; // 3 minutes
unsigned long outsideLightLastTime = 0;
unsigned long outsideLightInterval = 500; // 0.5 seconds
unsigned long photoResistorThreshold = 2000; // Light = 0 -> 4095 = Dark

bool outsideAutoLighting = false;
bool outsideAutoLightingState = false;
bool dhtValid = true;
bool gasSensorEnabled = true;
bool fireSensorEnabled = true;
bool rainSensorEnabled = true;

void callBack(char *topic, byte *payload, unsigned int length) {
    Serial.print("\tMessage arrived!\nTopic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    String message, strTopic = String(topic);
    for (size_t i = 0; i < length; i++)
        message += (char)payload[i];
    Serial.println(message);

    if (strTopic == topicSub_LivingRoom_Light) {
        if (message == "on") {
            digitalWrite(livingRoom_LED, LOW);
            Serial.println("Living Room Light - ON");

        } else if (message == "off") {
            digitalWrite(livingRoom_LED, HIGH);
            Serial.println("Living Room Light - OFF");
        }
    } else if (strTopic == topicSub_BedRoom1_Light) {
        if (message == "on") {
            digitalWrite(bedRoom_1_LED, LOW);
            Serial.println("Bedroom 1 Light - ON");

        } else if (message == "off") {
            digitalWrite(bedRoom_1_LED, HIGH);
            Serial.println("Bedroom 1 Light - OFF");
        }
    } else if (strTopic == topicSub_BedRoom2_Light) {
        if (message == "on") {
            digitalWrite(bedRoom_2_LED, LOW);
            Serial.println("Bedroom 2 Light - ON");

        } else if (message == "off") {
            digitalWrite(bedRoom_2_LED, HIGH);
            Serial.println("Bedroom 2 Light - OFF");
        }
    } else if (strTopic == topicSub_Kitchen_Light) {
        if (message == "on") {
            digitalWrite(kitchen_LED, LOW);
            Serial.println("Kitchen Light - ON");

        } else if (message == "off") {
            digitalWrite(kitchen_LED, HIGH);
            Serial.println("Kitchen Light - OFF");
        }
    } else if (strTopic == topicSub_Kitchen_GasSensor) {
        if (message == "disable") {
            gasSensorEnabled = false;
            Serial.println("Gas Sensor - OFF");

        } else if (message == "enable") {
            gasSensorEnabled = true;
            Serial.println("Gas Sensor - ON");

        } else if (message == "stop action") {
            digitalWrite(gasSensorActionPin, HIGH);
            Serial.println("Gas Sensor Vent Fan - STOPPED");
        }
    } else if (strTopic == topicSub_Kitchen_FireSensor) {
        if (message == "disable") {
            fireSensorEnabled = false;
            Serial.println("Fire Sensor - OFF");

        } else if (message == "enable") {
            fireSensorEnabled = true;
            Serial.println("Fire Sensor - ON");

        } else if (message == "stop action") {
            digitalWrite(fireSensorActionPin, HIGH);
            Serial.println("Fire Sensor Buzzer - STOPPED");
        }
    } else if (strTopic == topicSub_RestRoom_Light) {
        if (message == "on") {
            digitalWrite(restRoom_LED, LOW);
            Serial.println("Restroom Light - ON");

        } else if (message == "off") {
            digitalWrite(restRoom_LED, HIGH);
            Serial.println("Restroom Light - OFF");
        }
    } else if (strTopic == topicSub_RestRoom_Vent) {
        if (message == "on") {
            digitalWrite(restroom_VENT, LOW);
            Serial.println("Restroom Ventilator - ON");

        } else if (message == "off") {
            digitalWrite(restroom_VENT, HIGH);
            Serial.println("Restroom Ventilator - OFF");
        }
    } else if (strTopic == topicSub_Outside_Light) {
        if (message == "false") { // because the switch is reversed to ensure the Light Switch is disabled
            outsideAutoLighting = true;
            Serial.println("Outside Light - AUTO ON");

        } else if (message == "true") {
            outsideAutoLighting = false;
            Serial.println("Outside Light - AUTO OFF");

        } else if (message == "on") {
            digitalWrite(outside_LED, LOW);
            Serial.println("Outside Light - ON");

        } else if (message == "off") {
            digitalWrite(outside_LED, HIGH);
            Serial.println("Outside Light - OFF");
        }
    } else if (strTopic == topicSub_Outside_RainSensor) {
        if (message == "disable") {
            rainSensorEnabled = false;
            Serial.println("Rain Sensor - OFF");

        } else if (message == "enable") {
            rainSensorEnabled = true;
            Serial.println("Rain Sensor - ON");

        } else if (message == "stop action") {
            digitalWrite(rainSensorActionPin, HIGH);
            Serial.println("Rain Sensor Motor - STOPPED");
        }
    }

    Serial.println("------------------------------");
}

void connectWIFI() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, pass);
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to the WiFi: " + ssid);
    Serial.println("IP Address: " + WiFi.localIP().toString());
    Serial.println();
}

void connectMQTT() {
    mqttClient.setServer(serverMQTT, portMQTT);
    mqttClient.setCallback(callBack);

    Serial.print("Connecting to MQTT Broker...");
    while (!mqttClient.connect(idMQTT, userMQTT, passMQTT)) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to MQTT Broker: " + String(serverMQTT) + ":" + String(portMQTT));
    Serial.println();
    mqttClient.publish(topicPub, "Hello from ESP32");
}

void pinModes() {
    pinMode(livingRoom_LED, OUTPUT);
    pinMode(bedRoom_1_LED, OUTPUT);
    pinMode(bedRoom_2_LED, OUTPUT);
    pinMode(kitchen_LED, OUTPUT);
    pinMode(restRoom_LED, OUTPUT);
    pinMode(restroom_VENT, OUTPUT);
    pinMode(outside_LED, OUTPUT);
    pinMode(dhtPin, INPUT);
    pinMode(motionSensorPin, INPUT);
    pinMode(photoResistorPin, INPUT);

    pinMode(rainSensorPin, INPUT);
    pinMode(gasSensorPin, INPUT);
    pinMode(fireSensorPin, INPUT);
    pinMode(rainSensorActionPin, OUTPUT);
    pinMode(gasSensorActionPin, OUTPUT);
    pinMode(fireSensorActionPin, OUTPUT);
}

void initiateLightState(){
    digitalWrite(livingRoom_LED, HIGH);
    digitalWrite(bedRoom_1_LED, HIGH);
    digitalWrite(bedRoom_2_LED, HIGH);
    digitalWrite(kitchen_LED, HIGH);
    digitalWrite(restRoom_LED, HIGH);
    digitalWrite(restroom_VENT, HIGH);
    digitalWrite(outside_LED, HIGH);

    digitalWrite(rainSensorActionPin, HIGH);
    digitalWrite(gasSensorActionPin, HIGH);
    digitalWrite(fireSensorActionPin, HIGH);
}

void subscribeTopics() {
    mqttClient.subscribe(topicSub_LivingRoom_Light);
    mqttClient.subscribe(topicSub_BedRoom1_Light);
    mqttClient.subscribe(topicSub_BedRoom2_Light);
    mqttClient.subscribe(topicSub_Kitchen_Light);
    mqttClient.subscribe(topicSub_Kitchen_GasSensor);
    mqttClient.subscribe(topicSub_Kitchen_FireSensor);
    mqttClient.subscribe(topicSub_RestRoom_Light);
    mqttClient.subscribe(topicSub_RestRoom_Vent);
    mqttClient.subscribe(topicSub_Outside_Light);
    mqttClient.subscribe(topicSub_Outside_RainSensor);
}

void sendDHT11() {
    if (millis() - dhtLastTime > dhtInterval) {
        dhtLastTime = millis();
        double humid = dht.readHumidity();
        double temp = dht.readTemperature();
        dhtValid = !(isnan(humid) || isnan(temp) || humid < 0 || humid > 100);

        if (dhtValid) {
            mqttClient.publish(topicPub_dhtTemp, String(temp).c_str());
            mqttClient.publish(topicPub_dhtHumid, String(humid).c_str());
        }
    }
}

void rainSensorAction() {
    if (rainSensorEnabled) {
        if (millis() - rainSensorLastTime > rainSensorInterval) {
            rainSensorLastTime = millis();
            int value = digitalRead(rainSensorPin); // 0: water detected, 1: no water detected
            digitalWrite(rainSensorActionPin, value);
            mqttClient.publish(topicPub_rainSensor, String(!value).c_str());
        }
    }
}

void gasSensorAction() {
    if (gasSensorEnabled) {
        if (millis() - gasSensorLastTime > gasSensorInterval) {
            gasSensorLastTime = millis();
            int value = digitalRead(gasSensorPin);  // 0: gas detected, 1: no gas
            // Serial.println("Gas Sensor: " + String(value));
            digitalWrite(gasSensorActionPin, value);    // 0: turn on, 1: turn off 
            mqttClient.publish(topicPub_gasSensor, String(!value).c_str()); // 0: no gas, 1: gas detected
        }
    }
}

void fireSensorAction() {
    if (fireSensorEnabled) {
        if (millis() - fireSensorLastTime > fireSensorInterval) {
            fireSensorLastTime = millis();
            int value = digitalRead(fireSensorPin);
            digitalWrite(fireSensorActionPin, !value);
            mqttClient.publish(topicPub_fireSensor, String(!value).c_str());
        }
    }
}

void outsideAutoLightingFunc() {
    if (outsideAutoLighting) {
        if (millis() - outsideLightLastTime > outsideLightInterval) {
            outsideLightLastTime = millis();
            int photoresistorValue = analogRead(photoResistorPin);
            // Serial.println("PhotoResistor: " + String(photoresistorValue));
            int motionSensorValue = digitalRead(motionSensorPin);
            Serial.println("MotionSensor: " + String(motionSensorValue));
            // Serial.println("Motion Sensor: " + String(motionSensorValue));
            outsideAutoLightingState = (photoresistorValue > photoResistorThreshold) || motionSensorValue;
            digitalWrite(outside_LED, !outsideAutoLightingState);
        }
    }
}


void sendToServer() {
    sendDHT11();
    rainSensorAction();
    gasSensorAction();
    fireSensorAction();
    outsideAutoLightingFunc();
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    connectWIFI();
    pinModes();
    initiateLightState();
    delay(1000);
    connectMQTT();
    delay(1000);
    subscribeTopics();
}

void loop() {
    // put your main code here, to run repeatedly:
    mqttClient.loop();
    sendToServer();
}
