/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/

#include <WiFi.h>
#include <secrets.h>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define PUMP_RELAY_1 26 //Pump Relays
#define PUMP_RELAY_2 27

#define HEATER_RELAY 25 //Heater Relay

#define MQTT_UPDATE_FREQ 10000 //Updater frequency in ms

const char pumpsetting[] = "test/control/pump";
const char pumpsettingstatus[] = "test/stat/pump";

const char heater_setpoint[] = "test/control/setpoint";
const char heater_control[] = "test/control/heater";
const char heater_run_status[] = "test/stat/heating";
const char setpoint_status[] = "test/stat/setpoint";
const char heater_stat[] = "test/stat/heater";

const char temptopic[] = "test/stat/temp";

char pumpspeed = '0';
bool heat_power = false;
int heatsetpoint = 0;
bool heating = false;


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
    Serial.println("Connected to MQTT.");
    Serial.print("Session present: ");
    Serial.println(sessionPresent);

    uint16_t pumpsettingPub = mqttClient.subscribe(pumpsetting, 0);
    Serial.print("Subscribed to pump topic: ");
    Serial.println(pumpsettingPub);

    uint16_t heateronPub = mqttClient.subscribe(heater_control, 0);
    Serial.print("Subscribed to heat control topic: ");
    Serial.println(heateronPub);

    uint16_t heatersetpointPub = mqttClient.subscribe(heater_setpoint, 0);
    Serial.print("Subscribed to heat setpoint topic: ");
    Serial.println(heatersetpointPub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("Disconnected from MQTT.");

    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

// void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
//   Serial.println("Subscribe acknowledged.");
//   Serial.print("  packetId: ");
//   Serial.println(packetId);
//   Serial.print("  qos: ");
//   Serial.println(qos);
// }

// void onMqttUnsubscribe(uint16_t packetId) {
//   Serial.println("Unsubscribe acknowledged.");
//   Serial.print("  packetId: ");
//   Serial.println(packetId);
//}

void pumpcontrol(char speed) {  //Pump speed setting.
    switch (speed) {
    case '0':
        Serial.println("Pump off");
        digitalWrite(PUMP_RELAY_1, HIGH);
        digitalWrite(PUMP_RELAY_2, HIGH);
        pumpspeed = '0';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "0") == 0) {
            Serial.println("Mqtt Failed");
        }
        break;
    case '1':
        Serial.println("Pump 1");
        digitalWrite(PUMP_RELAY_1, HIGH);
        digitalWrite(PUMP_RELAY_2, LOW);
        pumpspeed = '1';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "1") == 0) {
            Serial.println("Mqtt Failed");
        }
        break;
    case '2':
        Serial.println("Pump 2");
        digitalWrite(PUMP_RELAY_1, LOW);
        digitalWrite(PUMP_RELAY_2, HIGH);
        pumpspeed = '2';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "2") == 0) {
            Serial.println("Mqtt Failed");
        }
        break;
    case '3':
        Serial.println("Pump 3");
        digitalWrite(PUMP_RELAY_1, LOW);
        digitalWrite(PUMP_RELAY_2, LOW);
        pumpspeed = '3';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "3") == 0) {
            Serial.println("Mqtt Failed");
        }
        break;
    default:
        Serial.print("Unknown speed: ");
        Serial.println(speed);
        break;
    }
}

void onHeaterControl(String power) {  //Heater power control.
    if (power == "off"){
      heat_power = false;
      Serial.println("Heater turned off.");
      if (mqttClient.publish(heater_stat, 2, false, "off") == 0) {
        Serial.println("Mqtt Failed");
      }
    } else if(power == "on") {
      heat_power = true;
      Serial.println("Heater turned on.");
      if (mqttClient.publish(heater_stat, 2, false, "on") == 0) {
        Serial.println("Mqtt Failed");
      }
    } else {
      Serial.println("Unknown heater command payload.");
    }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    Serial.print("Publish received. Len: ");
    Serial.println(len);
    if (payload) { // Check for invalid payload. 
      Serial.print(topic);
      Serial.print(" : ");
      Serial.println(String(payload).substring(0,len));
      // Check topic
      if (strcmp(topic, pumpsetting) == 0) {            //Pump speed setting topic.
        Serial.println("Got pump command.");
        pumpcontrol(payload[0]);
      }
      else if(strcmp(topic, heater_control) == 0) {     //Heater control topic.
        Serial.print("Got Heater command: ");
        Serial.println(String(payload).substring(0,len));
        onHeaterControl(String(payload).substring(0,len));
      }
        else if(strcmp(topic, heater_setpoint) == 0) {  //Heater setpoint topic.
        Serial.print("Got Heater setpoint command: ");
        Serial.println(payload);
      }
      else {                                            //Catchall.
        Serial.print("Unhandled command: ");
        Serial.println(topic);
      }
    } else {
      Serial.println("Invalid Payload");
    }
}

// void onMqttPublish(uint16_t packetId) {
//   Serial.println("Publish acknowledged.");
//   Serial.print("  packetId: ");
//   Serial.println(packetId);
// }

void UpdateStatus(void *pvParameters) {
    for (;;) {
        Serial.println("      Status");
        Serial.println("-------------------");
        Serial.print("Pump setting: ");
        Serial.println(pumpspeed);
        if (mqttClient.publish(pumpsettingstatus, 2, false, String(pumpspeed).c_str()) == 0) {
            Serial.println("Mqtt Failed");
        }
        Serial.print("Heater setting: ");
        if (heat_power) {
          Serial.println("on.");
          if (mqttClient.publish(heater_stat, 2, false, "on") == 0) {
            Serial.println("Mqtt Failed");}
        } else {
          Serial.println("off.");
          if (mqttClient.publish(heater_stat, 2, false, "off") == 0) {
            Serial.println("Mqtt Failed");}
        }
        vTaskDelay(MQTT_UPDATE_FREQ / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    pinMode(HEATER_RELAY, OUTPUT);
    digitalWrite(HEATER_RELAY, HIGH);
    pinMode(PUMP_RELAY_1, OUTPUT);
    digitalWrite(PUMP_RELAY_1, HIGH);
    pinMode(PUMP_RELAY_2, OUTPUT);
    digitalWrite(PUMP_RELAY_2, HIGH);

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    //mqttClient.onSubscribe(onMqttSubscribe);
    //mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    //mqttClient.onPublish(onMqttPublish);
    mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    xTaskCreate(UpdateStatus, "UpdaterTask", 2000, NULL, 1, NULL);

    connectToWifi();

    vTaskStartScheduler();
}

void loop() {
}