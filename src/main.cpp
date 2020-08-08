/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/

#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "***REMOVED***"
#define WIFI_PASSWORD "***REMOVED***"

#define MQTT_HOST IPAddress(192, 168, 0, 232)
#define MQTT_PORT 1883
#define MQTT_USER ***REMOVED***
#define MQTT_PASSWORD ***REMOVED***

#define PUMP_RELAY_1 26 //Pump Relays
#define PUMP_RELAY_2 27

#define HEATER_RELAY 25 //Heater Relay


const char pumpsetting[] = "test/control/pump";
const char pumpsettingstatus[] = "test/status/pump";

const char setpoint[] = "test/control/setpoint";
const char heater[] = "test/control/heater";
const char heaterstatus[] = "test/stat/heating";
const char setpointstatus[] = "test/stat/setpoint";
const char heaterstat[] = "test/stat/heater";

const char temptopic[] = "test/stat/temp";

char pumpspeed = '0';
char heat_on = '0';
int heatsetpoint = 0;


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
    switch(event) {
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
  
  uint16_t heateronPub = mqttClient.subscribe(heater, 0);
  Serial.print("Subscribed to heat control topic: ");
  Serial.println(heateronPub);

  uint16_t heatersetpointPub = mqttClient.subscribe(setpoint, 0);
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

void pumpcontrol(char speed){
  switch (speed){
    case '0':
      Serial.println("Pump off");
      digitalWrite(PUMP_RELAY_1, HIGH);
      digitalWrite(PUMP_RELAY_2, HIGH);
      pumpspeed = '0';
      if(mqttClient.publish(pumpsettingstatus, 2, false, "0") == 0){
        Serial.println("Mqtt Failed");
      }
      break;
    case '1':
      Serial.println("Pump 1");
      digitalWrite(PUMP_RELAY_1, HIGH);
      digitalWrite(PUMP_RELAY_2, LOW);
      pumpspeed = '1';
      if(mqttClient.publish(pumpsettingstatus, 2, false, "1") == 0){
        Serial.println("Mqtt Failed");
      }
      break;
    case '2':
      Serial.println("Pump 2");
      digitalWrite(PUMP_RELAY_1, LOW);
      digitalWrite(PUMP_RELAY_2, HIGH);
      pumpspeed = '2';
      if(mqttClient.publish(pumpsettingstatus, 2, false, "2") == 0){
        Serial.println("Mqtt Failed");
      }
      break;
    case '3':
      Serial.println("Pump 3");
      digitalWrite(PUMP_RELAY_1, LOW);
      digitalWrite(PUMP_RELAY_2, LOW);
      pumpspeed = '3';
      if(mqttClient.publish(pumpsettingstatus, 2, false, "3") == 0){
        Serial.println("Mqtt Failed");
      }
      break;
    default:
      Serial.print("Unknown speed: ");
      Serial.println(speed);
      break;
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");

  if (strcmp(topic,pumpsetting) == 0){
    Serial.println("Got pump command.");
    if (payload){
      pumpcontrol(payload[0]);
    } else {
      Serial.println("Invalid Payload");
    }
  } else {
    Serial.print("Unhandled command: ");
    Serial.println(topic);
  }

}

// void onMqttPublish(uint16_t packetId) {
//   Serial.println("Publish acknowledged.");
//   Serial.print("  packetId: ");
//   Serial.println(packetId);
// }

void UpdateStatus(  void *pvParameters  ) {
  for (;;){
    Serial.println("      Status");
    Serial.println("-------------------");
    Serial.print("Pump setting: ");
    Serial.println(pumpspeed);
    if(mqttClient.publish(pumpsettingstatus, 2, false, String(pumpspeed).c_str()) == 0){
      Serial.println("Mqtt Failed");
    }

    vTaskDelay( 10000 / portTICK_PERIOD_MS );
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
  //mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials("***REMOVED***","***REMOVED***");
  mqttClient.setServer("***REMOVED***", MQTT_PORT);

  xTaskCreate(UpdateStatus,"UpdaterTask",2000,NULL,1,NULL);
    
  connectToWifi();

  vTaskStartScheduler();
}

void loop() {
}