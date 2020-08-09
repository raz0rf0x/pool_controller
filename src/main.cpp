/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library

Include following in secrets.h file:
#define WIFI_SSID 
#define WIFI_PASSWORD 

#define MQTT_PORT
#define MQTT_HOST

Leave the following commented/absent if authentication is ununsed.
#define MQTT_USER 
#define MQTT_PASSWORD 

*/

#include <secrets.h> //Credentials storage

#include <WiFi.h>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define MQTT_UPDATE_FREQ 10000 //Updater frequency in ms
const char eventtopic[] = "test/stat/event";

#define PUMP_RELAY_1 26     //Pump Relays
#define PUMP_RELAY_2 27
const char pumpsetting[] = "test/control/pump";
const char pumpsettingstatus[] = "test/stat/pump";
char pumpspeed = '0';       //Init pumpspeed global

#define NUM_TEMP_READ 10
#define TEMP_UPDATE_FREQ 1000  //Temp update freqency in ms
#define TEMP_PROBE_PIN 17      //Temp probe pin
const char temptopic[] = "test/stat/temp";
float temp = 0.0;

#define HEATER_RELAY 25 //Heater Relay
#define HEAT_UPDATE_FREQ 5000 //Heater check freqency in ms
bool heat_power = false;    //Init heatercommand global
int heatsetpoint = 0;       //Init heatsetpoint global
bool heating = false;       //Init heater status global
const char setpoint_status[] = "test/stat/setpoint";
const char heater_stat[] = "test/stat/heater";
const char heater_setpoint[] = "test/control/setpoint";
const char heater_control[] = "test/control/heater";
const char heater_run_status[] = "test/stat/heating";

OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature sensors(&oneWire);

AsyncMqttClient mqttClient;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TaskHandle_t xUpdateStatus;
TaskHandle_t xGetTempTask;
TaskHandle_t xrunHeater;

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
        if (mqttClient.publish(eventtopic, 2, false, "Unknown Heater Command payload.") == 0) {
          Serial.println("Mqtt Failed");}   
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
        Serial.println(atoi(payload));
        heatsetpoint = int(atoi(payload));
        if (mqttClient.publish(setpoint_status, 2, false, payload) == 0) {
          Serial.println("Mqtt Failed");}
      }
      else {                                            //Catchall.
        if (mqttClient.publish(eventtopic, 2, false, "Unhandled command topic: ") == 0) {
          Serial.println("Mqtt Failed");}        
      }
    } else {
      Serial.println("Invalid Payload");
    }
}

void GetTempTask(void *pvParameters) {
  int readindex = 0;
  float temptemp = 0.0;
  float temp_readings[NUM_TEMP_READ];
  float temperatureF = 0.0;

  Serial.println("Init Temp");
  if (mqttClient.publish(eventtopic, 2, false, "Init Temp") == 0) {
    Serial.println("Mqtt Failed");}
  do{
    sensors.requestTemperatures();
    Serial.println("Init retry");
    temperatureF = sensors.getTempFByIndex(0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  } while (temperatureF <= 0);
  
  for (int i = 0; i <= NUM_TEMP_READ ; i++){
    temp_readings[i] = temperatureF;
    temptemp = temptemp + temperatureF;
  }
  for (;;) {
    sensors.requestTemperatures();
    temperatureF = sensors.getTempFByIndex(0);
    if (temperatureF > 0) {
      temptemp = temptemp - temp_readings[readindex];
      temp_readings[readindex] = temperatureF;
      temptemp = temptemp + temp_readings[readindex];
      readindex = readindex + 1;

      if (readindex >= NUM_TEMP_READ) {
        readindex = 0;
      }
      temp = temptemp / (NUM_TEMP_READ + 1);
    } else {
      Serial.println("TempFail");
      if (mqttClient.publish(eventtopic, 2, false, "TempFail") == 0) {
        Serial.println("Mqtt Failed");}
    }
    vTaskDelay(TEMP_UPDATE_FREQ / portTICK_PERIOD_MS);
  }
}

void runHeater(void *pvParameters) {
  for (;;) {
    // if ( heat_power){Serial.println("heat turned on");}
    // if ( heating ){Serial.println("heat currently running");}
    // Serial.print(heatsetpoint);
    // Serial.print(" / ");
    // Serial.println(int(temp-1));
    if ( heat_power && !heating && (int(heatsetpoint) > (int(temp-1))) ){ // && (pressure > 7) ){
      digitalWrite(HEATER_RELAY, LOW);
      heating = true;
      if (mqttClient.publish(heater_stat, 2, false, "On") == 0) {
          Serial.println("Mqtt Failed");}
    }
    if ((heat_power && heating && (int(heatsetpoint) < (int(temp)))) ){ //|| (pressure < 8) ){
      digitalWrite(HEATER_RELAY, HIGH);
      heating = false;
      if (mqttClient.publish(heater_stat, 2, false, "Off") == 0) {
        Serial.println("Mqtt Failed");}
    }
    if (!heat_power){
      digitalWrite(HEATER_RELAY, HIGH);
      heating = false;
      if (mqttClient.publish(heater_stat, 2, false, "Off") == 0) {
        Serial.println("Mqtt Failed");}
    }
  vTaskDelay(HEAT_UPDATE_FREQ / portTICK_PERIOD_MS);
  }
}

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

    Serial.print("Heater run status: ");
    if (heating) {
      Serial.println("on.");
      if (mqttClient.publish(heater_run_status, 2, false, "On") == 0) {
        Serial.println("Mqtt Failed");}
    } else {
      Serial.println("off.");
      if (mqttClient.publish(heater_run_status, 2, false, "Off") == 0) {
        Serial.println("Mqtt Failed");}
    }
    
    Serial.print("Temperature: ");
    char charVal[10];
    dtostrf(temp, 4, 2, charVal);
    Serial.println(charVal);
    if (mqttClient.publish(temptopic, 2, false, charVal) == 0) {
      Serial.println("Mqtt Failed");}

    if (mqttClient.publish(eventtopic, 2, false, "Alive") == 0) {
      Serial.println("Mqtt Failed");}
    
    vTaskDelay(MQTT_UPDATE_FREQ / portTICK_PERIOD_MS);
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

    if (mqttClient.publish(eventtopic, 2, false, "Connected to MQTT") == 0) {
      Serial.println("Mqtt Failed");}

    xTaskCreate(UpdateStatus, "UpdaterTask", 2000, NULL, 1, &xUpdateStatus);
    xTaskCreate(GetTempTask, "TempTask", 20000, NULL, 1, &xGetTempTask);
    xTaskCreate(runHeater, "HeaterTask", 20000, NULL, 1, &xrunHeater);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("Disconnected from MQTT.");

    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }

    vTaskDelete(xUpdateStatus);
    vTaskDelete(xGetTempTask);
    vTaskDelete(xrunHeater);

}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    sensors.begin(); //Onewire temp sensor bus
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    //InitTempArray();

    pinMode(HEATER_RELAY, OUTPUT);          //Init GPIO for heater relay
    digitalWrite(HEATER_RELAY, HIGH);
    pinMode(PUMP_RELAY_1, OUTPUT);          //Init GPIO for pump control relays
    digitalWrite(PUMP_RELAY_1, HIGH);
    pinMode(PUMP_RELAY_2, OUTPUT);
    digitalWrite(PUMP_RELAY_2, HIGH);

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);

    #ifdef MQTT_USER //if MQTT username is set then include credentials
    mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
    #endif

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    connectToWifi();

    vTaskStartScheduler();
}

void loop() {
}