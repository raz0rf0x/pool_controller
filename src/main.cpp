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
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <TelnetSpy.h>

TelnetSpy SerialAndTelnet;
#define SERIAL  SerialAndTelnet

#define MQTT_UPDATE_FREQ 10000 //Updater frequency in ms
const char eventtopic[] = "test/stat/event";

#define PUMP_RELAY_1 26     //Pump Relays
#define PUMP_RELAY_2 27
const char pumpsetting[] = "test/control/pump";
const char pumpsettingstatus[] = "test/stat/pump";
char pumpspeed = '0';       //Init pumpspeed global

#define NUM_TEMP_READ 10
#define TEMP_UPDATE_FREQ 5000  //Temp update freqency in ms
#define TEMP_PROBE_PIN 32      //Temp probe pin
const char temptopic[] = "test/stat/temp";
float temp = 0.0;

#define HEATER_RELAY 25 //Heater Relay
#define HEAT_UPDATE_FREQ 10000 //Heater check freqency in ms
bool heat_power = false;    //Init heatercommand global
int heatsetpoint = 0;       //Init heatsetpoint global
bool heating = false;       //Init heater status global
const char setpoint_status[] = "test/stat/setpoint";
const char heater_stat[] = "test/stat/heater";
const char heater_setpoint[] = "test/control/setpoint";
const char heater_control[] = "test/control/heater";
const char heater_run_status[] = "test/stat/heating";

#define PRESSURE_PIN 35 //Pressure sensor pin
#define PRESSURE_UPDATE_FREQ 1000 //Heater check freqency in ms
const char pressuretopic[] = "test/stat/pressure";
int pressure = 0;

OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature sensors(&oneWire);

AsyncMqttClient mqttClient;
AsyncWebServer server(80); //ElegantOTA

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

TaskHandle_t xUpdateStatus;
TaskHandle_t xGetTempTask;
TaskHandle_t xrunHeater;
TaskHandle_t xpressureTask;

void connectToWifi() {
    SERIAL.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
    SERIAL.println("Connecting to MQTT...");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    SERIAL.printf("[WiFi-event] event: %d\n", event);
    switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        SERIAL.println("WiFi connected");
        SERIAL.println("IP address: ");
        SERIAL.println(WiFi.localIP());
        connectToMqtt();
        server.begin(); //ElegantOTA
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        SERIAL.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void pumpcontrol(char speed) {  //Pump speed setting.
    switch (speed) {
    case '0':
        SERIAL.println("Pump off");
        digitalWrite(PUMP_RELAY_1, HIGH);
        digitalWrite(PUMP_RELAY_2, HIGH);
        pumpspeed = '0';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "0") == 0) {
            SERIAL.println("Mqtt Failed");
        }
        break;
    case '1':
        SERIAL.println("Pump 1");
        digitalWrite(PUMP_RELAY_1, HIGH);
        digitalWrite(PUMP_RELAY_2, LOW);
        pumpspeed = '1';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "1") == 0) {
            SERIAL.println("Mqtt Failed");
        }
        break;
    case '2':
        SERIAL.println("Pump 2");
        digitalWrite(PUMP_RELAY_1, LOW);
        digitalWrite(PUMP_RELAY_2, HIGH);
        pumpspeed = '2';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "2") == 0) {
            SERIAL.println("Mqtt Failed");
        }
        break;
    case '3':
        SERIAL.println("Pump 3");
        digitalWrite(PUMP_RELAY_1, LOW);
        digitalWrite(PUMP_RELAY_2, LOW);
        pumpspeed = '3';
        if (mqttClient.publish(pumpsettingstatus, 2, false, "3") == 0) {
            SERIAL.println("Mqtt Failed");
        }
        break;
    default:
        SERIAL.print("Unknown speed: ");
        SERIAL.println(speed);
        break;
    }
}

void onHeaterControl(String power) {  //Heater power control.
    if (power == "off"){
      heat_power = false;
      SERIAL.println("Heater turned off.");
      if (mqttClient.publish(heater_stat, 2, false, "off") == 0) {
        SERIAL.println("Mqtt Failed");
      }
    } else if(power == "on") {
      heat_power = true;
      SERIAL.println("Heater turned on.");
      if (mqttClient.publish(heater_stat, 2, false, "on") == 0) {
        SERIAL.println("Mqtt Failed");
      }
    } else {
      SERIAL.println("Unknown heater command payload.");
        if (mqttClient.publish(eventtopic, 2, false, "Unknown Heater Command payload.") == 0) {
          SERIAL.println("Mqtt Failed");}   
    }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    std::string payloadtrimmed1{payload, len};
    const char* payloadtrimmed = payloadtrimmed1.c_str();
    SERIAL.print("Publish received. Len: ");
    SERIAL.println(len);
    if (payload) { // Check for invalid payload. 
      SERIAL.print(topic);
      SERIAL.print(" : ");
      SERIAL.println(String(payload).substring(0,len));
      SERIAL.println(payloadtrimmed);
      // Check topic
      if (strcmp(topic, pumpsetting) == 0) {            //Pump speed setting topic.
        SERIAL.println("Got pump command.");
        pumpcontrol(payload[0]);
      }
      else if(strcmp(topic, heater_control) == 0) {     //Heater control topic.
        SERIAL.print("Got Heater command: ");
        SERIAL.println(String(payload).substring(0,len));
        onHeaterControl(String(payload).substring(0,len));
      }
        else if(strcmp(topic, heater_setpoint) == 0) {  //Heater setpoint topic.
        SERIAL.print("Got Heater setpoint command: ");
        SERIAL.println(atoi(payload));
        // SERIAL.println(payloadtrimmed);
        heatsetpoint = int(atoi(payload));
        if (mqttClient.publish(setpoint_status, 2, false, payload) == 0) {
          SERIAL.println("Mqtt Failed");}
      }
      else {                                            //Catchall.
        if (mqttClient.publish(eventtopic, 2, false, "Unhandled command topic: ") == 0) {
          SERIAL.println("Mqtt Failed");}        
      }
    } else {
      SERIAL.println("Invalid Payload");
    }
}

void PressureTask(void *pvParameters) {
  int readindex = 0;
  int pressure_sum = 0;
  int readings[10];
  int pval = analogRead(PRESSURE_PIN);
  for (int i = 0; i <= 9; i++) {
    readings[i] = pval;
    pressure_sum += pval;
  }
  for (;;) {
    pval = analogRead(PRESSURE_PIN);
    pressure_sum = pressure_sum - readings[readindex];
    readings[readindex] = pval;
    pressure_sum = pressure_sum + readings[readindex];
    readindex++;
    if (readindex >= 10) {
      readindex = 0;
    }
    pressure = map((pressure_sum / 10), 31, 479, 0, 30);
    if (pressure < 0) {
      pressure = 0;
    }
    vTaskDelay(PRESSURE_UPDATE_FREQ / portTICK_PERIOD_MS);
  }
}

void GetTempTask(void *pvParameters) {
  int readindex = 0;
  float temptemp = 0.0;
  float temp_readings[NUM_TEMP_READ];
  float temperatureF = 0.0;

  SERIAL.println("Init Temp");
  if (mqttClient.publish(eventtopic, 2, false, "Init Temp") == 0) {
    SERIAL.println("Mqtt Failed");}
  do{
    noInterrupts();
    sensors.requestTemperatures();
    SERIAL.println("Temp Init retry");
    temperatureF = sensors.getTempFByIndex(0);
    interrupts();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  } while (temperatureF <= 0);
  
  for (int i = 0; i <= NUM_TEMP_READ ; i++){
    temp_readings[i] = temperatureF;
    temptemp = temptemp + temperatureF;
  }
  for (;;) {
    sensors.requestTemperaturesByIndex(0);
    temperatureF = sensors.getTempFByIndex(0);
    // SERIAL.println("Blyat");
    if (temperatureF > 0) {
      temptemp = temptemp - temp_readings[readindex];
      temp_readings[readindex] = temperatureF;
      temptemp = temptemp + temp_readings[readindex];
      readindex = readindex + 1;
      // SERIAL.print("Temp: ");
      // SERIAL.println(temperatureF);

      if (readindex >= NUM_TEMP_READ) {
        readindex = 0;
      }
      temp = temptemp / (NUM_TEMP_READ + 1);
    } else {
      SERIAL.print("TempFail");
      SERIAL.println(temperatureF);
      sensors.requestTemperaturesByIndex(0);
      if (mqttClient.publish(eventtopic, 2, false, "TempFail") == 0) {
        SERIAL.println("Mqtt Failed");}
    }
    vTaskDelay(TEMP_UPDATE_FREQ / portTICK_PERIOD_MS);
  }
}

void runHeater(void *pvParameters) {
  for (;;) {
    // if ( heat_power){SERIAL.println("heat turned on");}
    // if ( heating ){SERIAL.println("heat currently running");}
    // SERIAL.print(heatsetpoint);
    // SERIAL.print(" / ");
    // SERIAL.println(int(temp-1));
    if ( heat_power && !heating && (int(heatsetpoint) > (int(temp-1))) ){ // && (pressure > 7) ){
      digitalWrite(HEATER_RELAY, LOW);
      heating = true;
      if (mqttClient.publish(heater_stat, 2, false, "On") == 0) {
          SERIAL.println("Mqtt Failed");}
    }
    if ((heat_power && heating && (int(heatsetpoint) < (int(temp)))) ){ //|| (pressure < 8) ){
      digitalWrite(HEATER_RELAY, HIGH);
      heating = false;
      if (mqttClient.publish(heater_stat, 2, false, "Off") == 0) {
        SERIAL.println("Mqtt Failed");}
    }
    if (!heat_power){
      digitalWrite(HEATER_RELAY, HIGH);
      heating = false;
      if (mqttClient.publish(heater_stat, 2, false, "Off") == 0) {
        SERIAL.println("Mqtt Failed");}
    }
  vTaskDelay(HEAT_UPDATE_FREQ / portTICK_PERIOD_MS);
  }
}

void UpdateStatus(void *pvParameters) {
  for (;;) {
    SERIAL.println("      Status");
    SERIAL.println("-------------------");
    SERIAL.print("Pump setting: ");
    SERIAL.println(pumpspeed);
    if (mqttClient.publish(pumpsettingstatus, 2, false, String(pumpspeed).c_str()) == 0) {
      SERIAL.println("Mqtt Failed");
    }

    SERIAL.print("Heater setting: ");
    if (heat_power) {
      SERIAL.println("on.");
      if (mqttClient.publish(heater_stat, 2, false, "on") == 0) {
        SERIAL.println("Mqtt Failed");}
    } else {
      SERIAL.println("off.");
      if (mqttClient.publish(heater_stat, 2, false, "off") == 0) {
        SERIAL.println("Mqtt Failed");}
    }

    SERIAL.print("Heater setpoint: ");
    SERIAL.println(heatsetpoint);
    char setVal[10];
    dtostrf(heatsetpoint, 4, 0, setVal);
      if (mqttClient.publish(setpoint_status, 2, false, setVal) == 0) {
          SERIAL.println("Mqtt Failed");}

    SERIAL.print("Heater run status: ");
    if (heating) {
      SERIAL.println("on.");
      if (mqttClient.publish(heater_run_status, 2, false, "On") == 0) {
        SERIAL.println("Mqtt Failed");}
    } else {
      SERIAL.println("off.");
      if (mqttClient.publish(heater_run_status, 2, false, "Off") == 0) {
        SERIAL.println("Mqtt Failed");}
    }
    
    SERIAL.print("Temperature: ");
    char charVal[10];
    dtostrf(temp, 4, 2, charVal);
    SERIAL.println(charVal);
    if (mqttClient.publish(temptopic, 2, false, charVal) == 0) {
      SERIAL.println("Mqtt Failed");}

    // SERIAL.print("Heat Setpoint: ");
    // char heatVal[10];
    // dtostrf(heatsetpoint, 4, 2, heatVal);
    // SERIAL.println(heatVal);
    // if (mqttClient.publish(setpoint_status, 2, false, heatVal) == 0) {
    //   SERIAL.println("Mqtt Failed");}

    SERIAL.print("Pressure: ");
    char pressval[10];
    dtostrf(pressure, 2, 0, pressval);
    SERIAL.println(pressval);
    if (mqttClient.publish(pressuretopic, 2, false, pressval) == 0) {
      SERIAL.println("Mqtt Failed");}
    
    if (mqttClient.publish(eventtopic, 2, false, "Alive") == 0) {
      SERIAL.println("Mqtt Failed");}
    vTaskDelay(MQTT_UPDATE_FREQ / portTICK_PERIOD_MS);
  }
}

void onMqttConnect(bool sessionPresent) {
    SERIAL.println("Connected to MQTT.");
    SERIAL.print("Session present: ");
    SERIAL.println(sessionPresent);

    uint16_t pumpsettingPub = mqttClient.subscribe(pumpsetting, 0);
    SERIAL.print("Subscribed to pump topic: ");
    SERIAL.println(pumpsettingPub);

    uint16_t heateronPub = mqttClient.subscribe(heater_control, 0);
    SERIAL.print("Subscribed to heat control topic: ");
    SERIAL.println(heateronPub);

    uint16_t heatersetpointPub = mqttClient.subscribe(heater_setpoint, 0);
    SERIAL.print("Subscribed to heat setpoint topic: ");
    SERIAL.println(heatersetpointPub);

    if (mqttClient.publish(eventtopic, 2, false, "Connected to MQTT") == 0) {
      SERIAL.println("Mqtt Failed");}

    xTaskCreate(UpdateStatus, "UpdaterTask", 4000, NULL, 1, &xUpdateStatus);
    xTaskCreate(GetTempTask, "TempTask", 20000, NULL, 1, &xGetTempTask);
    xTaskCreate(runHeater, "HeaterTask", 20000, NULL, 1, &xrunHeater);
    xTaskCreate(PressureTask, "PressureReadTask", 5000, NULL, 1, &xpressureTask);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    SERIAL.println("Disconnected from MQTT.");

    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }

    vTaskDelete(xUpdateStatus);
    vTaskDelete(xGetTempTask);
    vTaskDelete(xrunHeater);    

}

void setup() {
    SERIAL.begin(74880);
    delay(100);
    // SERIAL.begin(115200);
    // SERIAL.println();
    // SERIAL.println();

    sensors.begin(); //Onewire temp sensor bus
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    pinMode(HEATER_RELAY, OUTPUT);          //Init GPIO for heater relay
    digitalWrite(HEATER_RELAY, HIGH);

    pinMode(PUMP_RELAY_1, OUTPUT);          //Init GPIO for pump control relays
    digitalWrite(PUMP_RELAY_1, HIGH);
    pinMode(PUMP_RELAY_2, OUTPUT);
    digitalWrite(PUMP_RELAY_2, HIGH);
    
    pinMode(PRESSURE_PIN, INPUT);           //Init GPIO for analog read of pressure sensor
    analogReadResolution(8);                //Dampen resolution. Mapping from 0-30, I don't need 12 bit resolution on the ADC

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

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { //ElegantOTA
      request->send(200, "text/plain", "Hi! I am ESP32.");
    });
    AsyncElegantOTA.begin(&server);    // Start ElegantOTA

    connectToWifi();

    vTaskStartScheduler();
}

void loop() {
  AsyncElegantOTA.loop(); //ElegantOTA
  SerialAndTelnet.handle();

  if (SERIAL.available() > 0) {
    char c = SERIAL.read();
    switch (c) {
      case '\r':
        SERIAL.println();
        break;
      // case 'C':
      //   SERIAL.print("\nConnecting ");
      //   WiFi.begin(ssid, password);
      //   waitForConnection();
      //   break;
      // case 'D':
      //   SERIAL.print("\nDisconnecting ...");
      //   WiFi.disconnect();
      //   waitForDisconnection();
      //   break;
      // case 'R':
      //   SERIAL.print("\nReconnecting ");
      //   WiFi.reconnect();
      //   waitForDisconnection();
      //   waitForConnection();
      //   break;
      default:
        SERIAL.print(c);
        break;
    }
  }
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
}