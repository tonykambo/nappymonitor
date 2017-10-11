/**
 * Nappy Monitor
 *
 * Author: Tony Kambourakis
 * License: Apache License v2
 */

extern "C" {
  #include "user_interface.h"
  #include "gpio.h"
}

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Time.h>
#include "SensitiveConfig.h"

#include "Adafruit_SHT31.h"

void callback(char* topic, byte* payload, unsigned int length);

float nappyTempC = 0;
float nappyHumid = 0;
uint16_t volts = 0;
int counter = 0;

const char* ssid = WIFI_SSID;

//@WiFiChange
//const char* password = WIFI_PASSWORD;

// IBM Watson IoT PubSub configuration

char server[] = IOT_ORG IOT_BASE_URL;
char authMethod[] = "use-token-auth";
char token[] = IOT_TOKEN;
char clientId[] = "d:" IOT_ORG ":" IOT_DEVICE_TYPE ":" IOT_DEVICE_ID;

// MQTT topics

char sensorTopic[] = "iot-2/evt/sensor/fmt/json";

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

//Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// Temperature and Humidity sensor

Adafruit_SHT31 sht31 = Adafruit_SHT31();


// Timers

os_timer_t sensorTimer;
bool isSensorTimerComplete = false;

void initSensorTimer() {
  os_timer_setfn(&sensorTimer, sensorTimerFinished, NULL);
}

void startSensorTimer() {
  os_timer_arm(&sensorTimer, 60000, true);
}

void stopSensorTimer() {
  os_timer_disarm(&sensorTimer);
}

// JSON buffer size
const int BUFFER_SIZE = JSON_OBJECT_SIZE(10);





// Configuration and Settings Functions

void configureOTA() {
  Serial.println("Initialising OTA");
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname(IOT_DEVICE_ID);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

    ArduinoOTA.onStart([]() {
      Serial.println("Starting OTA update");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnding OTA update");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.print("Test: ");
    Serial.println(WiFi.localIP());
}

String macToStr(const uint8_t* mac)
{
  String result;

  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
    result += ':';
    }
  return result;
}

void printMacAddress() {
  String clientMac = "";
  unsigned char mac[6];
  WiFi.macAddress(mac);
  clientMac += macToStr(mac);
  Serial.print("Mac address is...");
  Serial.println(clientMac);
}

void init_wifi() {
  Serial.println("Nappy Monitor v1");
  Serial.println("Initialising Wifi");
  WiFi.mode(WIFI_STA);
  printMacAddress();
  Serial.print("Connecting to ");
  Serial.println(ssid);


  if (strcmp (WiFi.SSID().c_str(), ssid) != 0) {
    //@WiFiChange
    //WiFi.begin(ssid, password);
    WiFi.begin(ssid);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected with OTA4, IP address: ");
  Serial.println(WiFi.localIP());
  //publishDebug("WiFi Connected with OTA on IP: "+WiFi.localIP());
  Serial.println("About to configure OTA");
  configureOTA();
  Serial.println("Configured OTA");
  Serial.println("Checking WiFi again");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
}


void connectWithBroker() {

  Serial.println("Calling HTTP first");
  WiFiClient httpClient;

  while(httpClient.available()){
      String line = httpClient.readStringUntil('\r');
      Serial.print(line);
  }

  if (!!!client.connected()) {
    Serial.print("Reconnecting client to ");
    Serial.println(server);

    int cursorPosition = 0;
    char connectionStateString[20];
    int connectionState;

    Serial.println("Delaying for 10 seconds before trying to connect with Broker");
    delay(10000);
    while (!!!client.connect(clientId, authMethod, token)) {

      connectionState = client.state();
      sprintf(connectionStateString,"state=%d",connectionState);
      Serial.println(connectionStateString);
      Serial.print(".");
      delay(500);
    }

    client.subscribe("iot-2/cmd/config/fmt/json");
    Serial.println();
  }
}

void setup()  {
  // Configure serial port
  Serial.begin(115200);
  Serial.println(server);
  Serial.println(clientId);
  pinMode(D3, OUTPUT);
//  digitalWrite(D1, 1); // switch off
//  delay(3000);
  digitalWrite(D3, 1);
  init_wifi();
  connectWithBroker();
  Serial.println("sensor is starting..");

  // if (!tempsensor.begin()) {
  //   Serial.println("Couldn't find MCP9808!");
  // }

  if (!sht31.begin(0x44)) {
    Serial.println("sensor SHT31 couldn't be found");
  } else {
    Serial.println("found SHT31-D sensor");
  }

  // Start the sensor timer

  initSensorTimer();
  startSensorTimer();
}

void loop() {

  if (isSensorTimerComplete == true) {
    isSensorTimerComplete = false;
    readSensorData();
    sendSensorData();


  }

  // Check if the WiFI is still connected
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
  }

  if (!!!client.connected()) {
    connectWithBroker();
  }

  // read analog temp sensor for battery

  // int reading = analogRead(A0);
  //
  // float voltage = reading * 3.3;
  // voltage /= 1024.0;
  //
  // float temperatureC = (voltage - 0.5) * 100;
  // Serial.print(temperatureC);
  // Serial.println(" degrees C");


  ArduinoOTA.handle();
  client.loop();
}

void sensorTimerFinished(void *pArg) {
  isSensorTimerComplete = true;
}

void sendSensorData() {

  if (!isnan(nappyTempC) && !isnan(nappyHumid)) {
      // we have good data, send it
      Serial.print(nappyTempC);
      Serial.println(" degrees C");
      Serial.print(nappyHumid);
      Serial.println(" % humidity");

      String deviceId(IOT_DEVICE_ID);

      String payload = "{\"d\":{\"myName\":\"ESP8266.Test1\",\"counter\":";

      int hic = 0;

      payload += counter;
      payload += ",\"volts\":";
      payload += volts;
      payload += ",\"temperature\":";
      payload += nappyTempC;
      payload += ",\"humidity\":";
      payload += nappyHumid;
      payload += ",\"heatIndex\":";
      payload += hic;
      payload += ",\"deviceId\":";

      //payload += ",\"deviceId\":\""+IOT_DEVICE_ID+"\"";
      payload += "\""+deviceId+"\"";
      payload += "}}";

      Serial.print("Sending payload: ");
      Serial.println(payload);

      if (client.publish(sensorTopic, (char*) payload.c_str())) {
        Serial.println("Publish ok");
      } else {
        Serial.print("Publish failed with error:");
        Serial.println(client.state());
      }




  } else {
    Serial.println("Could not read sensor data");
  }
}

void readSensorData() {
  nappyTempC = sht31.readTemperature();
  nappyHumid = sht31.readHumidity();

}


void processJson(char * message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  // publishDebug("processing Json");
  // publishDebug("processing raw message="+String(message));

  if (root.containsKey("config")) {
    // if (strcmp(root["config"], "here") == 0) {
    //   publishDebug("setting presenceStatus to true");
    //   presenceStatus = true;
    //   //displayLCD();
    //   lcd.setCursor(0,3);
    //   lcd.print("                    ");
    //   lcd.setCursor(0,3);
    //   lcd.print("Tony is in today");
    //   digitalWrite(OWNER_STATUS_IN, 1); // switch off
    //   digitalWrite(OWNER_STATUS_OUT, 0);
    //   digitalWrite(OWNER_MESSAGE, 0);
    //
    //   stopJokeRequestTimer();

  } //else if (strcmp(root["status"],"away") == 0) {
  //     publishDebug("setting presenceStatus to false");
  //     presenceStatus = false;
  //     //displayLCD();
  //     lcd.setCursor(0,3);
  //     lcd.print("                    ");
  //     lcd.setCursor(0,3);
  //     lcd.print("Tony is away today");
  //
  //     digitalWrite(OWNER_STATUS_IN, 0); // switch off
  //     digitalWrite(OWNER_STATUS_OUT, 1);
  //     digitalWrite(OWNER_MESSAGE, 0);
  //
  //     // Start the Joke timer to display jokes
  //
  //     startJokeRequestTimer();
  //     requestJoke();
  //
  //   } else {
  //     lcd.setCursor(0,3);
  //     lcd.print("                    ");
  //     lcd.setCursor(0,3);
  //     lcd.print("Important Message");
  //
  //     digitalWrite(OWNER_STATUS_IN, 0); // switch off
  //     digitalWrite(OWNER_STATUS_OUT, 0);
  //     digitalWrite(OWNER_MESSAGE, 1);
  //
  //   }
  // } else if (root.containsKey("joke")) {
  //
  //   // TODO:  start a timer to display the joke for a brief amount of time and then
  //   // return to the main display
  //
  //   digitalWrite(OWNER_STATUS_IN, 0); // switch off
  //   digitalWrite(OWNER_STATUS_OUT, 0);
  //   digitalWrite(OWNER_MESSAGE, 1);
  //
  //   String joke = root["joke"];
  //   displayJoke(joke);
  //   startJokesDisplayTimer();
  // } else if (root.containsKey("travelTime")) {
  //   // TODO: Display travel time to Host
  //   publishDebug("Received travelTime");
  //
  //   //update travelTime
  //
  //   travelTime = root["travelTime"];
  //   displayLCD();
  //
  // } else if (root.containsKey("outsideClimate")) {
  //
  //   // TODO: read outside climate
  //
  //   publishDebug("Received outsideClimate data");
  //   outsideTemp = root["outsideClimate"]["outsideTemp"];
  //   outsideHumid = root["outsideClimate"]["outsideHumid"];
  //
  //   String outsideTempString = "Outside Temp String="+String(outsideTemp)+"C";
  //   String outsideHumidString = "Outside Humid String="+String(outsideHumid)+"%";
  //   Serial.println("Received outside climate with "+outsideTempString+" and "+outsideHumidString);
  //   displayLCD();
  //
  //   //publishDebug(outsideSt);
  // }

}


void callback(char* topic, byte* payload, unsigned int length) {
 Serial.println("callback invoked");
 char message_buff[length+1];
 strncpy(message_buff,(char *)payload, length);
 message_buff[length] = '\0';
 String topicString = String(topic);
 String payloadString = String(message_buff);

 processJson(message_buff);

 String callBackDetails = "callback received on topic ["+topicString+"] with payload ["+payloadString+"]";

 Serial.println(callBackDetails);
}
