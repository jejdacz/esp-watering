#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "TaskManager.h"

#define FIRSTRUN_SEED 129
#define MAX_SLEEP_TIME 60 * 60
#define MIN_SLEEP_TIME 1

#define WATERING_PERIOD 24 * 60 * 60
#define WATERING_DURATION 40 * 60

typedef struct
{
  uint32_t seconds;
  uint32_t checkSum;
  uint32_t sleepTime;
} rtcMemStore __attribute__((aligned(4)));

rtcMemStore rtcData;
TaskManager taskManager;

const char *ssid = "********";
const char *password = "********";
const char *mqtt_server = "********";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup()
{
  uint32_t timeStamp = millis();
  
  // setup PWM
  analogWriteRange(100);
  analogWriteFreq(100); // ESP doesn support less than 100Hz

  pinMode(D6, INPUT_PULLUP);

  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH);

  Serial.begin(115200);
  delay(500);
  Serial.println();

  if (!digitalRead(D6))
  {
    Serial.println("starting OTA mode");
    closeValve();    
    if (!connectWifi()) {
      blink(D7,3,1000);
      delay(5000);
      ESP.restart();
    }
    connectMQTT();    
    startOTA();
    Serial.println("ready for OTA");
    client.publish("esp-garden", "ready for OTA");
    return;
  }

  if (ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData)))
  {
    Serial.println("Read global... ");
    Serial.print("seconds: ");
    Serial.println(rtcData.seconds);
  }

  if (rtcData.checkSum != calcCheckSum())
  {
    Serial.println("checksum failed... ");
    Serial.print("FIRSTRUN_SEED: ");
    Serial.println(FIRSTRUN_SEED);

    connect();
    client.publish("esp-garden", "device powered on");

    rtcData.seconds = millis() / 1000;
    rtcData.sleepTime = 0;

    closeValve();

    taskManager.addTask(&startWatering, rtcData.seconds + 0);
  }
  else
  {
    if (ESP.rtcUserMemoryRead(16, (uint32_t *)taskManager.getTaskList(), taskManager.getTaskListByteSize()))
    {
      Serial.println("Read task list: ");
    }
  }

  rtcData.seconds += rtcData.sleepTime;

  taskManager.handle(rtcData.seconds);

  rtcData.seconds += (millis() - timeStamp) / 1000;
  rtcData.sleepTime = max((uint32_t)MIN_SLEEP_TIME, min((uint32_t)(MAX_SLEEP_TIME), taskManager.nextTaskTime(rtcData.seconds) - rtcData.seconds));

  Serial.print("Seconds: ");
  Serial.println(rtcData.seconds);

  rtcData.checkSum = calcCheckSum();

  if (ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData)))
  {
    Serial.println("Writing global data...");
  }

  if (ESP.rtcUserMemoryWrite(16, (uint32_t *)taskManager.getTaskList(), taskManager.getTaskListByteSize()))
  {
    Serial.println("Writing task list...");
  }

  // Serial.println(taskManager.getTaskListSize());

  Serial.print("next task time: ");
  Serial.println(taskManager.nextTaskTime(rtcData.seconds));

  Serial.print("going sleep for... ");
  Serial.println(rtcData.sleepTime);

  ESP.deepSleep(rtcData.sleepTime * 1e6);
}

void loop()
{
  ArduinoOTA.handle();
}

uint32_t calcCheckSum()
{
  return FIRSTRUN_SEED + rtcData.seconds;
}

void startWatering()
{
  taskManager.addTask(&startWatering, rtcData.seconds + WATERING_PERIOD);

  connect();
  client.publish("esp-garden", "processing TASK");

  // read sensor and maybe start watering
  if (readSoilSensor())
  {
    taskManager.addTask(&stopWatering, rtcData.seconds + WATERING_DURATION);
    // open valve
    Serial.println("opening valve");
    client.publish("esp-garden", "opening valve");
    openValve();
  }
  else
  {
    Serial.println("skipping watering");
    client.publish("esp-garden", "skipping watering");
  }
}

void stopWatering()
{
  Serial.println("closing valve");

  connect();
  client.publish("esp-garden", "closing valve");

  closeValve();
}

void openValve()
{
  analogWrite(D2, 5); //pos 0 deg
  delay(2000);        // safety delay for servo rotation
}

void closeValve()
{
  analogWrite(D2, 14); // pos 90 deg
  delay(2000);         // safety delay for servo rotation
}

int readSoilSensor()
{
  // set D5 GPIO14 HIGH
  pinMode(D5, OUTPUT); // soil sensor comparator Vcc pin
  delay(100);
  digitalWrite(D5, HIGH);
  delay(100);
  return digitalRead(D1);
}

bool connectWifi()
{
  if (WiFi.status() == WL_CONNECTED)
    return true;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Attempting WiFi connection...");

  uint32_t tries = 20;

  while ((WiFi.status() != WL_CONNECTED) && tries != 0)
  {
    delay(500);
    tries--;
    Serial.print(".");
  }

  if (tries == 0)
    return false;

  Serial.println("connected");

  return true;
}

bool connectMQTT()
{
  if (client.connected())
    return true;

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  if (!client.connected())
  {
    reconnect();
  }

  //client.loop();
  return client.connected();
}

void connect()
{
  if (connectWifi())
  {
    connectMQTT();
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect()
{
  // Loop until we're reconnected
  int tries = 3;
  while (!client.connected() && tries != 0)
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266";
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("esp-garden", "connected");
      // ... and resubscribe
      //client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      tries--;
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}

void startOTA()
{  
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);    
    ESP.restart();
  }

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      Serial.println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      Serial.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void blink(int port, int n, int d) {  
  for(int i = 0; i <= n; i++) {
    digitalWrite(port, LOW);
    delay(d);
    digitalWrite(port, HIGH);
    delay((int)(d/2));
  }
  digitalWrite(port, LOW);
}
