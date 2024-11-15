#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define DEBUG

void reconnect_wifi();
void reconnect_mqtt();
void callback(char*, byte*, unsigned int);

const char* ssid = "Milkyway";
const char* password = "12345678";
const char* mqtt_server = "192.168.107.178";
const int mqtt_port = 1883;
const char* topic = "data/alphabets";


unsigned long previousMillis = 0;
unsigned long interval = 30000;


WiFiClient espClient;
PubSubClient client(espClient);

void setup() {

  // setup delay
  delay(5000);

  Serial.begin(115200);

  // Init wifi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("Your Local IP address is: ");
  Serial.println(WiFi.localIP());      /*Print the Local IP*/



  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (!client.connected()){
    reconnect_mqtt();
  }

  client.subscribe(topic);
}

void loop(){

  client.loop();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wifi connection lost...");
    Serial.println("Reconnecting");
    reconnect_wifi();
  }

  // if (!client.connected()){
  //   Serial.println("Broker Connection lost...");
  //   Serial.println("Reconnecting...");
  //   reconnect_mqtt();
  // }

  // client.publish(topic, "hello world...");
}

void reconnect_wifi() {
 unsigned long currentMillis = millis();
 if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
   Serial.println("Reconnecting to WiFi...");
   WiFi.disconnect();
   WiFi.reconnect();   
   previousMillis = currentMillis;
 }
}

void reconnect_mqtt(){
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the local MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str())) {
      Serial.println("Local MQTT broker connected");
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}