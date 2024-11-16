#include <Arduino.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <lookup.hpp>
#include <angle_converter.hpp>

#define ESP_ID 0

// User defined functions declaration
// Position Control
void readEncoder();
void forward();
void reverse();
void pwmOut(int);



// MQTT
void reconnect_wifi();
void reconnect_mqtt();
void callback(char*, byte*, unsigned int);

void IRAM_ATTR prevCallback();
void IRAM_ATTR nextCallback();


#define DEBUG
const long MONITOR_BAUD_RATE = 115200;

// PUSH BUTTONS
#define PREV 16
#define NEXT 17

bool prevPressed=false;
bool nextPressed=false;

volatile unsigned long lastPrevInterruptTime = 0;
volatile unsigned long lastNextInterruptTime = 0;
#define DEBOUNCE_DELAY 100


// POSITION CONTROL
// Pins to motor driver
#define IN1 18
#define IN2 19
#define PWM 23

// Encoder Pins
#define ENC1 25
#define ENC2 26

volatile int encoderCount = 0; 
const int PPR=150;
const float angle=360.0;
const int units=8;
const float unitAngle=angle/units;

float targetAngle=0;
int targetCount=0;

// PID define
double kp = 3.5 , ki = 0.5 , kd = 0.05;            
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  

// target change time
// unsigned long prevT=millis();
// unsigned long interval=5000;
// unsigned long currT;



// MQTT
const char* ssid = "Milkyway";
const char* password = "12345678";
const char* mqtt_server = "192.168.107.178";
const int mqtt_port = 1883;
const char* dataTopic = "data/characters";
const char* controlTopic = "control/nav";

WiFiClient espClient;
PubSubClient client(espClient);

// unsigned long previousMillisWifi=millis();
// unsigned long reconnectInterval=5000;

char receivedPayload[10];

// Braille
std::map<char, String> brailleMap = initializeBrailleMap();


void setup() {
  // Initialize serial monitor
  #ifdef DEBUG 
    Serial.begin(MONITOR_BAUD_RATE);
  #endif

  // INIT ENCODER PINS
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1),readEncoder,RISING);

  // INIT MOTOR DRIVER PINS
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);

  // SET PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-150, 150);

  // Init wifi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  #ifdef DEBUG
    Serial.print("Connecting to ");
    Serial.println(ssid);
  #endif
  while (WiFi.status() != WL_CONNECTED) {
    #ifdef DEBUG
      Serial.print('.');
    #endif
    delay(500);
  }
  #ifdef DEBUG
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("Your Local IP address is: ");
    Serial.println(WiFi.localIP());      /*Print the Local IP*/
  #endif

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (!client.connected()){
    reconnect_mqtt();
  }

  client.subscribe(dataTopic);

  // INPUTS
  pinMode(PREV, INPUT);
  pinMode(NEXT, INPUT);

  attachInterrupt(digitalPinToInterrupt(PREV), prevCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(NEXT), nextCallback, RISING);

}

void loop() {

  // Set target
  // currT = millis();

  // if (currT - prevT > interval){
  //   if (targetAngle < 360.0){
  //     targetAngle+=unitAngle;
  //   }
  //   else{
  //     targetAngle=0.0;
  //     encoderCount=0;
  //   }
  //   prevT=currT;
  // }

  // Map angle to PPR
  // targetCount = map(targetAngle, 0, 360, 0, PPR);
  targetCount = (int)((targetAngle/angle)*PPR);

  // Compute PID
  setpoint = targetCount;
  input = encoderCount;

  myPID.Compute();
  pwmOut(output);

  #ifdef DEBUG
    // Serial.print("setpoint: ");
    // Serial.print(setpoint);
    // Serial.print(" input: ");
    // Serial.print(input);
    // Serial.print(" output: ");
    // Serial.println(output);
  #endif

  // MQTT
  client.loop();
  if (WiFi.status() != WL_CONNECTED) {
    #ifdef DEBUG
      Serial.println("Wifi connection lost...");
      Serial.println("Reconnecting");
    #endif
    reconnect_wifi();
  }

  if (prevPressed){
    Serial.println("10");
    client.publish(controlTopic, "10");
    prevPressed=false;
  }
  if (nextPressed){
    client.publish(controlTopic, "01");
    nextPressed=false;
  }
}


void readEncoder(){
  int b = digitalRead(ENC2);
  if(b > 0){
    encoderCount++;
  }
  else{
    encoderCount--;
  }
}

void pwmOut(int out) {                               
  analogWrite(PWM, abs(out));         
  if (out > 0) {                           
    // forward(); 
    reverse();  
  }
  else {
    // reverse();  
    forward();                          
  }
}

void forward () {
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
}

void reverse () {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
}


void reconnect_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();   
  }
}

void reconnect_mqtt(){
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    #ifdef DEBUG
      Serial.printf("The client %s connects to the local MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str())) {
        Serial.println("Local MQTT broker connected");
      } 
    #endif
    else 
    {
      #ifdef DEBUG
        Serial.print("failed with state ");
        Serial.println(client.state());
      #endif
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    #ifdef DEBUG
      // Serial.print((char) payload[i]);
    #endif
    receivedPayload[i]=(char) payload[i];
  }
  String targetString = getBraille(receivedPayload[0], brailleMap);
  String part1, part2;
  divideString(targetString, part1, part2);
  targetAngle=greyCode2angle(part1);
  #ifdef DEBUG
    // Serial.print("Target Angle: ");
    // Serial.println(targetAngle);
  #endif
}

void IRAM_ATTR prevCallback(){
  unsigned long currentTime = millis();
  if (currentTime - lastPrevInterruptTime > DEBOUNCE_DELAY) {
    prevPressed = true;
    lastPrevInterruptTime = currentTime;
  }

}

void IRAM_ATTR nextCallback(){
  unsigned long currentTime = millis();
  if (currentTime - lastNextInterruptTime > DEBOUNCE_DELAY) {
    nextPressed = true;
    lastNextInterruptTime = currentTime;
  }
}