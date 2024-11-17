#include <Arduino.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <lookup.hpp>
#include <angle_converter.hpp>

#define ESP_ID 0

// User defined functions declaration
// Position Control
void readEncoderCount1();
void readEncoderCount2();
void forward(int in1, int in2);
void reverse(int in1, int in2);
void pwmOut(int in1, int in2, int en, int out);



// MQTT
void reconnect_wifi();
void reconnect_mqtt();
void callback(char*, byte*, unsigned int);

void IRAM_ATTR prevCallback();
void IRAM_ATTR nextCallback();

void sendControlMessage(void * pvParameters);

#define DEBUG
const long MONITOR_BAUD_RATE = 115200;

// PUSH BUTTONS
#define PREV 16
#define NEXT 17

bool prevPressed=false;
bool nextPressed=false;

volatile unsigned long lastPrevInterruptTime = 0;
volatile unsigned long lastNextInterruptTime = 0;
#define DEBOUNCE_DELAY 200


// POSITION CONTROL
// Pins to motor driver
#define MOTOR2_IN1 18
#define MOTOR2_IN2 19
#define MOTOR2_PWM 23

#define MOTOR1_IN1 32
#define MOTOR1_IN2 33
#define MOTOR1_PWM 27

// Encoder Pins
#define MOTOR2_ENC1 25
#define MOTOR2_ENC2 26

#define MOTOR1_ENC1 13
#define MOTOR1_ENC2 14

volatile int encoderCount1 = 0; 
volatile int encoderCount2 = 0; 
const int PPR1=150;
const int PPR2=150;
const float angle=360.0;
const int units=8;
const float unitAngle=angle/units;

float targetAngle1=0;
int targetCount1=0;

float targetAngle2=0;
int targetCount2=0;

// PID define
double kp1 = 3.5 , ki1 = 0.5 , kd1 = 0.05;            
double input1 = 0, output1 = 0, setpoint1 = 0;
PID myPID1(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);  

double kp2 = 4.0 , ki2 = 0.5 , kd2 = 0.005;            
double input2 = 0, output2 = 0, setpoint2 = 0;
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);  

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
  pinMode(MOTOR1_ENC1, INPUT);
  pinMode(MOTOR1_ENC2, INPUT);

  pinMode(MOTOR2_ENC1, INPUT);
  pinMode(MOTOR2_ENC2, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC1),readEncoderCount1,RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC1),readEncoderCount2,RISING);

  // INIT MOTOR DRIVER PINS
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);

  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  // SET PID
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(1);
  myPID1.SetOutputLimits(-150, 150);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(1);
  myPID2.SetOutputLimits(-150, 150);

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

  xTaskCreatePinnedToCore(
    sendControlMessage,   /* Task function. */
    "sendControlMessage",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    NULL,        /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
}

void loop() {

  // Set target
  // currT = millis();

  // if (currT - prevT > interval){
  //   if (targetAngle1 < 360.0){
  //     targetAngle1+=unitAngle;
  //   }
  //   else{
  //     targetAngle1=0.0;
  //     encoderCount1=0;
  //   }
  //   prevT=currT;
  // }

  // Map angle to PPR
  // targetCount1 = map(targetAngle1, 0, 360, 0, PPR1);
  targetCount1 = (int)((targetAngle1/angle)*PPR1);

  targetCount2 = (int)((targetAngle2/angle)*PPR2);

  // Compute PID
  setpoint1 = targetCount1;
  input1 = encoderCount1;

  setpoint2 = targetCount2;
  input2 = encoderCount2;

  myPID1.Compute();
  pwmOut(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, output1);

  myPID2.Compute();
  pwmOut(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, output2);

  #ifdef DEBUG
    // Serial.print("setpoint1: ");
    // Serial.print(setpoint1);
    // Serial.print(" input1: ");
    // Serial.print(input1);
    // Serial.print(" output1: ");
    // Serial.println(output1);

    Serial.print("setpoint2: ");
    Serial.print(setpoint2);
    Serial.print(" input2: ");
    Serial.print(input2);
    Serial.print(" output2: ");
    Serial.println(output2);
  #endif

  // // MQTT
  // client.loop();
  // if (WiFi.status() != WL_CONNECTED) {
  //   #ifdef DEBUG
  //     Serial.println("Wifi connection lost...");
  //     Serial.println("Reconnecting");
  //   #endif
  //   reconnect_wifi();
  // }

  // if (prevPressed){
  //   client.publish(controlTopic, "10");
  //   prevPressed=false;
  // }
  // if (nextPressed){
  //   client.publish(controlTopic, "01");
  //   nextPressed=false;
  // }
}


void readEncoderCount1(){
  int b = digitalRead(MOTOR1_ENC2);
  if(b > 0){
    encoderCount1++;
  }
  else{
    encoderCount1--;
  }
}

void readEncoderCount2(){
  int b = digitalRead(MOTOR2_ENC2);
  if(b > 0){
    encoderCount2++;
  }
  else{
    encoderCount2--;
  }
}

void pwmOut(int in1, int in2, int en, int out) {                               
  analogWrite(en, abs(out));         
  if (out > 0) {                           
    // forward(); 
    reverse(in1, in2);  
  }
  else {
    // reverse();  
    forward(in1, in2);                          
  }
}

void forward (int in1, int in2) {
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW); 
}

void reverse (int in1, int in2) {
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
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
  targetAngle1=greyCode2angle(part1);
  targetAngle2=greyCode2angle(part2);
  #ifdef DEBUG
    // Serial.print("Target Angle: ");
    // Serial.println(targetAngle1);
  #endif
}

void IRAM_ATTR prevCallback(){
  unsigned long currentTime = millis();
  if (currentTime - lastPrevInterruptTime > DEBOUNCE_DELAY) {
    // client.publish(controlTopic, "10");
    prevPressed = true;
    lastPrevInterruptTime = currentTime;
  }

}

void IRAM_ATTR nextCallback(){
  unsigned long currentTime = millis();
  if (currentTime - lastNextInterruptTime > DEBOUNCE_DELAY) {
    // client.publish(controlTopic, "01");

    nextPressed = true;
    lastNextInterruptTime = currentTime;
  }
}

void sendControlMessage(void * pvParameters){
  while(1){
    if (prevPressed){
      client.publish(controlTopic, "10");
      prevPressed=false;
    }
    if (nextPressed){
      client.publish(controlTopic, "01");
      nextPressed=false;
    }

     // MQTT
  client.loop();
  if (WiFi.status() != WL_CONNECTED) {
    #ifdef DEBUG
      Serial.println("Wifi connection lost...");
      Serial.println("Reconnecting");
    #endif
    reconnect_wifi();
  }
    vTaskDelay(DEBOUNCE_DELAY);
  }
}