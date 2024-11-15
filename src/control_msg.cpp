#include <Arduino.h>
// #include <ArduinoJson.h>

#define PREV 1
#define NEXT 2


// User defined functions
void sendPrevControlMessage();
void sendNextControlMessage();

void sendControlMessage(char*);

void setup(){
  // set as INPUT pins
  // EXTERNAL PULLUP
  pinMode(PREV, INPUT);
  pinMode(NEXT, INPUT);

  // controlMessage["prev"]=1;
  // controlMessage["next"]=1;

  attachInterrupt(digitalPinToInterrupt(PREV), sendPrevControlMessage, FALLING);
  attachInterrupt(digitalPinToInterrupt(NEXT), sendNextControlMessage, FALLING);

}

void loop(){

}

void sendPrevControlMessage(){
  // controlMessage["prev"]=0;
  // controlMessage["next"]=0;
  char* prevNext="10";
  
}