#include <Arduino.h>
#include <PID_v1.h>

#define DEBUG
const long MONITOR_BAUD_RATE = 115200;

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
float targetCount=0;

// PID define
double kp = 4.0 , ki = 0.2 , kd = 0.01;            
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  

// target change time
unsigned long prevT=millis();
unsigned long interval=5000;
unsigned long currT;


// User defined functions declaration
void readEncoder();
void forward();
void reverse();
void pwmOut(int);

void setup() {
  // Initialize 
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

}

void loop() {


  // Set target
  currT = millis();

  if (currT - prevT > interval){
    if (targetAngle < 360.0){
      targetAngle+=unitAngle;
    }
    else{
      targetAngle=0.0;
      encoderCount=0;
    }
    prevT=currT;
  }

  // Map angle to PPR
  // targetCount = map(targetAngle, 0, 360, 0, PPR);
  targetCount = (int)((targetAngle/angle)*PPR);

  // Compute PID
  setpoint = targetCount;
  input = encoderCount;

  myPID.Compute();
  pwmOut(output);

  #ifdef DEBUG
    Serial.print("setpoint: ");
    Serial.print(setpoint);
    Serial.print(" input: ");
    Serial.print(input);
    Serial.print(" output: ");
    Serial.println(output);
  #endif

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