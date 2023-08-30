#include <SoftwareSerial.h>     // Library for "speaking" with Arduino on UART
#include <Wire.h>

//constant values
#define eth 0.0488692
#define deth 0.07801622

//TO-DO: Move this to different file
#define MotorSpeed2 5 //pwm speed pin for motor
#define MotorSpeed1 3 //pwm speed pin for motor

int MOTORSPEED;
//global vars
String str; //string for reading bt data
unsigned long initTime; //time that app starts at, not sure if needed anymore
unsigned long prevTime; //previous time reading for angle measurement
float theta; // current theta
float dth; //curent dth val
long sumTh;
int curVel; //current velocity

//TO-DO: Declare BT connection here


void setup() {
  // put your setup code here, to run once:
  //Declare pins and initialize bluetooth connection and logging

  //Set global vars
  theta = 0;
  dth = 0;
  sumTh = 0;
  curVel = 0;
  MOTORSPEED = 3;

}

void loop() {
  // put your main code here, to run repeatedly:

  double aL = 0.29; //the lower a value as determined in the paper
  double aS = 0.93; //the upper a value as determined in the paper

  double a = 0; //the value for which the velocity is multiplied by
  double th = theta; //finds the current theta

  double minThreshTh = 6; //relatively zero thresh for theta
  double maxThreshTh = 12; //relative max threshold for theta
  double minThreshdTh = 100; //relative min thresh for dTheta

  //see flowchart in CDR presentation for full breakdown of this formula
  //this is directly pulled from the paper by Dr. L
  if(dth < 100 && dth > -150)
  {
    //Serial.println("nothing 1");
    analogWrite(MotorSpeed1, 2);
    analogWrite(MotorSpeed2, 0);
  }

  else if(th <= minThreshTh)
  {
    //middle of swing case
    //Serial.println("down");
     analogWrite(MotorSpeed1, 0);
    for(int i = 0; i < 10; i += 5){
      analogWrite(MotorSpeed2, i);
    } 
  }
  else if(th >= maxThreshTh){
    //ends of swing case
    //Serial.println("up");
    analogWrite(MotorSpeed2, 0);
      for(int i =0; i < 35; i += 10){
        analogWrite(MotorSpeed1, i);
      }    
  }

}
