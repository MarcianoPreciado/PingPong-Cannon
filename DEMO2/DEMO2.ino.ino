/*
  Created by: Marciano C. Preciado
  Latest Update: 03-05-16

  This libary was created to facilitate the necessary calculations
  to operate the ME 1010 ping-pong cannon.

   _PINS_    _HARDWARE_        _VARIABLE_NAME_
  1  RX
  2  TX
  3  *~
  4         Motor Direction     MotorDirectionPin
  5  *~     Motor Power         MotorPowerPin
  6  *~     Solenoid Power      SolenoidPowerPin
  7         Solenoid Direction  SolenoidDirectionPin
  8
  9  *~     Aiming Servo        cannonServo     (Servo Instance)
  10 *~     Loader Servo        loaderServo     (Servo Instance)
  11 *~     Right Bumper        bumperR
  12        Left Bumper         bumperL
  13        IR LED              LED

  A0
  A1
  A2
  A3
  A4
  A5        IR Sensor           IR
  A6
  A7        Built-In Buttons    btn

**************************************************************************/


#include <Servo.h>
#include "PingPong.h"

#define L1 0.1313
#define L2 0.0475
#define L3 0.0880
#define L4 0.0960
// d_vector:
#define d1  0.031
#define d2  0.19
#define d3  0.067
// offsets:
#define thetaS0  5.9
#define thetaL0  14.0
// initial ball velocity:
#define v0  3.761904022909760

Cannon Wallace(L1, L2, L3, L4, d1, d2, d3, thetaS0, thetaL0, v0);
Servo cannonServo;
Servo loaderServo;

const int LED = 13;

double targetX;
double targetZ;
byte junk;
int angleLowerBound = 33;
int angleUpperBound = 83;
byte k = 0;

double coordinates[6][2] = {{1, 30},
                            {1.2, 25},
                            {.8, 20},
                            {.7, 15},
                            {1.3, 10},
                            {.9, 5}};
double servoAngles[6] = {30,50,70,90,110,130};
int pos;
bool lastState;

void setup() {
  pinMode(MotorDirectionPin, OUTPUT);
  pinMode(SolenoidDirectionPin, OUTPUT);
  pinMode(bumperL, INPUT_PULLUP);
  pinMode(bumperR, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);               // initialize communication
  Serial.write(97);                 // send 'a' confirmation value
  double launchAngles[6];
  int i;
  while(Serial.available() < 0);
  Serial.read();
  //for (i = 0; i < 6; i++) {
    //while (Serial.available() < 2); // waiting for coordinates to come through
    //coordinates[i][0] = Serial.read();
    //coordinates[i][1] = Serial.read();
    //launchAngles[i] = Wallace.getServoAngle(angleLowerBound, angleUpperBound, coordinates[i][0]);
    //servoAngles[i] = Wallace.servoAngle(launchAngles[i]);
  //}
  
  cannonServo.attach(9);            // attach servos
  loaderServo.attach(10);
  Wallace.returnHome(lastState, pos);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}

void loop() {
int j;
  for (j = 0; j < 6; j++) {
    Wallace.moveTo(coordinates[j][1], lastState, pos);
    cannonServo.write(servoAngles[j]);
    delay(350);
    analogWrite(SolenoidPowerPin, fullPower);
    delay(onTime);
    analogWrite(SolenoidPowerPin, 0);
    Wallace.reload(cannonServo, loaderServo,lastState, pos);
  }

  Wallace.returnHome(lastState, pos);           // finish game
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

}
