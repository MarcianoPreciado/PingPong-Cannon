/*
  Created by: Marciano C. Preciado, Matt D. Ludlow
  Latest Update: 04-14-16

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

Cannon Wallace;
Servo cannonServo;
Servo loaderServo;

const int LED = 13;
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
  while(Serial.available() <= 0);
  Serial.read();
  //for (i = 0; i < 6; i++) {
    //while (Serial.available() < 2); // waiting for coordinates to come through
    //coordinates[i][0] = Serial.read();
    //coordinates[i][1] = Serial.read();
    //launchAngles[i] = Wallace.getLaunchAngle(angleLowerBound, angleUpperBound, coordinates[i][0]);
    //servoAngles[i] = Wallace.servoAngle(launchAngles[i]);
  //}
  
  cannonServo.attach(9);            // attach servos
  loaderServo.attach(10);
  loaderServo.write(20);
  cannonServo.write(35);
  Wallace.returnHome(lastState, pos);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}

void loop() {
int j;
  for (j = 0; j < 6; j++) {
    Serial.println(coordinates[j][0]);
    Serial.println(servoAngles[j]);
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
