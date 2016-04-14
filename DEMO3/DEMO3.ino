/*
  Created by: Marciano C. Preciado, Matthew D. Ludlow
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

#define pi 3.1415923565
Cannon Wallace;
Servo cannonServo;
Servo loaderServo;

const int LED = 13;

int angleLowerBound = 33;
int angleUpperBound = 83;
byte k = 0;

double encoderPos[6];
double xTarget_HB[6];
double xTarget_LB[6];
double xTarget_mm[6];
double xTarget_m[6];
int servoAngles[6];
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

  for (i = 0; i < 6; i++) {
    while (Serial.available() < 3);
    encoderPos[i] = Serial.read();
    xTarget_HB[i] = Serial.read();
    xTarget_LB[i] = Serial.read();

    xTarget_mm[i] = (256 * xTarget_HB[i]) + xTarget_LB[i];
    xTarget_m[i] = (xTarget_mm[i] / 1000);

    Serial.print("For target ");
    Serial.print(i + 1);
    Serial.print(",drive to stripe ");
    Serial.print(encoderPos[i]);
    Serial.print(", and aim for ");
    Serial.print(xTarget_m[i], 3);
    Serial.println(". ");

  }
  Serial.println("Initializing IR LED Protocol");
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);


  for (i = 0; i < 6; i++) {
    launchAngles[i] = Wallace.getLaunchAngle(angleLowerBound, angleUpperBound, xTarget_m[i]);
    servoAngles[i] = Wallace.servoAngle(launchAngles[i]);
    Serial.print("Target dist = ");
    Serial.print( xTarget_m[i]);
    Serial.print(" [m] ---> Servo angle = ");
    Serial.print(servoAngles[i]);
    Serial.println(" [deg]");
  }
  Serial.println("");
  
  cannonServo.attach(9);            // attach servos
  loaderServo.attach(10);
  Wallace.returnHome(lastState, pos); 
}

void loop() {
  int j;
  for (j = 0; j < 6; j++) {
    Wallace.moveTo(encoderPos[j], lastState, pos);
    cannonServo.write(servoAngles[j]);
    delay(350);
    analogWrite(SolenoidPowerPin, fullPower);
    delay(onTime);
    analogWrite(SolenoidPowerPin, 0);
    Wallace.reload(cannonServo, loaderServo, lastState, pos);
  }

  Wallace.returnHome(lastState, pos);           // finish game
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

}
