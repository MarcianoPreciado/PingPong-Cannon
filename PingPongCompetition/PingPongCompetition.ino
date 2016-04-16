/*
  Created by: Marciano C. Preciado, Matt D. Ludlow
  Latest Update: 04-16-16

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

Cannon Wallace;               // Initializes Cannon instance (named Wallace)
Servo cannonServo;            // Initializes the servo used for aiming.
Servo loaderServo;            // Initializes the servo used for reloading.

const int LED = 13;           // Defines LED as 13

int angleLowerBound = 33;     // Used for 'getLaunchAngle' function
int angleUpperBound = 83;

double encoderPos[6];                                     // Vector for storing encoder positions
int solenoidPower[6] = {255, 255, 255, 255, 255, 255};    // default power is HIGH
int servoAngles[6];                                       // Vector for storing computed servo angles
int pos;                                                  // Stores position
bool lastState;                                           // Stores the color of the last recorded color bar. (Black = 1, White = 0)

void setup() {
  pinMode(MotorDirectionPin, OUTPUT);       // Setting the mode for each pin
  pinMode(SolenoidDirectionPin, OUTPUT);
  pinMode(bumperL, INPUT_PULLUP);
  pinMode(bumperR, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);                       // initialize serial communication
  Serial.write(97);                         // send the confirmation value ('a')

  double xTarget_HB[6];                     // Temporary vectors for storing recieved values
  double xTarget_LB[6];                     //    and steps between computations
  double xTarget_mm[6];
  double xTarget_m[6];
  double launchAngles[6];

  int i;
  for (i = 0; i < 6; i++) {
    while (Serial.available() < 3);       // Wait to receive all 3 values
    encoderPos[i] = Serial.read();        // Read and store the values
    xTarget_HB[i] = Serial.read();
    xTarget_LB[i] = Serial.read();

    xTarget_mm[i] = (256 * xTarget_HB[i]) + xTarget_LB[i];      // Converts messages into a target distance in [mm]
    xTarget_m[i] = (xTarget_mm[i] / 1000);                      // Converts target distance to [m]

    //Sends final vaules back to MATLAB
    Serial.print("For target ");
    Serial.print(i + 1);
    Serial.print(",drive to stripe ");
    Serial.print(encoderPos[i]);
    Serial.print(", and aim for ");
    Serial.print(xTarget_m[i], 3);
    Serial.println(". ");
  }

  Wallace.returnHome(lastState, pos);                           // Sends the cannon to the home position
  Serial.println("Initializing IR LED Protocol");
  digitalWrite(LED, HIGH);                                      // Start Timer
  delay(1000);
  digitalWrite(LED, LOW);

  for (i = 0; i < 6; i++) {
    if (xTarget_m[i] <= 0.94) {                                // If the desired distance is less than or equal to 94 cm, rewrite for LOW(240) power
      solenoidPower[i] = 240;                                  //   rather than use the default power: HIGH(255)
    }
    launchAngles[i] = Wallace.getLaunchAngle(angleLowerBound, angleUpperBound, xTarget_m[i]);
    servoAngles[i] = Wallace.servoAngle(launchAngles[i]);
    Serial.print("Target dist = ");
    Serial.print( xTarget_m[i]);
    Serial.print(" [m] ---> Servo angle = ");
    Serial.print(servoAngles[i]);
    Serial.println(" [deg]");
  }

  cannonServo.attach(9);            // Attach servos to required pins, and write them to starting positions.
  loaderServo.attach(10);
  loaderServo.write(20);
  cannonServo.write(35);
}

void loop() {
  cannonServo.write(servoAngles[0]);
  Wallace.moveTo(encoderPos[0], lastState, pos, true);

  analogWrite(SolenoidPowerPin, solenoidPower[0]);
  delay(onTime);
  analogWrite(SolenoidPowerPin, 0);
  int j;
  for (j = 1; j < 6; j++) {
    Wallace.reload(cannonServo, loaderServo, lastState, pos);
    Wallace.moveTo(32, lastState, pos, false);
    cannonServo.write(servoAngles[j]);
    Wallace.moveTo(encoderPos[j], lastState, pos, true);
    analogWrite(SolenoidPowerPin, solenoidPower[j]);
    delay(onTime);
    analogWrite(SolenoidPowerPin, 0);
  }

  Wallace.returnHome(lastState, pos);           // finish game
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("");
  while (true);
}
