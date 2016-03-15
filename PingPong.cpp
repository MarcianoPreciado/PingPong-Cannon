/*
* Created by: Marciano C. Preciado
* Latest Update: 03-05-16
*
* This libary was created to facilitate the necessary calculations
* to operate the ME 1010 ping-pong cannon.
*
*	_PINS_		_HARDWARE_			_VARIABLE_NAME_
*	1  RX
*	2  TX
*	3  *~
*	4			Motor Direction		MotorDirectionPin
*	5  *~		Motor Power			MotorPowerPin
*	6  *~		Solenoid Power		SolenoidPowerPin
*	7			Solenoid Direction	SolenoidDirectionPin
*	8
*	9  *~		Aiming Servo		Servo aim
*	10 *~		Loader Servo		Servo loader
*	11 *~		Right Bumper		bumperR
*	12			Left Bumper			bumperL
*	13
*
*	A0
*	A1
*	A2
*	A3
*	A4
*	A5			IR Sensor			IR
*	A6
*	A7			Built-In Buttons	btn
*/

#include "PingPong.h"
#include <math.h>
#include "Arduino.h"
#include "Servo.h" 

/*
* Cannon Constructor:
* Initializes all permanent properties of Cannon Object
*/
Cannon::Cannon(double L1, double L2, double L3, double L4, double d1, double d2, double d3, double thetaS0, double thetaL0, double v0)
{
	this->L1 = L1;
	this->L2 = L2;
	this->L3 = L3;
	this->L4 = L4;
	this->d1 = d1;
	this->d2 = d2;
	this->d3 = d3;
	this->thetaS0 = thetaS0;
	this->thetaL0 = thetaL0;
	this->v0 = v0;
}


/*
*				Landing Distance member function:
*-----------------------------------------------------------
*/

/*
* Calculates the initial coordinates of the pingpong ball
* Outputs: x0 and y0 respectively [m]
*/
double Cannon::initialX(double launchAngle)
{
	double theta = launchAngle * (3.14159265)/ 180;
	double x0 = (this->d2)*cos(theta) - (this->d3)*sin(theta);
	return x0;
}
double Cannon::initialY(double launchAngle)
{
	double theta = launchAngle*(3.14159265) / 180;

	double d1 = this->d1;
	double d2 = this->d2;
	double d3 = this->d3;
	double y0 = d1 + d2*sin(theta) + d3*cos(theta);
	return y0;
}

/*
* Calculates the initial velocity components of the pingpong ball
* Outputs: v0x and v0y respectively [m/s]
*/
double Cannon::initialVelocityX(double launchAngle)
{
	double theta = launchAngle*(3.14159265) / 180;
	double v0x = this->v0 * cos(theta);
	return v0x;
}
double Cannon::initialVelocityY(double launchAngle)
{
	double theta = launchAngle*(3.14159265) / 180;
	double v0y = this->v0 * sin(theta);
	return v0y;
}

/*
* Calculates the time at which the ball's centroid is ground level
* Outputs: Landing Time [sec] t_land
*/
double Cannon::t_land(double v0y, double y0)
{
	const double g = -9.818216325797522;
	double a = g / 2;
	double b = v0y;
	double c = y0;
	double t = (-b - sqrt(pow(b,2) - 4 * a*c)) / (2 * a);
	return t;
}

/*
* Calculates horizontal distance travelled before reaching ground level
* Outputs: horizontal distance, x_land [m]
*/
double Cannon::x_land(double v0x, double x0, double t_land)
{
	double k = 1.421000056204362;
	double x = x0 + (v0x / k) * (1 - exp(-k*t_land));
	return x;
}


/*
* Final Function [[LANDING DISTANCE]]
* Combines all members of Landing Distance member function
*/
double Cannon::landingDistance(double launchAngle)
{


	double x0 = initialX(launchAngle);
	double y0 = initialY(launchAngle);
	double v0x = initialVelocityX(launchAngle);
	double v0y = initialVelocityY(launchAngle);
	double t = t_land(v0y, y0);
	double x = x_land(v0x, x0, t);
	return x;
}



/*
*				Launch Angle member function:
*-----------------------------------------------------------
*/

/* 
* Finds the corresponding thetaL for a given thetaS [deg]
*/
double Cannon::servoAngle(double launchAngle)
{
	
	double a = 0.034915161132756;
	double b = 0.015053057998110 * 1000;
	double y_critical = 49.89238602;
	double x_critical = 54.03235397231884;
	double thetaS = 1000 / b*asin((launchAngle / 1000 - y_critical/1000) / a) + x_critical;

	return thetaS;
}

/*
* Finds the necessary launch angle to acheive a target distance.
* Inputs: ([deg], [deg], [m])
*/
double Cannon::getServoAngle(double angleLowerBound, double angleUpperBound, double target) {
	double maxError = 0.001; // [m]
	double midVal = (angleLowerBound + angleUpperBound) / 2 ;	// [deg]
	double launchAngle;											//x
	double position = landingDistance(midVal);				//yHat

	if(fabs(target - position) <= maxError){
		launchAngle = midVal;
	}
	else {
		if (position > target) {
			launchAngle = searchServo(midVal, angleUpperBound, target);
		}
		else if (position < target) {
			launchAngle = searchServo(angleLowerBound, midVal, target);
		}
		
	}
	return launchAngle;

}


/*
* Returns Cannon to Reloading Station and Position, and Releases Ball into Cannon.
*/
int Cannon::reload(Servo &servo_aim,Servo &servo_loader) {

	digitalWrite(MotorDirectionPin,!LEFT);
	analogWrite(MotorPowerPin, fullPower);

	while (!digitalRead(bumperR));			// Wait until the Cannon reaches the end of the platform.
	delay(10);
	while (!digitalRead(bumperR));
	Serial.println("Reload Break");
	delay(25);								// Apply the break.
	digitalWrite(MotorDirectionPin, LEFT);
	analogWrite(MotorPowerPin, fullPower);
	delay(50);
	analogWrite(MotorPowerPin, 0);

	servo_aim.write(60);					// Reload the Cannon.
	delay(200);
	servo_loader.write(0);
	delay(500);
	servo_loader.write(27);
	delay(550);
	servo_loader.write(20);
	delay(90);
	return 0;
}

/*
* Moves the Cannon to the desired Z Coordinate
*/

int Cannon::moveTo(int zCoordinate) {
	if (zCoordinate <= 0)
		return 0;

	const bool white = 1;
	const bool black = 0;
	bool state = 1;
	int position = 0;
	int thresh = 374;
	int motorPower = 200;

	digitalWrite(MotorDirectionPin, LEFT);
	analogWrite(MotorPowerPin, motorPower);
	//Count the black strips until at desired location.
	while (position < zCoordinate) {

		delay(10);
		if ((analogRead(IR) > thresh) && (state == white)) {
			state = black;
			if (LEFT)
				position++;
			else
				position--;
			Serial.print(position);
			Serial.print(" |         |- -> ");
			Serial.println(position);
			
			

		}

		if ((analogRead(IR) < thresh) && (state == black)) {
			state = white;
			if (LEFT)
				position++;
			else
				position--;
			Serial.print(position);
			Serial.print(" |||||||||||- -> ");
			Serial.println(position);
		}
		if (digitalRead(bumperL) == HIGH) {
			
			Serial.println("Left Breaking");
			break;
		}
		
		
	}
	// Apply the break.
	delay(25);
	digitalWrite(MotorDirectionPin, !LEFT);
	analogWrite(MotorPowerPin, fullPower);
	delay(50);
	analogWrite(MotorPowerPin, 0);
	return 0;
}