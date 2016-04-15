/*
* Created by: Marciano C. Preciado, Matt D. Ludlow
* Latest Update: 04-15-16
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
*	9  *~		Aiming Servo		cannonServo		(Servo Instance)
*	10 *~		Loader Servo		loaderServo	(Servo Instance)
*	11 *~		Right Bumper		bumperR
*	12			Left Bumper			bumperL
*	13			IR LED				LED
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

#pragma once
#ifndef PingPong_h
#define PingPong_h
#include "Servo.h"
#include "Arduino.h"

//----------DEFINITIONS------------//
#define MotorDirectionPin  4
#define MotorPowerPin 5

#define SolenoidDirectionPin  7
#define SolenoidPowerPin 6

#define shotDelay  5000
#define onTime  500
#define fullPower  255

#define LEFT 1
#define bumperL 12
#define bumperR 11

const int IR = A5;
//---------------------------------//


class Cannon {

public:

	// Preparations for cannon:
	Cannon(void); //Constructor

	// Function to find an accurate landing distance for a given launch angle:
	double landingDistanceIdealHIGH(double launchAngle);
	double landingDistanceIdealLOW(double launchAngle);

	// Function to find the Servo Angle for a given Launch Angle:
	double servoAngle(double launchAngle);

	// Finds necessary Servo Angle to Hit a Target:
	// Inputs: Minimum angle, Maximum angle, target in [m]
	double getLaunchAngle(double, double, double);

	// Performs the reloading action
	// Both inputs are Servo References
	int reload(Servo &servo_aim, Servo &servo_loader, bool &lastState, int &pos);

	// Moves the platform to the given coordinate
	int moveTo(int zCoordinate, bool &lastState, int &pos);

	//Moves the platform to the home position
	int returnHome(bool &lastState, int &pos);


	/*
	* ---------------------------------------------------------------------------------
	*	ARCHIVED PREVIOUS FUNCTIONS  BASED ON THE DERIVED MECHATRONIC FUNCTIONS GIVEN
	* ---------------------------------------------------------------------------------
	*/
	//Constructor
	//Cannon(double L1, double L2, double L3, double L4, double d1, double d2, double d3, double thetaS0, double thetaL0, double v0);

	// LandingDistance member function:
	double initialX(double launchAngle);
	double initialY(double launchAngle);
	double initialVelocityX(double launchAngle);
	double initialVelocityY(double launchAngle);
	double t_land(double v0y, double y0);
	double x_land(double v0x, double x0, double t_land);
	//double landingDistance(double launchAngle);


private:

	// Qualities of Cannon:
	double v0;		// Initial ball launch velocity [m/s]

	double L1;		// Linkage lengths [L1,L2,L3,L4] [m]
	double L2;
	double L3;
	double L4;

	double d1;		// Cannon d values [d1,d2,d3] [m]
	double d2;
	double d3;

	double thetaS0;	// Angle offset values [thetaS0,thetaL0] [deg]
	double thetaL0;

};
#endif 
