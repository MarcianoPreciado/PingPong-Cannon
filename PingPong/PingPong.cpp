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
*	5  *~		Motor Power		MotorPowerPin
*	6  *~		Solenoid Power		SolenoidPowerPin
*	7			Solenoid Direction	SolenoidDirectionPin
*	8
*	9  *~		Aiming Servo		cannonServo		(Servo Object)
*	10 *~		Loader Servo		loaderServo		(Servo Object)
*	11 *~		Right Bumper		bumperR
*	12			Left Bumper			bumperL
*	13			IR LED				LED
*
*	A0
*	A1
*	A2
*	A3
*	A4
*	A5		IR Sensor				IR
*	A6
*	A7		Built-In Buttons		btn
*/

#include "PingPong.h" // This is the Library created for this assignment
#include <math.h>
#include "Arduino.h"
#include "Servo.h" 

/*
* Cannon Constructor:
* Initializes Cannon Object
*/

Cannon::Cannon(void)
{

}


/*
* Final Function [[LANDING DISTANCE]]
* The most accurate function within reason to model our
* launch angle to distance data
*
* There are two versions. They are designed so that we can acheive a deep
* dive angle into the targets at all available distances, this requires two different
* functions for two different solenoid powers.
*/

double Cannon::landingDistanceIdealHIGH(double launchAngle)
{
	// Coefficients to a polynomial model which represents
	// a more accurate projectile effected by air resistance
	double c1 = -0.0000015953768248;
	double c2 = -0.0000846188736932;
	double c3 = 0.0011130901399068;
	double c4 = -4.8775853991479501;
	double c5 = 1.6279903018639805;

	double x = launchAngle;
	return c1*(pow(x, 3) - c4) + c2*(pow(x, 2) - c4) + c3*(x - c4) + c5;		// 5th degree poynomial representation of launchAngle -> landing distance
}

double Cannon::landingDistanceIdealLOW(double launchAngle)
{
	// Coefficients to a polynomial model which represents
	// a more accurate projectile effected by air resistance
	double c1 = -0.0000017643496539;
	double c2 = -0.0000579257365502;
	double c3 = -0.0002178814249680;
	double c4 = -216.6004993511213600;
	double c5 = 1.5845729636838506;
	double x = launchAngle;
	return c1*(pow(x, 3) - c4) + c2*(pow(x, 2) - c4) + c3*(x - c4) + c5;		// 5th degree poynomial representation of launchAngle -> landing distance
}

/*
*				Launch Angle member function:
*-----------------------------------------------------------
*/

/*
* Finds the corresponding thetaL for a given thetaS [deg]
* Uses an inverse sinusoidal model of the taken data.
*/

double Cannon::servoAngle(double launchAngle)
{
	double a = 0.034766074461727;					// <---Coefficients to an inverse sinusoidal model which 
	double b = 14.540019926536459;					// represents a thetaL vs. thetaS function
	double d = 0.050182708411012;			//the critical values represent the coordinates of the critical point of f(thetaS) = A*sin(B*thetaS+C)+D
	double x_critical = 55.439655033345026;			// a = A/1000, b = B/1000, y_critical = D, x_critical was found using fzero of A*sin(B*thetaS+C)
	double thetaS = (1000 / b) * asin((launchAngle / 1000 - d) / a) + x_critical;
	return thetaS;
}

/*
* Finds the necessary launch angle to acheive a target distance using recursive binary searching.
* Inputs: ([deg], [deg], [m])
*/

double Cannon::getLaunchAngle(double angleLowerBound, double angleUpperBound, double target)
{
	double maxError = 0.001;									// [m]
	double midVal = (angleLowerBound + angleUpperBound) / 2;	// [deg]
	double launchAngle;											
	double guess;
	// Choosing correct trajectory function
	if (target <= 0.94) {
		guess = landingDistanceIdealLOW(midVal);
	}
	else {
		guess = landingDistanceIdealHIGH(midVal);
	}
	// If the guess is within acceptable error, return the guess.
	if (fabs(target - guess) <= maxError)
	{
		launchAngle = midVal;
	}
	// Otherwise, continue binary search, and modify the boundaries.
	else
	{
		if (guess > target)
		{
			launchAngle = getLaunchAngle(midVal, angleUpperBound, target);
		}
		else if (guess < target)
		{
			launchAngle = getLaunchAngle(angleLowerBound, midVal, target);
		}
	}
	//Serial.println(launchAngle);
	return launchAngle;
}

/*
* Returns Cannon to Reloading Station and Position, and Releases Ball into Cannon.
*/

int Cannon::reload(Servo &servo_aim, Servo &servo_loader, bool &lastState, int &pos)
{
	servo_aim.write(35);
	digitalWrite(MotorDirectionPin, !LEFT);			// Sets direction of cannon to RIGHT
	analogWrite(MotorPowerPin, fullPower);			// Moves cannon right
	Serial.println("Moving to reload");
	while (!digitalRead(bumperR));					// Does nothing until the right bumper is pressed
	//Serial.println("Reload Break");
	delay(10);
	while (!digitalRead(bumperR));					// (Debounce)			
	Serial.println("Reload Break");
	delay(25);										// Apply the break:
	digitalWrite(MotorDirectionPin, LEFT);			// Changes direction to LEFT
	analogWrite(MotorPowerPin, fullPower);			// Moves cannon slightly left to unpress button
	delay(50);
	analogWrite(MotorPowerPin, 0);
	pos = 38;
	lastState = (analogRead(IR) > 500);
	// Cannon Reload Sequence and return 0
	delay(90);
	servo_loader.write(0);
	delay(500);
	servo_loader.write(27);
	delay(550);
	servo_loader.write(20);
	delay(50);
	return 0;
}

/*
* Moves the Cannon to the desired Z Coordinate
*/

int Cannon::moveTo(int zCoordinate, bool &lastState, int &pos, bool stop)
{
	const bool white = 0;
	const bool black = 1;
	bool state = (analogRead(IR) > 500);
	int thresh = 374;
	int motorPower = 255;
	bool DIR;
	if (zCoordinate < pos)
	{
		digitalWrite(MotorDirectionPin, LEFT);
		DIR = LEFT;
	}
	if (zCoordinate > pos)
	{
		digitalWrite(MotorDirectionPin, !LEFT);
		DIR = !LEFT;
	}
	analogWrite(MotorPowerPin, motorPower);

	//Count the black stripes until at desired location.
	while (pos != zCoordinate)
	{
		delay(10);
		if ((analogRead(IR) > thresh) && (state == white) && (lastState == state))
		{
			state = black;
			if (DIR)
				pos--;
			else
				pos++;
			Serial.print("|||||||||||- -> ");
			Serial.println(pos);
			delay(1);
		}

		if ((analogRead(IR) < thresh) && (state == black) && (lastState == state))
		{
			state = white;
			if (DIR)
				pos--;
			else
				pos++;
			Serial.print("|         | ");
			Serial.println(pos);
			delay(1);
		}
		if ((digitalRead(bumperL) == HIGH) && (DIR))
		{
			Serial.println("Left Breaking");
			pos = 0;
			break;
		}
		if ((digitalRead(bumperR) == HIGH) && !DIR)
		{
			Serial.println("Right Breaking");
			pos = 38;
			break;
		}
		lastState = state;
	}

	// Apply the break if stop was selected.
	if (stop) {
		if (state == white)
		{
			delay(270);
		}
		delay(85);
		digitalWrite(MotorDirectionPin, !DIR);
		analogWrite(MotorPowerPin, fullPower);
		delay(50);
	}
	analogWrite(MotorPowerPin, 0);
	lastState = state;

	return 0;
}

/*
* Moves the Cannon to the home position
* Inputs( reference to the last color bar recorded, reference to last position recorded)
*/

int Cannon::returnHome(bool &lastState, int &pos)
{
	Serial.println("Going Home");
	digitalWrite(MotorDirectionPin, LEFT);		// Changes direction to left
	analogWrite(MotorPowerPin, fullPower);		// Moves cannon left
	while (!digitalRead(bumperL));			// Wait until the bumper is being pressed
	Serial.println("Breaking (AT HOME)");
	analogWrite(MotorPowerPin, 0);			// Stops cannon
	delay(5);
	digitalWrite(MotorDirectionPin, !LEFT);		// Changes direction to right
	analogWrite(MotorPowerPin, fullPower);		// Moves cannon slightly right to unpress botton
	delay(50);
	analogWrite(MotorPowerPin, 0);			// Stops cannon
	pos = 0;
	lastState = (analogRead(IR) > 500);		// Updates which color the LED senses
	return 0;
}


/*
* ---------------------------------------------------------------------------------
*	ARCHIVED PREVIOUS FUNCTIONS BASED ON THE DERIVED MECHATRONIC FUNCTIONS GIVEN
*							None are in use anymore
* ---------------------------------------------------------------------------------
*/

/*
*        Landing Distance member function:
*-----------------------------------------------------------
*
*/

/*
* Cannon Constructor:
* Initializes all permanent properties of Cannon Object
*/

/*
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
*/

/*
* Calculates the initial coordinates of the pingpong ball
* Outputs: x0 and y0 respectively [m]
*/

double Cannon::initialX(double launchAngle)
{
	double theta = launchAngle * (3.14159265) / 180;
	double x0 = ((this->d2) * cos(theta)) - ((this->d3) * sin(theta));
	return x0;
}

double Cannon::initialY(double launchAngle)
{
	double theta = launchAngle * (3.14159265) / 180;
	double d1 = this->d1;
	double d2 = this->d2;
	double d3 = this->d3;
	double y0 = d1 + (d2 * sin(theta)) + (d3*cos(theta));
	return y0;
}

/*
* Calculates the initial velocity components of the pingpong ball
* Outputs: v0x and v0y respectively [m/s]
*/

double Cannon::initialVelocityX(double launchAngle)
{
	double theta = launchAngle * (3.14159265) / 180;
	double v0x = this->v0 * cos(theta);
	return v0x;
}

double Cannon::initialVelocityY(double launchAngle)
{
	double theta = launchAngle * (3.14159265) / 180;
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
	double t = (-b - sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);
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

