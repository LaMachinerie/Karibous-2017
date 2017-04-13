#include <Arduino.h>
#include <math.h>
#include "Deplacement.h"


Deplacement::Deplacement(int dirRight,
	int dirLeft,
	int stepRight,
	int stepLeft,
	int mode,
	int entreAxe,
	int diametreRoue,
	int reduction)
{
	_pins[0] = dirRight;
	_pins[1] = dirLeft;
	_pins[2] = stepRight;
	_pins[3] = stepLeft;

	_mode = mode;
	setNbStep(NB_STEPPER_TURN);

	_minPulseWidth = 2;

	enableOutputs();
}

void Deplacement::enableOutputs()
{
	pinMode(_pins[0], OUTPUT);
    pinMode(_pins[1], OUTPUT);
	pinMode(_pins[2], OUTPUT);
    pinMode(_pins[3], OUTPUT);
}

void Deplacement::setNbStep(int nbStep)
{
	_nbStep = _mode*nbStep;
}

void Deplacement::setDirection()
{
	// Mise à jour de la direction de rotation
	digitalWrite(_pins[0],_dirMotorRight);
	digitalWrite(_pins[1],!_dirMotorLeft);
	delayMicroseconds(1200);
}

void Deplacement::Step()
{
	// Réalisation d'un pas
	digitalWrite(_pins[2],HIGH);
	digitalWrite(_pins[3],HIGH);
	delayMicroseconds(_minPulseWidth);
	digitalWrite(_pins[2],LOW);
	digitalWrite(_pins[3],LOW);
}

void Deplacement::speedToTime()
{
	//Equation 18
	_maxSpeedTime = 1000000/_speed; // temps en microsecond entre deux pas à vitesse maxi
}

void Deplacement::accelToTime()
{
	//------------------
	//DEPRECIATE
	//Temps complet de la phase d'acceleration avant d'atteindre la vitesse maxi
	/*
	long fullAccelTime = (_speed/_accel)*1000000;
	long nbStepAccel = 0;
	*/

	//Increment de temps entre deux pas dans la phase d'acceleration et de decceleration
	//_incAccTime = sqrt(1000.0/(_accel/_percentages[0]));
	//-------------------
}

//DEPRECIATE
void Deplacement::setMaxPercentageProfil(char percentage)
{
	if(percentage > 100)percentage=100;
	_percentages[1] = percentage;
	_percentages[0]=_percentages[2]=(100-_percentages[1])/2;
}

bool Deplacement::run()
{
	bool state;
	if (_currentStep<_targetStep)
	{
		if ((micros()-_lastTime) >=_P)
		{
			if(_currentStep<_profileSteps[0])
			{
				// Phase d'acceleration
				_m=-_R;
			}
			else if(_currentStep>=_profileSteps[0] && _currentStep<(_profileSteps[1]+_profileSteps[0]))
			{
				// Phase vitesse constante
				_m=0;
			}
			else if(_currentStep<_targetStep)
			{
				// Phase de decceleration
				_m=_R;
			}
			// Equation 22
			_q = _m*_P*_P;
			// Equation 22
			//p = p*(1 + q + 1.5*q*q)
			_Pa = _P*(1+_q+1.5*_q*_q);
			_lastTime = micros();
			Step();
			_currentStep=_currentStep+1;
		}
		state = true;
	}
	else
	{
		state = false;
	}
	return state;
}

void Deplacement::setMaxSpeed(unsigned long speed)
{
	_speed = speed;
	speedToTime();
	_stepTime = _maxSpeedTime;
}

void Deplacement::setAcceleration(unsigned long accel)
{
	_accel = accel;
	//accelToTime();
}

void Deplacement::turn(long angle)
{
	// angle en pas
	_targetStep=abs(angle);
	_currentStep=0;

	if (angle > 0)
	{
		_dirMotorLeft = 1;
		_dirMotorRight = 0;
	}
	else
	{
		_dirMotorLeft = 1;
		_dirMotorRight = 0;
	}
	setDirection();
	setProfil();
}

void Deplacement::go(long distance)
{
	// distance en pas

	_targetStep = abs(distance);
	_currentStep = 0;
	_currentStepTime = INIT_TIME;

	if (distance > 0)
	{
		_dirMotorLeft = 1;
		_dirMotorRight = 1;
	}
	else
	{
		_dirMotorLeft = 0;
		_dirMotorRight = 0;
	}
	setDirection();
	setProfil();
	computeSpeedAccel();
}

void Deplacement::setProfil()
{
	/*
	Depreciate Methode
	for(int i = 0;i<=2;i++)
	{
		_profileSteps[i]=(long(_percentages[i])*_targetStep)/100 ;
	}
	*/
	_profileSteps[0]=(pow(_speed,2)/(2*_accel)); //Equation 16
}

void Deplacement::computeSpeedAccel()
{
	// Equation 17
	_P1 = 1000000/sqrt(2*_accel);
	_Pa=_P1; // Deplacement initial
	// Equation 19
	_R=_accel/pow(1000000,2);
}

void Deplacement::turnGo(long angle, long distance)
{

}

void Deplacement::goTo(long X, long Y, long orientation)
{
	//sqrt((YA - YB)2 + (XB - XA)2)
	double distance;
	double angle;
	distance = sqrt(pow((Y-_YActu),2)+pow((X-_XActu),2));
	angle = atan2(Y-_YActu,X-_XActu);

}
