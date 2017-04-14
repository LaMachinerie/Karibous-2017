#ifndef Deplacement_h
#define Deplacement_h

#define FULL_STEP 1
#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHTH_STEP 8
#define SIXTEENTH_STEP 16
#define THIRTYTH_STEP 32

#define NB_STEPPER_TURN 200	//Nombre de pas pour un tour sur un moteur pas à pas classique. Peut être ajusté avec setNbStep(int nbStep)

class Deplacement
{
public:

	Deplacement(int dirRight,
		int dirLeft,
		int stepRight,
		int stepLeft,
		int mode,
		int entreAxe,
		int diametreRoue,
		int reduction);

	/**
	 * Fonction permettant de mettre à jour la position du robot en fonction de
	 * la vitesse, de l'acceleration et de la position demandée. A appeler le
	 * plus souvent possible et au moins une fois par pas.
	 * @return Vrai si la distance n'est pas encore atteinte. Faux si la distance est atteinte
	 */
	bool run();
	/**
	 * Définie la vitesse maximum de rotation du robot
	 * @param speed Vitesse de rotation en pas/s
	 */
	void setMaxSpeed(unsigned long speed);
	/**
	 * Définie la valeur de la pente d'acceleration et de déceleration des moteurs
	 * @param accel Acceleration en pas /s2
	 */
	void setAcceleration(unsigned long accel);
	/**
	 * Permet de faire bouger le robot selon la méthode "turn and go" : rotation puis déplacement
	 * @param rot  rotation en degrées
	 * @param dist distance en pas
	 */
	void turnGo(long angle, long distance);
	/**
	 * Permet de faire aller le robot en X,Y relatif avec une orientation
	 * @param X           Coordonnée X souhaitée
	 * @param Y           Coordonnée Y souhaitée
	 * @param orientation Orientation souhaitée
	 */
	void goTo(long X, long Y, long orientation);

	/**
	 * Définit le poucentage du profil de vitesse à vitesse maximum constante
	 * @param percentage Pourcentage du profil entre 0 et 100
	 */
	void setMaxPercentageProfil(char percentage);

	/**
	 * Fait tourner le robot autour du centre de l'essieu d'un nombre de pas.
	 * @param angle Angle en pas.
	 */
	void turn(long angle);

	void go(long distance);

	void setNbStep(int nbStep);

	//DEPRECIATE
	//void setProfil();


protected:

	void enableOutputs();
	/**
	 * Réalise un step en envoyant une impulsion sur les pattes de "step" des
	 * drivers. Le temps de l'impulsion est définie par la variable _minPulseWidth
	 */
	void Step();
	/**
	 * convertit la distance en mm en pas moteur en fonction des parametres du robot
	 * @param distance distance en mm
	 */
	void distanceToStep(long distance);

	void setDirection();

	void computeSpeedAccel();

	//DEPRECIATE
	//void speedToTime();
	//void accelToTime();


private:
	/**
	 * Temps minimum d'une impulsion en microsecond
	 */
	unsigned int _minPulseWidth;

	/**
	 * Pins pour le pilotage des drivers
	 * 0 : dirRight  1 : dirLeft  2 : stepRight  3 : stepLeft
	 */
	int _pins[4];

	unsigned long _lastTime;

	unsigned long _speed;

	unsigned long _accel;

	unsigned long _targetStep;
	unsigned long _currentStep;

	bool _dirMotorLeft;
	bool _dirMotorRight;

	long _XActu;
	long _YActu;
	long _orientationActu;

	unsigned long _currentStepTime;

	unsigned long _accelDistance;

	double _P1;
	double _R;
	double _m;
	double _Pa;
	double _q;

	/**
	 * Nombre de step par tour de moteur. Défini par le mode de pilotage et par
	 * le nombre max de step des moteurs pas à pas
	 */
	int _nbStep;

	int _mode;

	//DEPRECIATE
	//unsigned long _percentages[3];
	//unsigned long _profileSteps[3];
	//unsigned long _incAccTime;
	//unsigned int _stepTime;
	//unsigned int _accelTime;
	//unsigned int _maxSpeedTime;

};
#endif
