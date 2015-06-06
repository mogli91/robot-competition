/*
 * simulation.h
 *
 *  Created on: Jun 3, 2015
 *      Author: pasquale
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "brain.h"
#include "Map.h"
#include "Robot.h"

#include <ctime>
#include <unistd.h>

//Define the number of sensors for obstacle avoidance (IR)
#define IR_SENSORS           5

//Define the number of sensors for bottle detection and approaching (US): Phototaxis
#define US_SENSORS         4

enum POS{X,Y};

class Simulation {
public:
	Simulation(Brain* brain);
    ~Simulation(){};

	void loop ();

private:

	bool bottleCaptured();
	void moveWithVector();
	void approachBottlesCam();
	void avoidObstaclesCam();
	void avoidObstaclesIR();
	void displacement();
	void homeDisplacement();
	bool brushIsBlocked();
	void emergencyProcedure();
	bool emergencyDetected();
	void goHome();
	void search();
	bool homeReached();
	void emptyTailGate();
	void liftBottle();


	int m_currentState;
	Robot* m_robot;
	int m_displacementVector[2];
	clock_t m_timeInit;
	int m_bottlesCollected;

};




#endif /* SIMULATION_H_ */
