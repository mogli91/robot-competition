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
#define IR_SENSORS           8

//Define the number of sensors for bottle detection and approaching (US): Phototaxis
#define US_SENSORS         4

/*DEFINE THE THRESHOLDS FOR THE ACTIVATIONS OF THE LOCAL OBSTACLE AVOIDANCE (BRAITENBERG VEHICLES)*/
#define BRAITEN_THRESHOLD_ENABLE       55      //50 cm
#define BRAITEN_THRESHOLD_DISABLE      75    //HYSTERESIS

#define STATE_INIT                     0       //State at the beginning of the 10:00
#define STATE_MOVE                     1       //Follow the predetermined checkpoints
#define STATE_RETURN                   2       //If storage is full return to the recycling area
#define STATE_PICKUP                   3       //If bottle detected approach it and pick it up
#define STATE_AVOIDANCE                4       //If obstacle detected (BRAITEN_THRESHOLD)
                                               // avoid it
enum POS{X,Y};

class Simulation {
public:
	Simulation(Brain* brain);
    ~Simulation(){};

	void loop ();
	//Function for the obstacle avoidance
	void braitenberg_avoidance ();
	//Function for the phototaxis
//	void braitenberg_phototaxis ();
	//Method for the change of the robot's state
	void change_state(int newState);

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
