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

//Define the number of sensors for obstacle avoidance (IR)
#define IR_SENSORS           9

//Define the number of sensors for bottle detection and approaching (US): Phototaxis
#define US_SENSORS         4

/*DEFINE THE THRESHOLDS FOR THE ACTIVATIONS OF THE LOCAL OBSTACLE AVOIDANCE (BRAITENBERG VEHICLES)*/
#define BRAITEN_THRESHOLD_ENABLE       55      //50 cm
#define BRAITEN_THRESHOLD_DISABLE      75    //HYSTERESIS

#define STATE_INIT                     0       //State at the beginning of the 10:00
#define STATE_FOLLOWPATH               1       //Follow the predetermined checkpoints
#define STATE_RETURN                   2       //If storage is full return to the recycling area
#define STATE_PICKUP                   3       //If bottle detected approach it and pick it up
#define STATE_AVOIDANCE                4       //If obstacle detected (BRAITEN_THRESHOLD)
                                               // avoid it


class Simulation {
private:
    vector<int> visionDistances;
public:
	Simulation(Brain* brain);
    ~Simulation(){};

	void loop ();
	//Function for the obstacle avoidance
	void braitenberg_avoidance ();
	//Function for the phototaxis
	void braitenberg_phototaxis ();
	//Method for the change of the robot's state
	void change_state(int newState);
	//mycurrentstate should be a private attribute of the class robot and all this simulation
	//file should be merged with the robot file otherwise we cannot access to the private attribute
	//from here.
	//Otherwise, if we want to keep these 2 files separated, we can pass to the method change_state
	//the pointer to the robot (myrobot) from which we can access to a private member

private:
	int m_currentState;
	Robot* m_robot;
	//Map* m_map;



};




#endif /* SIMULATION_H_ */
