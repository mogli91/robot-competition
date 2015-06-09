/*
 * simulation.h
 *
 *  Created on: Jun 3, 2015
 *      Author: pasquale
 */

#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "brain.h"
#include "Map.h"
#include "Robot.h"

#include <ctime>
#include <unistd.h>
#include <queue>

//Define the number of sensors for obstacle avoidance (IR)
#define IR_SENSORS           8

enum STATES{	STATE_INIT,
				STATE_LIFT_UP,
	    		STATE_LIFT_DOWN,
				STATE_LIFT_MED,
				STATE_STOP,
				STATE_MOVE_TRAVEL,
				STATE_MOVE_HOME,
				STATE_MOVE_BOTTLE,
				STATE_AVOID_CAM,
				STATE_AVOID_IR,
				STATE_TAIL_DOWN,
				STATE_TAIL_UP,
				STATE_WAIT};

#define CAM_OBST 100
#define BOTTLE_ENABLE 100

/*DEFINE THE THRESHOLDS FOR THE ACTIVATIONS OF THE LOCAL OBSTACLE AVOIDANCE (BRAITENBERG VEHICLES)*/
#define BRAITEN_THRESHOLD_ENABLE       55      //50 cm
#define BRAITEN_THRESHOLD_DISABLE      75    //HYSTERESIS

class Autonomous {
private:
    vector<int> visionDistances;
public:
	Autonomous(Brain* brain);
    ~Autonomous(){};

	void loop ();
	void change_state(int newState);

private:
	void changeState(int newState);
	void goToNextState();
	void wait(int msec);
	bool stateChanged();
	bool braitenbergEnable();
	bool braitenbergDisable();
	void braitenberg_avoidance();
	void liftBottle();
	bool bottleCaptured();
	float norm(float x, float y);
	void moveBottle();
	void displacement();
	void homeDisplacement();
	void goHome();
	void emptyTailGate();
	bool homeReached();
	void updateVision();
	bool bottleEnable();

	int l_weight_IR[IR_SENSORS] = { 3, 5, -5, -3, 0, 1,-1}; //for the power of 2 law
	int r_weight_IR[IR_SENSORS] = { -2, -4, 4, 3, 0, -1, 1}; //for the power of 2 law

	int m_currentState;
	Robot* m_robot;
	//float m_displacementVector[2];
	clock_t m_timeInit;
	int m_bottlesCaptured;
	VisionMeasure m_vm;
	int m_tInit;
	int m_time;
	int m_wait;
	int m_waitInit;
	std::queue<int> m_nextState;
};



#endif
