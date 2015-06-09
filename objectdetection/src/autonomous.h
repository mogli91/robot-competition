/*
 * simulation.h
 *
 *  Created on: Jun 3, 2015
 *      Author: pasquale
 */

#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_
/*
#include "brain.h"
#include "Map.h"
#include "Robot.h"

#include <ctime>
#include <unistd.h>

//Define the number of sensors for obstacle avoidance (IR)
#define IR_SENSORS           8

enum STATES{	STATE_LIFT_UP,
	    		STATE_LIFT_DOWN,
				STATE_LIFT_MED,
				STATE_STOP,
				STATE_MOVE_TRAVEL,
				STATE_MOVE_HOME,
				STATE_MOVE_BOTTLE,
				STATE_AVOID_CAM,
				STATE_AVOID_IR,
				STATE_TAIL_DOWN,
				STATE_TAIL_UP}

#define CAM_OBST 100

enum POS{X,Y};

class Autonomous {
private:
    vector<int> visionDistances;
public:
	Autonomous(Brain* brain);
    ~Autonomous(){};

	void loop ();
	void change_state(int newState);

private:
	/*bool bottleCaptured();
	void moveWithVector();
	void approachBottlesCam();
	void avoidObstaclesCam();
	void avoidObstaclesIR();
	void displacement();
	void homeDisplacement();
	bool brushIsBlocked();
	void emergencyProcedure();
	int emergencyDetected();
	void goHome();
	void search();
	bool homeReached();
	void emptyTailGate();
	void liftBottle();
	void updateVision();
	float calculateError();
*
	int m_currentState;
	Robot* m_robot;
	//float m_displacementVector[2];
	clock_t m_timeInit;
	int m_bottlesCollected;
	VisionMeasure m_vm;
	int m_tInit;
	int m_time;
	int m_wait;
	int m_waitInit;
	std::queue<int> m_nextState;
};


*/
#endif  AUTONOMOUS_H_
/*#endif /* SIMULATION_H_ */
