/*
 * simulation.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: pasquale
 */

#include "simulation.h"

/*         1             2
 _IR___________IR_
 |                 |                    Disposition of the IR sensors on the robot and
 0   IR                 IR  3                corresponding ID
 |                 |
 |                 |
 |________IR_______|
 4
 */
//Obstacle avoidance
//int l_weight_IR[IR_SENSORS] = { 5, 8, -8, -5, 0 };     //{ 5, 8, -8, -5, 0 }
//int r_weight_IR[IR_SENSORS] = { -4, -7, 7, 5, 0 };     //{ -5, -8, 8, 5, 0 };
int l_weight_IR[IR_SENSORS] = { 3, 5, -5, -3, 0, 1,-1}; //for the power of 2 law
int r_weight_IR[IR_SENSORS] = { -2, -4, 4, 3, 0, -1, 1}; //for the power of 2 law
/*
 1                2
 US______________US
 |              |                    Disposition of the US sensors on the robot and
 0  US              US  3                corresponding ID
 |              |
 |              |
 |______________|
 */
//Phototaxis
//int l_weight_US[US_SENSORS] = { -10, -5, 5, 10 };
//int r_weight_US[US_SENSORS] = { 10, 5, -5, -10 };
int l_weight_US[US_SENSORS] = { 0, 0, -5, 5 };
int r_weight_US[US_SENSORS] = { 0, 0, 5, -5 };

Simulation::Simulation(Brain* brain) {
	m_robot = new Robot(brain);
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = 0;

	m_timeInit = clock();
	m_bottlesCollected = 0;

	change_state( STATE_INIT);
}

//Function for the obstacle avoidance
void Simulation::braitenberg_avoidance() {

	int sensor_ir;
	int msl = 400;
	int msr = 400;

	for (sensor_ir = 0; sensor_ir < IR_SENSORS; sensor_ir++) { // read sensor values and
		// calculate motor speeds
		//        msr += (80-m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) * r_weight_IR[sensor_ir]; // motor speed right
		//		msl += (80-m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) * l_weight_IR[sensor_ir]; // motor speed left

		msr += (80 - m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) * (80
				- m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) / 25
				* r_weight_IR[sensor_ir]; // motor speed right
		msl += (80 - m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) * (80
				- m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) / 25
				* l_weight_IR[sensor_ir]; // motor speed left

	}
	if (msl > VAL_WHEELS_FW) {
		msl = VAL_WHEELS_FW;
	}
	if (msl < VAL_WHEELS_BW) {
		msl = VAL_WHEELS_BW;
	}
	if (msr > VAL_WHEELS_FW) {
		msr = VAL_WHEELS_FW;
	}
	if (msr < VAL_WHEELS_BW) {
		msr = VAL_WHEELS_BW;
	}

	m_robot->setWheelSpeeds(msr, msl);
}
/*
 //Function for the phototaxis
 void Simulation::braitenberg_phototaxis() {

 int sensor_us;
 int msl = 370;
 int msr = 370;

 for (sensor_us = 0; sensor_us < US_SENSORS; sensor_us++) {

 msr += (80-m_robot->getSensorValue(sensor_us + SENSOR_IR_BOTTOM_L)) * r_weight_US[sensor_us]; // motor speed right
 msl += (80-m_robot->getSensorValue(sensor_us + SENSOR_IR_BOTTOM_L)) * l_weight_US[sensor_us]; // motor speed left
 }
 if (msl>VAL_WHEELS_FW )  {msl=VAL_WHEELS_FW ;}
 if (msl<VAL_WHEELS_BW)   {msl=VAL_WHEELS_BW;}
 if (msr>VAL_WHEELS_FW )  {msr=VAL_WHEELS_FW ;}
 if (msr<VAL_WHEELS_BW)   {msr=VAL_WHEELS_BW;}

 m_robot->setWheelSpeeds(msl,msr);
 }
 */

void Simulation::liftBottle() {
	m_robot->setShovel(VAL_LIFT_HIGH);
	m_robot->setWheelSpeeds(VAL_WHEELS_STOP, VAL_WHEELS_STOP);
	m_robot->sendInstructions();
	sleep(3);
	m_robot->setShovel(VAL_LIFT_LOW);
	m_robot->sendInstructions();
	sleep(3);
}
bool Simulation::bottleCaptured() {
	//if(brushIsBlocked() || m_robot->getBrushCurrent() > 220)
	if (m_robot->getSensorValue(SENSOR_IR_BRUSH) < 20) {
		//usleep(500000);
		return true;
	}
	return false;
}
void Simulation::moveWithVector() {
	int wl, wr; //wheel speeds left and right
	wl = m_displacementVector[Y] + m_displacementVector[X];
	wr = m_displacementVector[Y] - m_displacementVector[X];
	if (wl > VAL_WHEELS_FW) {
		wr -= wl - VAL_WHEELS_FW;
	}
	if (wl < VAL_WHEELS_BW) {
		wr += VAL_WHEELS_BW - wl;
	}
	if (wr > VAL_WHEELS_FW) {
		wl -= wl - VAL_WHEELS_FW;
		wr = VAL_WHEELS_FW;
	}
	if (wr < VAL_WHEELS_BW) {
		wl += VAL_WHEELS_BW - wl;
		wr = VAL_WHEELS_BW;
	}
	/*if(wr > VAL_WHEELS_FW)
	 {
	 wr = VAL_WHEELS_FW;
	 }
	 if(wr < VAL_WHEELS_BW)
	 {
	 wr = VAL_WHEELS_BW;
	 }*/
	if (wl > VAL_WHEELS_FW) {
		wl = VAL_WHEELS_FW;
	}
	if (wl < VAL_WHEELS_BW) {
		wl = VAL_WHEELS_BW;
	}
	m_robot->setWheelSpeeds(wl, wr);
}
void Simulation::approachBottlesCam() {
	//TODO : real function
//	m_displacementVector[X] -= 150 * abs(m_robot->getBottleAngle());
}
void Simulation::avoidObstaclesCam() {
	//TODO : real function
}

//the displacement the robot should have if no collision is detected
void Simulation::displacement() {
	//for the moment just move forward;
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = VAL_WHEELS_FW;
	//TODO : real displacement
}
void Simulation::homeDisplacement() {
	//TODO : move in direction of a certain absolute angle;
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = 0;
}
//change direction of brush if overcurrent
bool Simulation::brushIsBlocked() {
	/*if (m_robot->getBrushCurrent() > 400) //300mA
	{
		m_robot->setBrushSpeed(VAL_BRUSH_BW);
		usleep(300000);
		m_robot->setBrushSpeed(VAL_BRUSH_FW);
		return true;
	}*/
	return false;
}
void Simulation::emergencyProcedure() {
	//TODO : add the procedure
}
//detect an emergency : way too close to an obstacle or perhaps the lift is blocked...
bool Simulation::emergencyDetected() {
	//TODO : add the condition
	return false;
}
void Simulation::goHome() {
	homeDisplacement(); //normal robot displacement if no collision detected
	if (homeReached()) {
		// TODO rotate 180 degrees;
		emptyTailGate();
		m_bottlesCollected = 0;
	}
}
void Simulation::emptyTailGate() {
	m_robot->setTailGate(VAL_TAIL_OPEN);
	usleep(2000000);
	m_robot->setTailGate(VAL_TAIL_CLOSE);
}
bool Simulation::homeReached() {
	//TODO add the condition
	return false;
}
void Simulation::search() {
	displacement(); //normal robot displacement if no collision detected
	approachBottlesCam(); //move towards bottles viewed with cam
}
// main function
void Simulation::loop(void) {

	m_robot->updateData();

	int elapsed_secs = double(clock() - m_timeInit) / CLOCKS_PER_SEC;

	// State machine
	switch (m_currentState) {
	case STATE_INIT:
		// Stuff to do at the beginning
		m_robot->setBrushSpeed(VAL_BRUSH_FW);

		change_state( STATE_MOVE);
		break;

	case STATE_MOVE:
		m_robot->setBrushSpeed(VAL_BRUSH_FW);
		if (emergencyDetected()) {
			emergencyProcedure();
		}

		if (bottleCaptured()) {
			liftBottle();
			m_bottlesCollected++;
		}

		for (int i = SENSOR_IR_L; i <= SENSOR_IR_BACK; i++) {
			if (m_robot->getSensorValue(i) < BRAITEN_THRESHOLD_ENABLE) {
				change_state( STATE_AVOIDANCE);
				break;
			}
		}

		if (m_bottlesCollected >= 4 || elapsed_secs > 60 * 8) //8 minutes or 4 bottles = go home
		{
			//goHome(); //go home using the compass
			//TODO STATE_GO_HOME
		} else {
			search(); //search for bottles in the arena
		}

		moveWithVector(); //move in the direction of the vector

		break;

	case STATE_AVOIDANCE:
		//while (state == STATE_AVOIDANCE) {
		if ((m_robot->getSensorValue(SENSOR_IR_FRONT_L)
				> BRAITEN_THRESHOLD_DISABLE) && (m_robot->getSensorValue(
				SENSOR_IR_FRONT_R) > BRAITEN_THRESHOLD_DISABLE)
				&& (m_robot->getSensorValue(SENSOR_IR_R)
						> BRAITEN_THRESHOLD_DISABLE)
				&& (m_robot->getSensorValue(SENSOR_IR_L)
						> BRAITEN_THRESHOLD_DISABLE)) {
			change_state(STATE_MOVE);
			break;
		} else {
			braitenberg_avoidance();
		}
		break;
	}

	m_robot->sendInstructions();

}

void Simulation::change_state(int newState) {
	//TODO add security
	m_currentState = newState;
}
