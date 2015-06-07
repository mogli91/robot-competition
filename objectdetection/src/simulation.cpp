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
int l_weight_IR[IR_SENSORS] = { 5, 8, -8, -5, 0 };     //{ 5, 8, -8, -5, 0 }
int r_weight_IR[IR_SENSORS] = { -4, -7, 7, 5, 0 };     //{ -5, -8, 8, 5, 0 };
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

Simulation::Simulation(Brain* brain)
{
	m_robot = new Robot(brain);
	change_state(STATE_INIT);
}

//Function for the obstacle avoidance
void Simulation::braitenberg_avoidance() {

	int sensor_ir;
	int msl = 400;
	int msr = 400;

	for (sensor_ir = 0; sensor_ir < IR_SENSORS; sensor_ir++) { // read sensor values and
															   // calculate motor speeds
        msr += (80-m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) * r_weight_IR[sensor_ir]; // motor speed right
		msl += (80-m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) * l_weight_IR[sensor_ir]; // motor speed left

	}
	if (msl>VAL_WHEELS_FW )  {msl=VAL_WHEELS_FW ;}
	if (msl<VAL_WHEELS_BW)   {msl=VAL_WHEELS_BW;}
	if (msr>VAL_WHEELS_FW )  {msr=VAL_WHEELS_FW ;}
	if (msr<VAL_WHEELS_BW)   {msr=VAL_WHEELS_BW;}
    
	m_robot->setWheelSpeeds(msr,msl);
}

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

// main function
void Simulation::loop(void) {

	m_robot->updateData();


	// State machine
	switch (m_currentState) {
		case STATE_INIT:
			// Stuff to do at the beginning
			change_state(STATE_FOLLOWPATH);
			break;

		case STATE_FOLLOWPATH:
			for (int i = SENSOR_IR_L; i <= SENSOR_IR_BACK; i++)
			{
				if (m_robot->getSensorValue(i) < BRAITEN_THRESHOLD_ENABLE )
				//check if obstacle or bottle by comparing with us
				//if (obstacle)
				{
					change_state(STATE_AVOIDANCE);
					break;
				}
				else
				{
					//follow_path();
					//m_robot->setWheelSpeeds(500, 500);
					braitenberg_phototaxis();
				}
				/*else
				{
					change_state -----> STATE_PICKUP
					break;
				}*/

			}
			break;

		case STATE_AVOIDANCE:
			//while (state == STATE_AVOIDANCE) {
			if ((m_robot->getSensorValue(SENSOR_IR_FRONT_L) > BRAITEN_THRESHOLD_DISABLE )&&(m_robot->getSensorValue(SENSOR_IR_FRONT_R) > BRAITEN_THRESHOLD_DISABLE )&&(m_robot->getSensorValue(SENSOR_IR_R) > BRAITEN_THRESHOLD_DISABLE )&&(m_robot->getSensorValue(SENSOR_IR_L) > BRAITEN_THRESHOLD_DISABLE ))
				{
					change_state(STATE_FOLLOWPATH);
					break;
				}
				else
				{
					braitenberg_avoidance();
				}
			break;
	}

    m_robot->sendInstructions();
    
}

void Simulation::change_state(int newState)
{
	//TODO add security
	m_currentState = newState;
}
