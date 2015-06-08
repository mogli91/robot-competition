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

	//never go backwards, turn instead.
	if(m_displacementVector[Y] < VAL_WHEELS_STOP)
	{
		//m_displacementVector[X] += m_displacementVector[Y];
		m_displacementVector[Y] = VAL_WHEELS_STOP;
	}
	wl = m_displacementVector[Y] + m_displacementVector[X];
	wr = m_displacementVector[Y] - m_displacementVector[X];
/*
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
	}*/

	if(wr > VAL_WHEELS_FW)
	 {
	 wr = VAL_WHEELS_FW;
	 }
	 if(wr < VAL_WHEELS_BW)
	 {
	 wr = VAL_WHEELS_BW;
	 }
	if (wl > VAL_WHEELS_FW) {
		wl = VAL_WHEELS_FW;
	}
	if (wl < VAL_WHEELS_BW) {
		wl = VAL_WHEELS_BW;
	}
	m_robot->setWheelSpeeds(wr, wl);
}
void Simulation::approachBottlesCam() {
	//TODO : real function
//	m_displacementVector[X] -= 150 * abs(m_robot->getBottleAngle());

	//if a bottle is detected
	std::vector<Point> bottles = m_vm.bottles;
	int selectedBottle = 0;
	if(bottles.size() > 0)
	{
		//chose closest bottle
		for(uint i = 0; i < bottles.size(); i++)
		{
			if(bottles[i].y < bottles[selectedBottle].y)
				selectedBottle = i;
		}
		m_displacementVector[X] += 50 * bottles[selectedBottle].x;
		m_displacementVector[Y] += 50;
	}
}
void Simulation::avoidObstaclesCam() {
	//TODO : real function

	//create a vector that is perpendicular to the obstacle
	m_displacementVector[X] = VAL_WHEELS_FW*m_vm.line.delta_y + 50; // vy
	m_displacementVector[Y] = -VAL_WHEELS_FW*m_vm.line.delta_x; // vx
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
	int angle = m_robot->getPose().angle;
	int destAngle = 225;
/*
	if(abs(angle - destAngle) > 40)
	{
		m_displacementVector[X] = VAL_WHEELS_FW*sign(destAngle - angle);
		m_displacementVector[Y] = VAL_WHEELS_STOP;
	}
	else
	{
		m_displacementVector[X] = 0;
		m_displacementVector[Y] = VAL_WHEELS_FW;
	}*/
	float deltaAngle = (((float)destAngle) - ((float)angle))*PI/180.0f;
	m_displacementVector[X] = +VAL_WHEELS_FW*sin(deltaAngle);
	m_displacementVector[Y] = VAL_WHEELS_FW*cos(deltaAngle);
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
int Simulation::emergencyDetected() {
	for (int i = SENSOR_IR_L; i <= SENSOR_IR_BACK; i++) {
		if (m_robot->getSensorValue(i) < BRAITEN_THRESHOLD_ENABLE) {
			return STATE_AVOIDANCE;
		}
	}

	//regression corresponds to a line, and no obstacle is detected
	//200 corresponds to a value regressed with rock angles
	if(m_vm.line.error < 200)
	{
		return STATE_CAM_AVOIDANCE;
	}

	return EMERGENCY_NONE;
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
	//no beacon detected
	if(m_vm.beacon.y != -1)
	{
		if(m_vm.beacon.y < 50)
		{
			return true;
		}
	}
	else
		return false;
}
void Simulation::search() {
	displacement(); //normal robot displacement if no collision detected
	approachBottlesCam(); //move towards bottles viewed with cam
}
// main function
void Simulation::loop(void) {

	m_robot->updateData();
	updateVision();

	int elapsed_secs = double(clock() - m_timeInit) / CLOCKS_PER_SEC;

	m_robot->setBrushSpeed(VAL_BRUSH_FW);

	// State machine
	switch (m_currentState) {
	case STATE_INIT:
		// Stuff to do at the beginning
		change_state( STATE_MOVE);
		break;

	case STATE_MOVE:
		int emergency;
		emergency = emergencyDetected();
		if (emergency != EMERGENCY_NONE) {
			change_state(emergency);
			break;
		}

		if (bottleCaptured()) {
			liftBottle();
			m_bottlesCollected++;
		}

		if (m_bottlesCollected >= 2 || elapsed_secs > 60 * 3) //8 minutes or 4 bottles = go home
		{
			goHome(); //go home using the compass
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
	case STATE_CAM_AVOIDANCE:
		if(m_vm.line.intercept > 100) //from approx the middle of the cam : 50 cm in front of robot.
		{
			change_state(STATE_MOVE);
			break;
		}
		else
			avoidObstaclesCam();
			moveWithVector();
		break;
	}

	m_robot->sendInstructions();
}

void Simulation::change_state(int newState) {
	//TODO add security
	m_currentState = newState;
}

void Simulation::updateVision()
{
	m_robot->getVisionData(m_vm);
}
