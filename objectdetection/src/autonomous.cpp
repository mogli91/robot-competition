/*
 * AUTONOMOUS.CPP
 *
 */
/*

#include "autonomous.h"

int l_weight_IR[IR_SENSORS] = { 3, 5, -5, -3, 0, 1,-1}; //for the power of 2 law
int r_weight_IR[IR_SENSORS] = { -2, -4, 4, 3, 0, -1, 1}; //for the power of 2 law

Autonomous::Autonomous(Brain* brain) {
	m_robot = new Robot(brain);
	m_robot->updateData();
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = 0;

	m_timeInit = clock();
	m_bottlesCollected = 0;

	change_state(STATE_MOVE_TRAVEL);

	m_tInit = m_robot->getTime();
	m_wait = 0;
	m_waitInit = 0;
}

void Autonomous::change_state(int newState) {
	m_nextState.push(newState);
	m_currentState = m_nextState.pop();
}

void Autonomous::loop()
{
	m_robot->updateData();
	//update time
	m_time = m_robot->getTime() - m_tInit;

	if(m_time - m_waitInit > m_wait)
	{
		switch(m_currentState)
		{
			case(STATE_LIFT_UP):
				m_robot->setLift(VAL_LIFT_HIGH);
				wait(3000);
				break;

			case(STATE_LIFT_DOWN):
				m_robot->setLift(VAL_LIFT_LOW);
				wait(3000);
				break;

			case(STATE_LIFT_MED):
				m_robot->setLift(VAL_LIFT_TRAVEL);
				wait(2000);
				break;

			case(STATE_STOP):
				m_robot->setWheelSpeed(VAL_WHEELS_STOP, VAL_WHEELS_STOP);
				break;

			case(STATE_MOVE_TRAVEL):

			break;

			case(STATE_MOVE_HOME):

			break;

			case(STATE_MOVE_BOTTLE):

			break;

			case(STATE_AVOID_CAM):

			break;

			case(STATE_AVOID_IR):
				if(braitenbergDisable())
					change_state(STATE_MOVE);
				else
					braitenberg_avoidance();
				break;

			case(STATE_TAIL_DOWN):
				m_robot->setLift(VAL_TAIL_OPEN);
				wait(2000);
			break;

			case(STATE_TAIL_UP):
				m_robot->setLift(VAL_TAIL_CLOSE);
				wait(2000);
			break;
		}

	m_wait = 0;
	m_waitInit = 0;
	m_robot->sendInstructions();
	}


}

void Autonomous::wait(int msec)
{
	m_waitInit = m_time;
	m_wait = msec;
}

/////Simulation data

bool braitenbergDisable()
{
	if ((m_robot->getSensorValue(SENSOR_IR_FRONT_L)	> BRAITEN_THRESHOLD_DISABLE) &&
		(m_robot->getSensorValue(SENSOR_IR_FRONT_R) > BRAITEN_THRESHOLD_DISABLE) &&
		(m_robot->getSensorValue(SENSOR_IR_R) > BRAITEN_THRESHOLD_DISABLE) &&
		(m_robot->getSensorValue(SENSOR_IR_L) > BRAITEN_THRESHOLD_DISABLE) &&
		(m_robot->getSensorValue(SENSOR_IR_BACK_R) > BRAITEN_THRESHOLD_DISABLE) &&
		(m_robot->getSensorValue(SENSOR_IR_BACK_L) > BRAITEN_THRESHOLD_DISABLE)) {
			return true;
	}

	return false;
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

		msr += (80 - m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) *
			   (80 - m_robot->getSensorValue(sensor_ir + SENSOR_IR_L))
			   / 25	* r_weight_IR[sensor_ir]; // motor speed right
		msl += (80 - m_robot->getSensorValue(sensor_ir + SENSOR_IR_L)) *
			   (80 - m_robot->getSensorValue(sensor_ir + SENSOR_IR_L))
			   / 25	* l_weight_IR[sensor_ir]; // motor speed left

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

void Simulation::liftBottle() {
	change_state(STATE_STOP);
	change_state(STATE_LIFT_UP);
	change_state(STATE_LIFT_DOWN);
}

bool Simulation::bottleCaptured() {
	if (m_robot->getSensorValue(SENSOR_IR_BRUSH) < 20) {
		return true;
	}
	return false;
}

void Simulation::moveWithVector() {
	int wl, wr;

	//normalize the vector
	float norm = sqrt(m_displacementVector[Y]*m_displacementVector[Y] + m_displacementVector[X]*m_displacementVector[X]);
	m_displacementVector[Y] /= norm;
	m_displacementVector[X] /= norm;

	//condition for not idling
	if(fabs(m_displacementVector[Y]) < 0.05 && fabs(m_displacementVector[X]) < 0.05)
			m_displacementVector[X] += 0.5;

	wl = VAL_WHEELS_STOP + VAL_WHEELS_STOP*m_displacementVector[Y] + VAL_WHEELS_STOP*m_displacementVector[X];
	wr = VAL_WHEELS_STOP + VAL_WHEELS_STOP*m_displacementVector[Y] - VAL_WHEELS_STOP*m_displacementVector[X];

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
/*
	if(wr > VAL_WHEELS_FW)
	 {
	 wr = VAL_WHEELS_FW;
	 }
	 if(wr < VAL_WHEELS_BW)
	 {
	 wr = VAL_WHEELS_BW;
	 }*
	if (wl > VAL_WHEELS_FW) {
		wl = VAL_WHEELS_FW;
	}
	if (wl < VAL_WHEELS_BW) {
		wl = VAL_WHEELS_BW;
	}
	m_robot->setWheelSpeeds(wr, wl);
}
void Simulation::approachBottlesCam() {
	/
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
		//x from -160 to 160
		m_displacementVector[X] += ((float)bottles[selectedBottle].x)/160.0f;
		//m_displacementVector[Y] += 0.5;
	}
}
void Simulation::avoidObstaclesCam() {
	//TODO : real function

	//create a vector that is perpendicular to the obstacle
	m_displacementVector[X] = 2*m_vm.line.delta_y; // vy
	if(m_displacementVector[X] < 0.1 && m_displacementVector[X] >= 0)
		m_displacementVector[X] = 0.1;
	if(m_displacementVector[X] > -0.1 && m_displacementVector[X] < 0)
			m_displacementVector[X] = -0.1;
	m_displacementVector[Y] = ((float)(CAM_OBST-m_vm.line.intercept))/((float)CAM_OBST)*(-m_vm.line.delta_x); // vx

	std::cout<<"delta x "<<m_vm.line.delta_x<<" delta y "<<m_vm.line.delta_y<<std::cout;
	std::cout<<"disp x "<<m_displacementVector[X]<<" disp y "<<m_displacementVector[Y]<<std::cout;
}

//the displacement the robot should have if no collision is detected
void Simulation::displacement() {
	//for the moment just move forward;
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = 1.0f;
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
	}*
	float deltaAngle = (((float)destAngle) - ((float)angle))*PI/180.0f;
	m_displacementVector[X] += sin(deltaAngle);
	m_displacementVector[Y] += cos(deltaAngle);
		if(m_displacementVector[Y] < 0)
		{
			m_displacementVector[X] = 0.5;
			m_displacementVector[Y] = 0;
		}

}
//change direction of brush if overcurrent
bool Simulation::brushIsBlocked() {
	/*if (m_robot->getBrushCurrent() > 400) //300mA
	{
		m_robot->setBrushSpeed(VAL_BRUSH_BW);
		usleep(300000);
		m_robot->setBrushSpeed(VAL_BRUSH_FW);
		return true;
	}*
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
	if(m_vm.line.error < 200 && m_vm.line.intercept < CAM_OBST && m_vm.line.intercept > 50)
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
		m_currentState = STATE_AVOIDANCE;
	}
}
void Simulation::emptyTailGate() {
	m_robot->setTailGate(VAL_TAIL_OPEN);
	m_robot->setWheelSpeeds(VAL_WHEELS_STOP, VAL_WHEELS_STOP);
	m_robot->sendInstructions();
	sleep(2);
	m_robot->setTailGate(VAL_TAIL_CLOSE);
}

bool Simulation::homeReached() {
	//no beacon detected
	if(m_vm.beacon.y < 150 && m_vm.beacon.y != -1)
	{
		if(m_robot->getPose().angle > 180 && m_robot->getPose().angle < 270)
			return true;
	}

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

    return;

	m_robot->setBrushSpeed(VAL_BRUSH_FW);

	if (bottleCaptured() && m_currentState != STATE_AVOIDANCE) {
		liftBottle();
		m_bottlesCollected++;
	}

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
		m_robot->setShovel(VAL_LIFT_LOW);

		if (m_bottlesCollected >= 2 || elapsed_secs > 60 * 3) //8 minutes or 4 bottles = go home
		{
			goHome(); //go home using the compass
		} else {
			search(); //search for bottles in the arena
		}

		moveWithVector(); //move in the direction of the vector

		break;

	case STATE_AVOIDANCE:
		//while (state == STATE_AVOIDANCE) {
		if ((m_robot->getSensorValue(SENSOR_IR_FRONT_L)	> BRAITEN_THRESHOLD_DISABLE) &&
			(m_robot->getSensorValue(SENSOR_IR_FRONT_R) > BRAITEN_THRESHOLD_DISABLE) &&
			(m_robot->getSensorValue(SENSOR_IR_R) > BRAITEN_THRESHOLD_DISABLE) &&
			(m_robot->getSensorValue(SENSOR_IR_L) > BRAITEN_THRESHOLD_DISABLE) &&
			(m_robot->getSensorValue(SENSOR_IR_BACK_R) > BRAITEN_THRESHOLD_DISABLE) &&
			(m_robot->getSensorValue(SENSOR_IR_BACK_L) > BRAITEN_THRESHOLD_DISABLE)) {
				change_state(STATE_MOVE);
			break;
		} else {
			braitenberg_avoidance();
		}
		break;
	case STATE_CAM_AVOIDANCE:
		if(m_vm.line.intercept > CAM_OBST && m_vm.line.intercept < 50) //from approx the middle of the cam : 50 cm in front of robot.
		{
			change_state(STATE_MOVE);
			break;
		}
		else
			avoidObstaclesCam();
			m_robot->setShovel(VAL_LIFT_TRAVEL);
			//m_robot->setWheelSpeeds(VAL_WHEELS_STOP, VAL_WHEELS_STOP);
			m_robot->sendInstructions();
			sleep(1);
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
*/
