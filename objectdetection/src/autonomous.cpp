/*
 * AUTONOMOUS.CPP
 *
 */


#include "autonomous.h"

Autonomous::Autonomous(Brain* brain) {
	m_robot = new Robot(brain);
	m_robot->updateData();

	m_timeInit = clock();
	m_bottlesCaptured = 0;

	changeState(STATE_MOVE_TRAVEL);

	m_tInit = m_robot->getTime();
	m_wait = 0;
	m_waitInit = 0;
}

void Autonomous::changeState(int newState) {
	m_nextState.push(newState);
	m_currentState = m_nextState.front();
	m_nextState.pop();
}

void Autonomous::loop()
{
	m_robot->updateData();
	updateVision();

	//update time
	m_time = m_robot->getTime() - m_tInit;

	if(bottleCaptured())
	{
		liftBottle();
		m_bottlesCaptured++;
	}


		switch(m_currentState)
		{
			case(STATE_INIT):
				changeState(STATE_LIFT_MED);
				wait(3000);
				changeState(STATE_MOVE_TRAVEL);
				break;

			case(STATE_LIFT_UP):
				m_robot->setShovel(VAL_LIFT_HIGH);
				break;

			case(STATE_LIFT_DOWN):
				m_robot->setShovel(VAL_LIFT_LOW);
				break;

			case(STATE_LIFT_MED):
				m_robot->setShovel(VAL_LIFT_TRAVEL);
				break;

			case(STATE_STOP):
				m_robot->setWheelSpeeds(VAL_WHEELS_STOP, VAL_WHEELS_STOP);
				break;

			case(STATE_MOVE_TRAVEL):
				if(stateChanged())
					break;
				else if(m_bottlesCaptured >= 3 || m_time*60/1000 > 8)
				{
					changeState(STATE_LIFT_MED);
					changeState(STATE_MOVE_HOME);
					break;
				}
				else
					displacement();
			break;

			case(STATE_MOVE_HOME):
				if(stateChanged())
					break;
				else
					goHome();
			break;

			case(STATE_MOVE_BOTTLE):
				if(braitenbergEnable())
				{
					changeState(STATE_LIFT_MED);
					wait(3000);
					changeState(STATE_AVOID_IR);
					break;
				}
				else
				{
					changeState(STATE_LIFT_DOWN);
					moveBottle();
				}
			break;

			//case(STATE_AVOID_CAM):

			//break;

			case(STATE_AVOID_IR):
				if(braitenbergDisable())
				{
					changeState(STATE_LIFT_MED);
					changeState(STATE_MOVE_TRAVEL);
				}
				else
					braitenberg_avoidance();
				break;

			case(STATE_TAIL_DOWN):
				m_robot->setTailGate(VAL_TAIL_OPEN);
			break;

			case(STATE_TAIL_UP):
				m_robot->setTailGate(VAL_TAIL_CLOSE);
			break;

			case(STATE_WAIT):
				if(m_time - m_waitInit > m_wait)
				{
					m_wait = 0;
					m_waitInit = 0;
					goToNextState();
				}

				break;
		}
	m_robot->sendInstructions();
}

bool Autonomous::bottleEnable()
{
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
			if(bottles[selectedBottle].y < BOTTLE_ENABLE)
				return true;
		}
		return false;
}

void Autonomous::goToNextState()
{
	if(m_nextState.size())
	{
		m_currentState = m_nextState.front();
		m_nextState.pop();
	}
	else m_currentState = STATE_MOVE_TRAVEL;
}

void Autonomous::wait(int msec)
{
	m_waitInit = m_time;
	m_wait = msec;
	changeState(STATE_WAIT);
}

/////Simulation data

//says yes if the state changed
bool Autonomous::stateChanged()
{
	if(braitenbergEnable())
	{
		changeState(STATE_LIFT_MED);
		wait(2000);
		changeState(STATE_AVOID_IR);
		return true;
	}
	else if(bottleEnable())
	{
		changeState(STATE_LIFT_DOWN);
		wait(3000);
		changeState(STATE_MOVE_BOTTLE);
		return true;
	}
	//else if()

	return false;
}

bool Autonomous::braitenbergEnable()
{
	for (int i = SENSOR_IR_L; i <= SENSOR_IR_BACK; i++) {
		if (m_robot->getSensorValue(i) < BRAITEN_THRESHOLD_ENABLE) {
			return true;
		}
	}
	return false;
}

bool Autonomous::braitenbergDisable()
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
void Autonomous::braitenberg_avoidance() {

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

void Autonomous::liftBottle() {
	changeState(STATE_STOP);
	changeState(STATE_LIFT_UP);
	wait(3000);
	changeState(STATE_LIFT_DOWN);
	wait(3000);
}

bool Autonomous::bottleCaptured() {
	if (m_robot->getSensorValue(SENSOR_IR_BRUSH) < 20) {
		return true;
	}
	return false;
}

float Autonomous::norm(float x, float y)
{
	return sqrt(x*x+y*y);
}

void Autonomous::moveBottle() {
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
		float x = ((float)bottles[selectedBottle].x)/160.0f;
		float y = (float)bottles[selectedBottle].y/240.0f;
		float normo = norm(x,y);
		x = x/normo*VAL_WHEELS_STOP + VAL_WHEELS_STOP;
		y = y/normo*VAL_WHEELS_STOP + VAL_WHEELS_STOP;
		m_robot->setWheelSpeeds(y - x, y + x);
	}
}
//the displacement the robot should have if no collision is detected
void Autonomous::displacement() {
	//for the moment just move forward;
	m_robot->setWheelSpeeds(VAL_WHEELS_FW, VAL_WHEELS_FW);
}

void Autonomous::homeDisplacement() {
	//TODO : move in direction of a certain absolute angle;
	int angle = m_robot->getPose().angle;
	int destAngle = 225;

	float deltaAngle = (((float)destAngle) - ((float)angle))*PI/180.0f;
	float x = sin(deltaAngle);
	float y = cos(deltaAngle);
	if(y < 0)
	{
		x = 0.5;
		y = 0;
	}
	x = x*VAL_WHEELS_STOP + VAL_WHEELS_STOP;
	y = y*VAL_WHEELS_STOP + VAL_WHEELS_STOP;
	m_robot->setWheelSpeeds(y - x, y + x);
}

void Autonomous::goHome() {
	homeDisplacement(); //normal robot displacement if no collision detected
	if (homeReached()) {
		// TODO rotate 180 degrees;
		emptyTailGate();
		m_bottlesCaptured = 0;
	}
}
void Autonomous::emptyTailGate() {
	changeState(STATE_STOP);
	changeState(VAL_TAIL_OPEN);
	wait(2000);
	changeState(VAL_TAIL_CLOSE);
	wait(1000);
	changeState(STATE_INIT);
}

bool Autonomous::homeReached() {
	//no beacon detected
	if(m_vm.beacon.y < 150 && m_vm.beacon.y != -1)
	{
		if(m_robot->getPose().angle > 180 && m_robot->getPose().angle < 270)
			return true;
	}

	return false;
}

void Autonomous::updateVision()
{
	m_robot->getVisionData(m_vm);
}

