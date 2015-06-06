/*
 * simulation.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: Eric
 */

#include "simulation.h"

Simulation::Simulation(Brain* brain)
{
	m_robot = new Robot(brain);
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = 0;

	m_timeInit = clock();
	m_bottlesCollected = 0;
}

// main function
void Simulation::loop(void)
{
	//update the sensor readings
	m_robot->updateData();

	int elapsed_secs = double(clock() - m_timeInit) / CLOCKS_PER_SEC;

	//if a problem is detected, do a certain routine
	if(emergencyDetected())
		emergencyProcedure();
	else
	{
		m_robot->setBrushSpeed(VAL_BRUSH_FW);
		if(bottleCaptured())
		{
			m_bottlesCollected++;
			m_robot->setShovel(VAL_LIFT_HIGH);
			sleep(5);
			m_robot->setShovel(VAL_LIFT_LOW);
		}

		/*if(m_bottlesCollected >= 4 || elapsed_secs > 60*8) //8 minutes or 4 bottles = go home
		{
			goHome();				//go home using the compass
		}
		else
		{
			search();				//search for bottles in the arena
		}*/

		//moveWithVector();			//move in the direction of the vector
	}

	//send instructions to the modules
    m_robot->sendInstructions();
    
}

bool Simulation::bottleCaptured()
{
	if(brushIsBlocked() || m_robot->getBrushCurrent() > 160)
	{
		usleep(500000);
		return true;
	}
	return false;
}

void Simulation::moveWithVector()
{
	int wl, wr; //wheel speeds left and right
	wl = m_displacementVector[Y] + m_displacementVector[X];
	wr = m_displacementVector[Y] - m_displacementVector[X];

	if(wl > VAL_WHEELS_FW)
	{
		wr -= wl-VAL_WHEELS_FW;
	}
	if(wl < VAL_WHEELS_BW)
	{
		wr += VAL_WHEELS_BW - wl;
	}

	if(wr > VAL_WHEELS_FW)
	{
		wl -= wl-VAL_WHEELS_FW;
		wr = VAL_WHEELS_FW;
	}
	if(wr < VAL_WHEELS_BW)
	{
		wl += VAL_WHEELS_BW - wl;
		wr = VAL_WHEELS_BW;
	}

	if(wl > VAL_WHEELS_FW)
	{
		wl = VAL_WHEELS_FW;
	}
	if(wl < VAL_WHEELS_BW)
	{
		wl = VAL_WHEELS_BW;
	}

	m_robot->setWheelSpeeds(wl, wr);
}

void Simulation::approachBottlesCam()
{
	//TODO : real function
}

void Simulation::avoidObstaclesCam()
{
	//TODO : real function
}


void Simulation::avoidObstaclesIR()
{
	int maxIRrange = 80;

	m_displacementVector[X] += (maxIRrange - m_robot->getSensorValue(SENSOR_IR_L))*7/10;
	m_displacementVector[Y] -= (maxIRrange - m_robot->getSensorValue(SENSOR_IR_L))*7/10;

	m_displacementVector[X] += (maxIRrange - m_robot->getSensorValue(SENSOR_IR_FRONT_L))*7/10;
	m_displacementVector[Y] -= (maxIRrange - m_robot->getSensorValue(SENSOR_IR_FRONT_L))*7/10;

	m_displacementVector[X] -= (maxIRrange - m_robot->getSensorValue(SENSOR_IR_R))*7/10;
	m_displacementVector[Y] -= (maxIRrange - m_robot->getSensorValue(SENSOR_IR_R))*7/10;

	m_displacementVector[X] -= (maxIRrange - m_robot->getSensorValue(SENSOR_IR_FRONT_R))*7/10;
	m_displacementVector[Y] -= (maxIRrange - m_robot->getSensorValue(SENSOR_IR_FRONT_R))*7/10;

	m_displacementVector[Y] += (maxIRrange - m_robot->getSensorValue(SENSOR_IR_BACK));

}

//the displacement the robot should have if no collision is detected
void Simulation::displacement()
{
	//for the moment just move forward;
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = VAL_WHEELS_FW;
	//TODO : real displacement
}

void Simulation::homeDisplacement()
{
	//TODO : move in direction of a certain absolute angle;
	m_displacementVector[X] = 0;
	m_displacementVector[Y] = 0;
}

 //change direction of brush if overcurrent
bool Simulation::brushIsBlocked()
{
	if(m_robot->getBrushCurrent() > 200) //300mA
	{
		m_robot->setBrushSpeed(VAL_BRUSH_BW);
		usleep(300000);
		m_robot->setBrushSpeed(VAL_BRUSH_FW);
		return true;
	}
	return false;
}

void Simulation::emergencyProcedure()
{
	//TODO : add the procedure
}

//detect an emergency : way too close to an obstacle or perhaps the lift is blocked...
bool Simulation::emergencyDetected()
{
	//TODO : add the condition
	return false;
}

void Simulation::goHome()
{
	homeDisplacement();				//normal robot displacement if no collision detected
	avoidObstaclesIR();			//avoid obstacles viewed with IR
	avoidObstaclesCam();		//avoid obstacles viewed with cam

	if(homeReached())
	{
		// TODO rotate 180 degrees;
		emptyTailGate();
		m_bottlesCollected = 0;
	}
}

void Simulation::emptyTailGate()
{
	m_robot->setTailGate(VAL_TAIL_OPEN);
	usleep(2000000);
	m_robot->setTailGate(VAL_TAIL_CLOSE);
}

bool Simulation::homeReached()
{
	//TODO add the condition
	return false;
}

void Simulation::search()
{
	displacement();				//normal robot displacement if no collision detected
	avoidObstaclesIR();			//avoid obstacles viewed with IR
	avoidObstaclesCam();		//avoid obstacles viewed with cam
	approachBottlesCam();		//move towards bottles viewed with cam
}
