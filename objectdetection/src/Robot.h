
/*
* Robot.h
*
* Created on: Jun 1, 2015
* Author: snake
*/
#ifndef ROBOT_H_
#define ROBOT_H_
#include "brain.h"
#include "Map.h"
enum Wheels{LEFT, RIGHT};
enum ServoState{UP, DOWN};
enum Sensor{IRL, IRFL, IRFR, IRR, IRB, USL, USFL, USFR, USR};

struct Pose
{
	int x;
	int y;
	int angle;
};

class Robot {
public:
	Robot(Brain* brain);
	virtual ~Robot();
	void updateData();
    void sendInstructions(){if (m_brain != NULL) m_brain->sendInstructions();};
	void setWheelSpeeds(int left, int right);
	void setTailGate(int tailGateState);
	void setShovel(int shovelState);
	void setBrushSpeed(int speed);

	int getSensorValue(char sensorId);
//	int getBrushCurrent();

	Pose getPose();

	void changeTileState(int x, int y, Tile newTile); // adds values to the map
	Tile getTile(int x, int y);	//returns tile contents

	//TODO : stuff with camera ? for example int getBottleDirection
    void getVisionData(VisionMeasure &vm);
	void printMap();

private:
	Brain* m_brain;
	Map* m_map;
};
#endif /* ROBOT_H_ */
