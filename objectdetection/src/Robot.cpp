
/*
* Robot.cpp
*
* Created on: Jun 1, 2015
* Author: snake
*/
#include "Robot.h"
Robot::Robot(Brain* brain) {
// TODO Auto-generated constructor stub
m_brain = brain;
m_map = new Map();
}
Robot::~Robot() {
// TODO Auto-generated destructor stub
}
void Robot::updateData() {
m_brain->updateReadings();
}
void Robot::setWheelSpeeds(int left, int right) {
m_brain->setWheelSpeeds(left, right);
}
void Robot::setTailGate(int tailGateState) {
//TODO : real angles
if (tailGateState == UP)
m_brain->setTailGateAngle(VAL_TAIL_CLOSE);
else
m_brain->setTailGateAngle(VAL_TAIL_OPEN);
}
void Robot::setShovel(int shovelState) {
m_brain->setLiftPosition(shovelState);
//TODO : real angles
// if (shovelState == UP)
// m_brain->setLiftPosition(VAL_LIFT_HIGH);
// else
// m_brain->setLiftPosition(VAL_LIFT_LOW);
}
void Robot::setBrushSpeed(int speed) {
m_brain->setBrushSpeed(speed);
}
int Robot::getSensorValue(char sensorId) {
return m_brain->getReading(sensorId);
}
void Robot::getVisionData(VisionMeasure &vm) {
    m_brain->getVisionData(vm);
}
//int Robot::getBrushCurrent() {
//int val = m_brain->getReading(SENSOR_BRUSH_CURRENT);
//cout <<"\n\n" << val <<"\n\n" <<endl;
//return m_brain->getReading(SENSOR_BRUSH_CURRENT);
//}
Pose Robot::getPose() {
Pose robotPose;
robotPose.x = m_brain->getReading(SENSOR_POSE_X);
robotPose.y = m_brain->getReading(SENSOR_POSE_Y);
robotPose.angle = m_brain->getReading(SENSOR_POSE_ANGLE);
return robotPose;
}
void Robot::changeTileState(int x, int y, Tile newTile) {
m_map->setTile(x, y, newTile);
}
Tile Robot::getTile(int x, int y)
{
return m_map->getTile(x,y);
}
void Robot::printMap()
{
m_map->print();
}
