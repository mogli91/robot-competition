#ifndef BRAIN_H
#define BRAIN_H

#include "pthread.h"
#include <cassert>
#include <cctype>

// locals
#include "defines.h"
#include "serial.h"
#include "detector.h"

class Brain
{
private:
    char m_commands_raw[MAX_PORTS][MSG_MAX];
    int m_readings[NUM_READINGS];
    int m_current_message_end[MAX_PORTS];
    int m_command_map[MAX_PORTS][2];
    
    Serial **m_ports;
    int *m_portmap;
    int m_numports;
    Detector *m_detector;
    VisionMeasure m_vision;
    
public:
    Brain(Serial **ports, int *portmap, int numports, Detector *detector = NULL);
    void updateReadings();
    void sendInstructions();
    void printInstructions();
    void printReadings();
    int getReading(char id);
    void getVisionData(VisionMeasure &vm);
    int getNumPorts();
    
    bool setCommand(char id, int val); // TODO make private
    bool setWheelSpeeds(int left, int right);
    bool setTailGateAngle(int angle);
    bool setBrushSpeed(int speed);
    bool setLiftPosition(int position);
};

#endif // BRAIN_H