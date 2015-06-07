#include "brain.h"

Brain::Brain(Serial **ports, int *portmap, int numports, Detector *detector) {
	m_numports = numports;
	m_ports = ports;
    m_portmap = portmap;
    m_detector  = detector;
    
	memset(m_commands_raw, CMD_NUL, MSG_MAX);
    memset(m_readings, VAL_NONE, NUM_READINGS * sizeof(int));
    memset(m_current_message_end, 0, MAX_PORTS * sizeof(int));

    printf("Brain will be connected to %d ports\n\r", m_numports);
    printf("portmap: 0 -> %d, 1 -> %d, 2 -> %d.\n\r", m_portmap[0], m_portmap[1], m_portmap[2]);
    
    // define which ranges of commands are sent to which controller
    m_command_map[ID_ARD1][0] = CMD_BRUSH;
    m_command_map[ID_ARD1][1] = CMD_LIFT;
    
    m_command_map[ID_ARD2][0] = CMD_NONE; // for testing: map all commands to WTC
    m_command_map[ID_ARD2][1] = CMD_NONE;
    
    m_command_map[ID_WTC][0] = CMD_WHEELS_L; // for testing: map all commands to WTC
    m_command_map[ID_WTC][1] = CMD_TAIL;
}

void Brain::updateReadings() {
	Serial *ser;
	char *read;
	pthread_mutex_t *mutex;
	int newread = 0;
	char *endofbuffer = 0;
    char str_val[4] = {'0', '0', '0', '\0'};
    int s;
    int idx;

	for (int p = 0; p < m_numports; ++p) {
        ser = m_ports[p];
		mutex = (pthread_mutex_t*) ser->getMutex();
		pthread_mutex_lock(mutex);
		newread = ser->newRead();
		if (newread) {

			// copy data into local buffer
			read = ser->getRead();
			endofbuffer = read + newread;

			while (read < endofbuffer) {
				if (IS_READING(read[0])) {
                    idx = read[0] - READING_ID_MIN;
                    ++read;
                    for (s = 0; s < 3; ++s)
                    {
                        if (!isdigit(read[0])) {
                            break;
                        }
                        str_val[s] = read[0];
                        ++read;
                    }
                    if (s == 3) {
                        m_readings[idx] = atoi(str_val);
                    }
                } else {
                    ++read;
                }
			}
			ser->resetRead();
		}
		pthread_mutex_unlock(mutex);
		newread = 0;
	}
    
    if (m_detector != NULL) {
        m_detector->getMeasurement(m_vision);
        for (vector<Point>::iterator it = m_vision.bottles.begin(); it != m_vision.bottles.end(); ++it) {
            printf("Bottle at: %d degrees, %d cm\n\r", it->x, it->y);
        }
    }
}

void Brain::sendInstructions() {

    for (int p = 0; p < m_numports; ++p)
    {
        Serial *ser = m_ports[p];
        
        if (ser == NULL) {
            printf("Cannot send commands because device %d is not connected.\n\r", p);
            return;
        }
        char *cmds = m_commands_raw[p];
        pthread_mutex_t *mutex = (pthread_mutex_t*) ser->getMutex();
        pthread_mutex_lock(mutex);
        ser->setWrite(cmds, m_current_message_end[p]);
        pthread_mutex_unlock(mutex);

        memset(cmds, CMD_NUL, m_current_message_end[p]);
        m_current_message_end[p] = 0;
    }

}

void Brain::printInstructions() { // TODO test
	char* cmds;
	for (int i = 0; i < MAX_PORTS; ++i) {
        cmds = m_commands_raw[i];
        
        for (int j = 0; j < m_current_message_end[i]; j += CMD_SIZE) {
            printf("%s\n", cmds);
            cmds += CMD_SIZE;
        }
	}
}

void Brain::printReadings() {
	for (int i = 0; i < NUM_READINGS; ++i) {
        if (m_readings[i] == VAL_NONE)
            continue;
        printf("%c%d;", i + READING_ID_MIN, m_readings[i]);
	}
    printf("\n\r");
}

bool Brain::setCommand(char id, int val)
{
    assert(val >= 0 && val <= 999);                         // checks value to write
    assert(id >= CMD_ID_MIN && id <= CMD_ID_MAX);   // checks address in table
    
    int deviceID = -1;
    // find micro controller to map to
    for (int i = 0; i < MAX_PORTS; ++i) {
        if (id >= m_command_map[i][0] && id <= m_command_map[i][1]) {
            deviceID = i;
            break;
        }
    }
    int portnumber = m_portmap[deviceID];
//    assert(deviceID != ID_NONE);
//    assert(portnumber != -1);
//    assert(m_current_message_end[portnumber] <= (MSG_MAX - CMD_SIZE));
    
    if (deviceID == ID_NONE || portnumber == -1 || m_current_message_end[portnumber] > (MSG_MAX - CMD_SIZE))
        return false;

    char* target = m_commands_raw[portnumber];
    
    int current_end = m_current_message_end[portnumber];
    
    // check if command id already exists
    for (int i = 0; i < current_end; i += 4)
    {
        if (target[i] == id)
        {
            char nextID = target[i + 4];
            sprintf(target + i + 1, "%03d", val); // overwrite former command
            target[i + 4] = nextID; // re-write this ID (got ovewritten by a null character in sprintf
            return true;
        }
    }
    
    // append new command in the end
    target += current_end;
    target[0] = id; // set ID first
    ++target;
    sprintf(target, "%03d", val); // print with leading zeros
    
    m_current_message_end[portnumber] += CMD_SIZE - 1; // ignore null termination
    
    return true;
}

int Brain::getReading(char id)
{
    if (id < READING_ID_MIN || id > READING_ID_MAX) return -1;  // checks address in table
    int address = id - READING_ID_MIN;
    
    return m_readings[address];
}

void Brain::getVisionData(VisionMeasure &vm) {
    vm = m_vision;
}

int Brain::getNumPorts() {
    return m_numports;
}

bool Brain::setWheelSpeeds(int left, int right) {
    if (left < VAL_WHEELS_BW || left > VAL_WHEELS_FW) {
        printf("left speed out of bounds\n");
        return false;
    }
    if (right < VAL_WHEELS_BW || right > VAL_WHEELS_FW) {
        printf("right speed out of bounds\n");
        return false;
    }
    setCommand(CMD_WHEELS_L, left);
    setCommand(CMD_WHEELS_R, right);
    return true;
}

bool Brain::setTailGateAngle(int angle) {
    if (angle < VAL_TAIL_CLOSE || angle > VAL_TAIL_OPEN) {
        printf("tail gate position out of bounds\n");
        return false;
    }
    setCommand(CMD_TAIL, angle);
    return true;
}

bool Brain::setBrushSpeed(int speed) {
    if (speed > VAL_BRUSH_BW || speed < VAL_BRUSH_FW) {
        printf("brush speed out of bounds\n");
        return false;
    }
    setCommand(CMD_BRUSH, speed);
    return true;
}

bool Brain::setLiftPosition(int position) {
    // here high is lower than low...
    if (position < VAL_LIFT_HIGH || position > VAL_LIFT_LOW) {
        printf("lift position out of bounds\n");
        return false;
    }
    setCommand(CMD_LIFT, position);
    return true;
}
