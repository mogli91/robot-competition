#ifndef DEFINES_H
#define DEFINES_H

#define DEBUG

// command line options
#define OPT_EXP         "exp"
#define OPT_ARDUINO_1   "ard1"
#define OPT_ARDUINO_2   "ard2"
#define OPT_WTC         "wtc"
#define OPT_NO_CAM      "no_cam"
#define OPT_BAUDRATE    "br"
#define OPT_IMAGE_LOG_TIME "img_interval"

#define PI 3.14159

// device IDs
#define ID_NONE -1
#define ID_ARD1 0
#define ID_ARD2 1
#define ID_WTC  2

// thread wait times
#define COM_WAIT_TIME 33333 // makes ~30Hz
#define CAM_WAIT_TIME 1000

// data transfer parameters
#define MAX_PORTS 3
#define READING_SIZE 5      // number of bytes per package from microcontroller -> odroid
#define READING_ID_MIN 'a'
#define READING_ID_MAX 'z'
#define NUM_READINGS (READING_ID_MAX - READING_ID_MIN + 1)      // buffer size for packages (will be referenced by 'a'-'z'
#define IS_READING(c) (c >= READING_ID_MIN && c <= READING_ID_MAX)

#define BAUDRATE 115200

// commands
#define CMD_SIZE 5      // number of bytes per command from odroid -> microcontroller
#define CMD_ID_MIN 'A'
#define CMD_ID_MAX 'Z'
#define NUM_COMMANDS (CMD_ID_MAX - CMD_ID_MIN + 1)     // buffer size for commands (have only 5 actors: left/right wheels, tailgate, brush, elevator
#define CMD_NONE        -1
#define CMD_NUL        '\0'
#define CMD_BRUSH       'A' /* Arduino 1 */
#define CMD_LIFT        'B'
#define CMD_WHEELS_L    'C' /* WTC */
#define CMD_WHEELS_R    'D'
#define CMD_TAIL        'E'
#define MSG_MAX          64 // there is no point in sending more bytes than arduino can receive

// special values
#define VAL_NONE         -1
#define VAL_BRUSH_FW     0   // arduino code looks like this should be between 0 and 100
#define VAL_BRUSH_STOP   500
#define VAL_BRUSH_BW     999

#define VAL_LIFT_LOW     900 // 2 bytes on Dynamixel TODO how do they map?
#define VAL_LIFT_TRAVEL  700
#define VAL_LIFT_HIGH    500

#define VAL_WHEELS_BW    0   // only 1 byte on
#define VAL_WHEELS_STOP  255
#define VAL_WHEELS_FW    510

#define VAL_TAIL_OPEN    180 // degrees
#define VAL_TAIL_CLOSE   50

// sensor IDs
#define SENSOR_IR_L               'a'         /* on ARDUINO 1 */
#define SENSOR_IR_FRONT_L         'b'
#define SENSOR_IR_FRONT_R         'c'
#define SENSOR_IR_R               'd'
#define SENSOR_IR_BACK            'e'

#define SENSOR_IR_BACK_L          'f'
#define SENSOR_IR_BACK_R          'g'
#define SENSOR_IR_BRUSH           'h'

#define SENSOR_POSE_X             'i'         /* on ARDUINO 2 */
#define SENSOR_POSE_Y             'j'
#define SENSOR_POSE_ANGLE         'k'

#define SENSOR_ENCODER_L          'l'         /* on WTC       */
#define SENSOR_ENCODER_R          'm'

#define SENSOR_BOTTLE_DISTANCE    'x'
#define SENSOR_BOTTLE_ANGLE       'y'

// vision
#define VISION_NUM_RAYS         16
#define VISION_DIST_BOTTOM      10          // distance of an object if seen in last pixel row
#define VISION_DIST_TOP         250         // distance of an object if seen in first pixel row


#endif // DEFINES_H
