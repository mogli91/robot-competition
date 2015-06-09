#include "defines.h"
#include "prior.h"
#include "pose.h"
#include "MultiLinearCameraARD.h"

#define BAUD_COMPASS 9600

// communication
#define READING_SIZE 5      // number of bytes per package from microcontroller -> odroid
#define CMD_SIZE 5
#define MSG_MAX 64
#define BAUDRATE 115200
int buf = -1;
char val_str[4] = {0, 0, 0, '\0'};
char message[MSG_MAX];
char current_message_end;
unsigned long t0, t1;
const unsigned long communication_interval = 33; // message exchange at ~30Hz



/* compass facts
 *
 * - values between 0.0 and 359.9
 * - frequency response 25Hz
 */
// compass commands
const char first_angle = 0x31;
const char calib_init = 0xc0;
const char restore_default[] = {0xa0, 0xaa, 0xa5, 0xc5};
const char prefix_magnetic_decl_high_byte = 0x03;
const char prefix_magnetic_decl_low_byte = 0x04;
// return values
const char byte0 = 0x0d;
const char byte1 = 0x0a;
const char byte5 = 0x2e;
char value[] = {'0', '0', '0', byte5, '0', '\0'};
int byte_val = -1;

// linear camera
const unsigned int num_px = 612;
const unsigned int num_cams = 6;
const unsigned int num_px_per_cam = 102;
unsigned char *image;
int calibration[612];
const unsigned int exposure = 2000; // exposure time in microseconds

// localization
// pixel locations of beacons and respective deltas to correct compass
int peak_predict[4] = {0, 0, 0, 0};
Pose pose;

void setup() {
  pose.location[X] = 50;
  pose.location[Y] = 50;
  pose.angle_deg = -1;
  pose.delta_old = 0;
  pose.neighbors[N00][X] = 0; // starting in recycling zone
  pose.neighbors[N00][Y] = 0;
  pose.neighbors[N10][X] = 2;
  pose.neighbors[N10][Y] = 0;
  pose.neighbors[N11][X] = 2;
  pose.neighbors[N11][Y] = 2;
  pose.neighbors[N01][X] = 0;
  pose.neighbors[N01][Y] = 2;  
  
  lcam_setup();
  memset(calibration, 0, 612);
  Serial1.begin(BAUD_COMPASS);
  Serial1.write(restore_default,4);
  
  // communication
  memset(message, '\0', MSG_MAX);
  current_message_end = 0;
  Serial.begin(BAUDRATE);
  Serial.flush();
  t0 = millis();
}

void loop() {
  t1 = millis();
  if ((t1 - t0) >= communication_interval) 
  {
      print_value(SENSOR_TIMER, t1 - t0);
      
      // check for new commands
      handle_instructions();
      // send collected data
      write_readings();

      t0 = t1;
  }
   
  int angle_deg_raw = compass_read();
  int local_angle_deg = transformAngleWorld2Local(angle_deg_raw);

  // angular offset to rotate coordinate system
  pose.offset = degree2Offset(local_angle_deg);// + pose.delta_old; // TODO intelligent weighting
  
  print_value(SENSOR_POSE_ANGLE, local_angle_deg);
  
  

  delay(100);
}

// --------------------------------------------------------- CAMERA --------------------------------------------------------

unsigned char *capture()
{
    lcam_reset();
    lcam_integrate(exposure);
    lcam_read();
    return lcam_getdata();
}

void calibrate()
{
    for (int i = 0; i < 612; ++i)
    {
        calibration[i] = (int) image[i];  
    }
}

void correct()
{
    for (int i = 0; i < 612; ++i)
    {
        int tmp = image[i] - calibration[i];
        image[i] = tmp >= 0 ? (unsigned char) tmp : 0;  
    }  
}

// ----------------------------------------------------- COMPASS ---------------------------------------------------------------

int compass_read()
{
    Serial1.write(first_angle);

    // begin of response
    byte_val = Serial1.read();
    if (byte_val != byte0)
        return 0;
    byte_val = Serial1.read();
    if (byte_val != byte1)
        return 1;

    int i = 0;
    for (i = 0; i < 5; ++i)
    {
        byte_val = Serial1.read();
        if (byte_val == -1) 
        {// reset value to zeros
            memset(value, '0', 5);
            break;
        }
        value[i] = byte_val;
    }
    int checksum = Serial1.read(); // don't forget to read checksum (byte7)

    return 100*(value[0]-'0') + 10*(value[1]-'0') + value[2]-'0';
}

// --------------------------------------------------- Communication ----------------------------------------------------------


/*
    reads as long as it makes sense and returns number of new commands
*/
int handle_instructions()
{
    int cmd_count = 0;
    while(cmd_count < 10) // read at most 10 commands at a time (otherwise we block too long)
    {
        buf = Serial.read();
        // check availability and validity of new command
        if (buf == -1 || !isupper(buf))
            break;
            
        char cmd_id = buf;
        int cmd_val = -1;
        int i = 0;
        memset(val_str, 0, 3); // reset command to 0 but leave end-of-string byte
        for (i = 0; i < 3; ++i)
        {
            buf = Serial.read();
            if (buf == -1 || !isdigit(buf))
                break;
            else
                val_str[i] = buf;
        }
        if (i < 3)
        {    
            Serial.print("stopped at i = ");
            Serial.println(i);         
            continue; // value was not valid (less than 3 bytes received)
        }
        ++cmd_count;

        cmd_val = atoi(val_str);
        act(cmd_id, cmd_val);
    }
    return cmd_count;
}

/*
    sends the whole table of values
*/
void write_readings()
{
    if (current_message_end > 0)
    {
        Serial.write(message, current_message_end);
        Serial.println("");
        reset_message();
    }
}

void reset_message() 
{
    memset(message, '\0', current_message_end);
    current_message_end = 0;
}

/*
    maps command adresses and values to functions (actuate a motor with a particular value)
    NOTE: we must decide whether this only calls "setter" functions or whether it actually executes 
*/
int act(char id, int val)
{
    switch (id)
    {
        case 'A':
            // do something with respective actuator
                break;
        case 'B':
            // another actuator
                break;
        default:
            // this shouldn't happen but anyway... just don't react
            // for testing one could just set the command as a reading
            // (if you want to read the commands you sent)
            //print_value(id - 'A' + 'a', val);
            break;
    }
//    print_value(id - 'A' + 'a', val);
}

/*
    writes a captured sensor reading to the table of values that are sent
*/
bool print_value(char id, int val)
{
    char old_str[4]; old_str[3] = '\0';
    int old_val;

    if ((current_message_end + READING_SIZE) > MSG_MAX)
      return false; 
    if (val < 0 || val > 999)
      return false;                         // checks value to write
    if (!islower(id))
      return false;   // checks address in table
      
    int idx = contains(id);
    if (idx > -1)
    {
      char nextID = message[idx + 4];
      
      memcpy(old_str, message + idx + 1, 3);
      old_val = atoi(old_str);

      sprintf(message + idx + 1, "%03d", (val + old_val) / 2); // compute average of this and previous value
      message[idx + 4] = nextID; // re-write this ID (got ovewritten by a null character in sprintf
      return true;
    }
    
    char* target = message + current_message_end;
    target[0] = id; // set ID first
    ++target;
    sprintf(target, "%03d", val); // print with leading zeros
    current_message_end += READING_SIZE - 1; // ignore '\0'

    return true;
}

int contains(char id)
{
    for (int i = 0; i < current_message_end; i += 4)
    {
       if (message[i] == id)
         return i;
    } 
    return -1;
}
