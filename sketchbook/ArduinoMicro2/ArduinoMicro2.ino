/*
 * Template for code that runs on microcontrollers
 * TODO fill part in loop() and add own needed functions
 */
#include "MultiLinearCameraARD.h"
#include "defines.h"
#include "prior.h"

//#define READING_SIZE 5      // number of bytes per package from microcontroller -> odroid
//#define NUM_READINGS 26     // buffer size for packages (will be referenced by 'a'-'z'
//#define READING_ID_MIN 'a'
//#define READING_ID_MAX 'z'
//#define IS_READING(c) (c >= READING_ID_MIN && c <= READING_ID_MAX)

//#define COMMAND_SIZE 5      // number of bytes per command from odroid -> microcontroller
//#define NUM_COMMANDS 26     // buffer size for commands (have only 5 actors: left/right wheels, tailgate, brush, elevator
//#define COMMAND_ID_MIN 'A'
//#define COMMAND_ID_MAX 'Z'

//lincam exposure time in usec
#define EXPOSURE 3000
unsigned char *image;

#define DELAY 50            // adjust to reach ~20 Hz when everything is fully implemented
int buf = -1;
char sensors[NUM_READINGS][READING_SIZE];   // stores measurements to be sent as lines of <ID, measured value>
char commands[NUM_COMMANDS][CMD_SIZE];  // stores commands to be executed as lines of <ID, desired value>
char *camera_image;
const unsigned int num_pixels = 612;
char img[612];

//prior position
enum POS{X, Y};
int pos[2]; //in cm
// position estimates
int pos_rough[2] = {0,0};
int pos_precise[2] = {0,0};

// pixel locations of beacons and respective deltas to correct compass
int peak_predict[4] = {0, 0, 0, 0};
int delta_px = 0;
int delta_angle = 0;

// direction variables
int direction_compass = 0;
int direction_idx = 0;
int image_y_coordinate = 0;

#define WORLDX 800
#define WORLDY 800

#define TILEX 50
#define TILEY 50

#define NUMX WORLDX/TILEX
#define NUMY WORLDY/TILEY


int dummy = 0;

//compass
////////////////////////
#define BAUD_COMPASS 9600
//#define BAUD_PC 9600

// commands
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
/////////////////////////







void setup()                    // run once, when the sketch starts
{
    // initialize table for sensor values to be sent
    memset(sensors[0], '\0', READING_SIZE * NUM_READINGS);
    for (int s = 0; s < NUM_READINGS; ++s)
    {
        sensors[s][0] = s + READING_ID_MIN; // sets chars 'a', 'b', 'c', ... as ID of a reading
        sensors[s][1] = 0; // init value always 0
    }

    // initialize table for received commands
    memset(commands[0], '\0', CMD_SIZE * NUM_COMMANDS);
    for (int s = 0; s < NUM_COMMANDS; ++s)
    {
        commands[s][0] = s + CMD_ID_MIN; // sets chars 'A', 'B', 'C', ... as ID of a command
        commands[s][1] = 0;
    }

    Serial.begin(BAUDRATE);
    
    //start pos 0 0
    pos[X] = 25; pos[Y] = 25;
    
    //lin cam setup
    lcam_setup();
    memset(img, 155, 612);
    
    
    Serial1.begin(BAUD_COMPASS);
}

void loop()                       // run over and over again
{
    // check for new commands
    if (read_instructions() > 0)
    {
        // execute most recent commands
        execute_commands();
    }

    // TODO perform measurements and calculations
    // NOTE for each measurement that is to be sent, use print_value(char id, int val)
    
    //print_value('a', read_angle()); // angle
    //print_value('b', XXX) // x
    //print_value('c', XXX) // y

    // send new measurements
    estimate_position();
    Serial.print("x = ");
    Serial.println(pos[X]);
    Serial.print("y = ");
    Serial.println(pos[Y]);
    
    write_readings();
    Serial.println("");
    delay(DELAY);
}

/*
    reads as long as it makes sense and returns number of new commands
*/
int read_instructions()
{
    int count = 0;
    while(true)
    {
        buf = Serial.read();
        // check availability and validity of new command
        if (buf == -1 || !isupper(buf) || buf < CMD_ID_MIN || buf > CMD_ID_MAX)
        {
//            if (count > 0) Serial.println(count);
            return count;
        }
        ++count;

        char cmd_id = buf - CMD_ID_MIN;
        
        // reset old command value
        memset(commands[cmd_id] + 1, '\0', CMD_SIZE - 1);
        for (int i = 1; i < CMD_SIZE - 1; ++i)
        {
            buf = Serial.read();
            if (buf == '\0' || buf == -1) break;
            commands[cmd_id][i] = buf;
        }
    }
    return count;
}

/*
    sends the whole table of values
*/
void write_readings()
{
    Serial.write(sensors[0], NUM_READINGS * READING_SIZE);
}

/*
    maps command adresses and values to functions (actuate a motor with a particular value)
*/
void execute_commands()
{
    for (int i = 0; i < NUM_COMMANDS; ++i)
    {
        if (commands[i][1] == '\0') continue;

        int value = atoi(commands[i] + 1); // read value from string to int
        // note: this seems inefficient but we won't have too many commands anyway
        switch (i + CMD_ID_MIN)
        {
            case SENSOR_POSE_X:
                // do something with respective actuator
//                break;
            case SENSOR_POSE_Y:
                // another actuator
//                break;
            case SENSOR_POSE_ANGLE:
                // another actuator
//                break;
            default:
                // this shouldn't happen but anyway... just don't react
                // for testing one could just set the command as a reading
                // (if you want to read the commands you sent)
                //print_value(i + 'a', value);
                break;
        }
        memset(commands[i] + 1, '\0', CMD_SIZE - 1); // reset this command because it's been executed already
    }
}

/*
    writes a captured sensor reading to the table of values that are sent
*/
bool print_value(char id, int val)
{
    if (val < 0 || val > 999) return false;                         // checks value to write
    if (id < READING_ID_MIN || id > READING_ID_MAX) return false;   // checks address in table

    int address = id - READING_ID_MIN;
    char* target = sensors[address];
    ++target;
    memset(target, '\0', READING_SIZE - 1);     // reset value
    sprintf(target, "%d", val);                 // print int to string
    return true;
}

int read_reading(char id)
{
    if (id < READING_ID_MIN || id > READING_ID_MAX) return -1;  // checks address in table
    
    return atoi(sensors[id - READING_ID_MIN] + 1);
}

int read_command(char id)
{
    if (id < CMD_ID_MIN || id > CMD_ID_MAX) return -1;  // checks address in table
    if (commands[id - CMD_ID_MIN][1] == '\0') return -1;    // there is currently no command
    return atoi(commands[id - CMD_ID_MIN] + 1);
}

void estimate_position()
{
    capture();
    //position for example x = 365 y = 298 case x = 3, 4, y = 2, 3
    
    //int th = read_angle();
    
    /*
    x, y values
    -11  01  11
    -10  00  10
    -1-1 0-1 1-1
    
    i values :
    7 8 9
    4 5 6
    1 2 3
    */
    
    ///ROUGH MODE
    // locate what square the robot is in, with a prior position.
    int x, y;
    int sums[9]; // sums of the intensities at the prior peak positions
    int index[4] = {0,0,0,0}; // index for the angles
    for(int i = 0; i < 9; i++) 
    {
        x = pos[X]/TILEX + i%3-1;
        y = pos[Y]/TILEY + i/3-1;
        if((x >= 0 && x < NUMX) && ( y >= 0 && y < NUMY ))
        {
            index[0] = 0; index[1] = 0; index[2] = 0; index[3] = 0;
            for(int j = 0; j < 4; j++)
            {
                if(pgm_read_word(&(locations2[x][y][j])) + 120/*-pgm_read_word_near(angle2px + th)*/ < 0)
                  index[j] = pgm_read_word(&(locations2[x][y][j])) + 120 + 612;//locations2[x][y][j]/*-pgm_read_word_near(angle2px + th)*/ + 612;
                else if(pgm_read_word(&(locations2[x][y][j])) + 120/*-pgm_read_word_near(angle2px + th)*/ >= 612)
                  index[j] = pgm_read_word(&(locations2[x][y][j])) + 120 - 612;//locations2[x][y][j]/*-pgm_read_word_near(angle2px + th)*/ + 612;
                else
                  index[j] = pgm_read_word(&(locations2[x][y][j])) + 120;//locations2[x][y][j];//-pgm_read_word_near(angle2px + th);
                  
                  //Serial.println(index[j]);
            }
            //Serial.println(index[0]);
            //Serial.println(index[1]);
            //Serial.println(index[2]);
            //Serial.println(index[3]);
            sums[i] = image[index[0]] + image[index[1]] + image[index[2]] + image[index[3]]; 
            //Serial.println(sums[i]);
        }
        else sums[i] = 0;
    }
    
    //reset x and y
    x = pos[X]/TILEX;
    y = pos[Y]/TILEY;
       
    //search for the sum maximum :: equivalent to the rough position
    int maxi = 4; // maximum i; begin at the robot usual position
    for(int i = 0; i < 9; i++) 
    {
        if(sums[i] > sums[maxi]) maxi = i;
    }
    
    //Serial.println(maxi);
    
    //give the new rough position in function of the maximum i

    x += maxi%3-1;
    y += maxi/3-1;
    
    if(x < 0) x = 0;
    if(x > 15) x = 15;
    if(y < 0) y = 0;
    if(y > 15) y = 15;

    pos[X] = 25+TILEX*x;
    pos[Y] = 25+TILEY*y;  
    
    /// PRECISE MODE
    /*
    int p[2][2][2]; // four precalculated intensities around the robot position ex p00 x or y
    int w[2][2]; //sums of the 4 peaks
    int tmp = 0; //temp sum
    
    for(int i = 0; i < 4; i++)
    {
        p[i%2][i/2][X] = pos[X]/100 + i%2;
        p[i%2][i/2][Y] = pos[Y]/100 + i/2;
        
        for(int j = 0; j < NUM_BEACONS; j++) // 00 01 10 11
        {
            tmp += image[ locations[ p[i%2][i/2][X] ][ p[i%2][i/2][Y] ][j] + angle2px[th]];
        }
        
        w[i%2][i/2] = tmp;
        tmp = 0;
    }
    
    pos[X] = (w[0][0]*p[0][0][X] + w[1][0]*p[1][0][X] + w[1][1]*p[1][1][X] + w[0][1]*p[0][1][X]) / ( w[0][0] + w[1][0] + w[1][1] + w[0][1] );
    pos[Y] = (w[0][0]*p[0][0][Y] + w[1][0]*p[1][0][Y] + w[1][1]*p[1][1][Y] + w[0][1]*p[0][1][Y]) / ( w[0][0] + w[1][0] + w[1][1] + w[0][1] );
    */
    //return pos;
}

void capture()
{
    lcam_reset();
    lcam_integrate(EXPOSURE);
    lcam_read();
    image = lcam_getdata();
}

int read_angle()
{
    Serial1.write(first_angle); delay(5);
    // wait for beginning of return command
    int i = 0;
    int val[10];
    while ((byte_val = Serial1.read()) > -1)
    {
      val[i] = (char)byte_val;
      i++;
    }
    int angle = 100*(val[2]-48) + 10*(val[3]-48) + val[4]-48;
    return angle;
}
