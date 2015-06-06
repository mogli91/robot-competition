#include <Servo.h>
#include "IOpins.h"
#include "Constants.h"

/*
 * Wild thumper with serial control based on "Template"
 */


//-------------------------------------------------------------- communication  -----------------------------------------------------

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



//-------------------------------------------------------------- define global variables --------------------------------------------

unsigned int Volts;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int Leftspeed=0;
int Rightspeed=0;
int Speed;
int Steer;
byte Charged=1;                                               // 0=Flat battery  1=Charged battery
int Leftmode=1;                                               // 0=reverse, 1=brake, 2=forward
int Rightmode=1;                                              // 0=reverse, 1=brake, 2=forward
byte Leftmodechange=0;                                        // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange=0;                                       // Right input must be 1500 before brake or reverse can occur
int LeftPWM;                                                  // PWM value for left  motor speed / brake
int RightPWM;                                                 // PWM value for right motor speed / brake
int data;
int servo;

#define MODE_BACKWARD 0
#define MODE_BRAKE 1
#define MODE_FORWARD 2

#define SPEED_ZERO 255
#define SPEED_MIN 0
#define SPEED_MAX 510

//-------------------------------------------------------------- define servos ------------------------------------------------------
Servo TailGate;

void setup()
{
    //------------------------------------------------------------ Initialize Servos ----------------------------------------------------
    TailGate.attach(SERVO);
    
    
    //------------------------------------------------------------ Set servos to default position ---------------------------------------

    TailGate.write(70); // default start pos TEST
    //------------------------------------------------------------ Initialize I/O pins --------------------------------------------------

    pinMode (Charger,OUTPUT);                                   // change Charger pin to output
    digitalWrite (Charger,1);                                   // disable current regulator to charge battery

    // communication
    memset(message, '\0', MSG_MAX);
    current_message_end = 0;
    Serial.begin(BAUDRATE);
    Serial.flush();
    t0 = millis();
}


void loop()
{
//  halt();
//  if (doAdminStuff())
  Charged = 1;
  if (true)
  {//----------------------------------------------------------- GOOD BATTERY speed controller opperates normally ----------------------
    // communication
    t1 = millis();
    if ((t1 - t0) >= communication_interval) 
    {
        // check for new commands
        handle_instructions();
        // send collected data
        write_readings();

        t0 = t1;
    }

    // make a drive command (also ensures that motors stop when there is no new speed command
    if (Charged==1)                                           // Only power motors if battery voltage is good
        drive();
    else       {                                               // Battery is flat
        halt();
        Serial.println("low bat");
    }

    // TODO perform measurements and calculations
    // NOTE for each measurement that is to be sent, use print_value(char id, int val)

  }
}

/* shuts down all motors */
void halt() 
{
      // TODO add other motors controlled by this mc
      analogWrite (LmotorA,0);                                // turn off motors
      analogWrite (LmotorB,0);                                // turn off motors
      analogWrite (RmotorA,0);                                // turn off motors
      analogWrite (RmotorB,0);                                // turn off motors
}

/* actuates left and right motors */
void drive()
{
      if ((millis()-leftoverload)>overloadtime)             
      {
        switch (Leftmode)                                     // if left motor has not overloaded recently
        {
        case 2:                                               // left motor forward
          analogWrite(LmotorA,0);
          analogWrite(LmotorB,LeftPWM);
          break;

        case 1:                                               // left motor brake
          analogWrite(LmotorA,LeftPWM);
          analogWrite(LmotorB,LeftPWM);
          break;

        case 0:                                               // left motor reverse
          analogWrite(LmotorA,LeftPWM);
          analogWrite(LmotorB,0);
          break;
        }
      }
      if ((millis()-rightoverload)>overloadtime)
      {
        switch (Rightmode)                                    // if right motor has not overloaded recently
        {
        case 2:                                               // right motor forward
          analogWrite(RmotorA,0);
          analogWrite(RmotorB,RightPWM);
          break;

        case 1:                                               // right motor brake
          analogWrite(RmotorA,RightPWM);
          analogWrite(RmotorB,RightPWM);
          break;

        case 0:                                               // right motor reverse
          analogWrite(RmotorA,RightPWM);
          analogWrite(RmotorB,0);
          break;
        }
      } 
   
}

void set_speed(char id, int val) 
{
    int mode_tmp = MODE_BRAKE;
    int speed_tmp = val - SPEED_ZERO;
    if (speed_tmp < 0) 
    {
        mode_tmp = MODE_BACKWARD;
        speed_tmp = abs(speed_tmp);
    }
    else if (speed_tmp > 0) 
        mode_tmp = MODE_FORWARD;

    switch (id)
    {
        case 'C':
            Leftmode = mode_tmp;
            LeftPWM = speed_tmp;
            break;
        case 'D':
            Rightmode = mode_tmp;
            RightPWM = speed_tmp;
            break;
    }
}

void set_servo_angle(int angle) // command from 0 to 180 degrees
{
    if(angle > 360)
      angle = 360;
    TailGate.write(angle);
}

bool doAdminStuff()
{
  //------------------------------------------------------------ Check battery voltage and current draw of motors ---------------------

  Volts=analogRead(Battery);                                  // read the battery voltage
  LeftAmps=analogRead(LmotorC);                               // read left motor current draw
  RightAmps=analogRead(RmotorC);                              // read right motor current draw

  //Serial.print(LeftAmps);
  //Serial.print("    ");
  //Serial.println(RightAmps);

  if (LeftAmps>Leftmaxamps)                                   // is motor current draw exceeding safe limit
  {
    analogWrite (LmotorA,0);                                  // turn off motors
    analogWrite (LmotorB,0);                                  // turn off motors
    leftoverload=millis();                                    // record time of overload
  }

  if (RightAmps>Rightmaxamps)                                 // is motor current draw exceeding safe limit
  {
    analogWrite (RmotorA,0);                                  // turn off motors
    analogWrite (RmotorB,0);                                  // turn off motors
    rightoverload=millis();                                   // record time of overload
  }

  if ((Volts<lowvolt) && (Charged==1))                        // check condition of the battery
  {                                                           // change battery status from charged to flat

    //---------------------------------------------------------- FLAT BATTERY speed controller shuts down until battery is recharged ----
    //---------------------------------------------------------- This is a safety feature to prevent malfunction at low voltages!! ------

    Charged=0;                                                // battery is flat
    highVolts=Volts;                                          // record the voltage
    startVolts=Volts;
    chargeTimer=millis();                                     // record the time

    digitalWrite (Charger,0);                                 // enable current regulator to charge battery
  }

  //------------------------------------------------------------ CHARGE BATTERY -------------------------------------------------------

  if ((Charged==0) && (Volts-startVolts>67))                  // if battery is flat and charger has been connected (voltage has increased by at least 1V)
  {
    if (Volts>highVolts)                                      // has battery voltage increased?
    {
      highVolts=Volts;                                        // record the highest voltage. Used to detect peak charging.
      chargeTimer=millis();                                   // when voltage increases record the time
    }

    if (Volts>batvolt)                                        // battery voltage must be higher than this before peak charging can occur.
    {
      if ((highVolts-Volts)>5 || (millis()-chargeTimer)>chargetimeout) // has voltage begun to drop or levelled out?
      {
        Charged=1;                                            // battery voltage has peaked
        digitalWrite (Charger,1);                             // turn off current regulator
      }
    } 
    return false;
  }
  else return true;
}

//-------------------------------------------------------------- communication  -----------------------------------------------------

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
        case 'C':
            set_speed('C', val);
            break;
        case 'D':
            set_speed('D', val);
            break;
        case 'E':
            set_servo_angle(val);
            break;
        default:
            // this shouldn't happen but anyway... just don't react
            // for testing one could just set the command as a reading
            // (if you want to read the commands you sent)
            //print_value(i + 'a', value);
            break;
    }
//    print_value(id - 'A' + 'a', val);
//    Serial.write(id);
//    Serial.println(val);
}

/*
    writes a captured sensor reading to the table of values that are sent
*/
bool print_value(char id, int val)
{
    if ((current_message_end + READING_SIZE) > MSG_MAX)
      return false; 
    if (val < 0 || val > 999)
      return false;                         // checks value to write
    if (!islower(id))
      return false;   // checks address in table
    
    char* target = message + current_message_end;
    target[0] = id; // set ID first
    ++target;
    sprintf(target, "%03d", val); // print with leading zeros
    current_message_end += READING_SIZE - 1; // ignore '\0'

    return true;
}


