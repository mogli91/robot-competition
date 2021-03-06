/*
 * Arduino micro 1 : the one with the sensors...
 * has 5 IR sensors, 
 * 4 Ultrasound sensors
 * controls the 
 motor
 * extra pins available for serial transfer
 */
 int k;
 
 #include "IR.h"
 #include "DynamixelSerial.h"
 #include "defines.h"
 
/* /////////////////
 * Pin configuration 
 *//////////////////
 
// infrared sensors
#define IRL A0  //left        a
#define IRFL A1 //front left  b
#define IRFR A2 //front right c
#define IRR A3  //right       d
#define IRB A4  //back        e

#define IRBRUSH      A5
#define IRBL     A7//6 //A7 //back left
#define IRBR     8 //A8 //back right
//#define IRBR      8 //A8

//brush motor
#define CS A11   //current sensor : 140mv per amp
#define PWM_ 11  //pwm pin
#define INA 9    //clockwise
#define INB 10   //counter clockwise

//serial protocol..
#define TX 1
#define RX 0

//dynamixel
#define DYMX_ID 14
//#define DYMX_ID2 13
#define CLOCKWISE 1
#define ANTICLOCKWISE 0

#define STACKSIZE 10
int stack[STACKSIZE];
//int stackCounter;

int stackInit()
{
  for(int i = 0; i < STACKSIZE; i++)
  {
      stack[i] = 0;
  }
  //stackCounter = 0;
}

//gives the average value of the stack
int stackAvg()
{
    int sum = 0;
    for(int i = 0; i < STACKSIZE; i++)
    {
      sum += stack[i];
    }
    
    return  sum/STACKSIZE;
}

//stackarray function
void stackAdd(int newdata)
{
    for(int i = STACKSIZE-1; i >= 1; i--)
    {
        stack[i] = stack[i-1];
    }
    stack[0] = newdata;
}

//-------------------------------------------------------------- communication  -----------------------------------------------------

//#define READING_SIZE 5      // number of bytes per package from microcontroller -> odroid
//#define CMD_SIZE 5
//#define MSG_MAX 64
//#define BAUDRATE 115200
int buf = -1;
char val_str[4] = {0, 0, 0, '\0'};
char message[MSG_MAX];
char current_message_end;
unsigned long t0, t1, tus;
//const unsigned long communication_interval = 33; // message exchange at ~30Hz
const unsigned long communication_interval = 100; // message exchange at ~30Hz

int ir_prev[9];
int us_prev[2];

// code starts here

void ir_init()
{
   IR_setup(IRL);
   IR_setup(IRFL);
   IR_setup(IRFR);
   IR_setup(IRR);
   IR_setup(IRB);
   IR_setup(IRBL);
   IR_setup(IRBR);
   IR_setup(IRBRUSH);
   
   for(int i = 0; i < 8; i ++)
   {
      ir_prev[i] = 80;
   }
}


void motor_setup(int cs, int pwm, int ina, int inb)
{
    pinMode(cs, INPUT);
    pinMode(pwm, OUTPUT);
    pinMode(ina, OUTPUT);
    pinMode(inb, OUTPUT);
    
    //set clockwise direction as default
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
}

void setup()                    // run once, when the sketch starts
{
    
    //setup infrareds
    ir_init();
    
    //setup motor
    motor_setup(CS, PWM_, INA, INB);
    
    //setup dynamixel
    Dynamixel.begin(1000000, 2);  // Initialize the servo at 1Mbps and Pin Control 2
    Dynamixel.setEndless(DYMX_ID, OFF);    
    //Dynamixel.setEndless(DYMX_ID2, OFF); 
    // communication
    memset(message, '\0', MSG_MAX);
    current_message_end = 0;
    Serial.begin(BAUDRATE);
    Serial.flush();
    k = 0;
    //stackInit();
}

void loop()                       // run over and over again
{
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
//
//    // TODO perform measurements and calculations
    int ir_new[8] = {80,80,80,80,80,80,80,80};
    int filt_ir = 9;
    //filtering
    ir_new[0] = ((10-filt_ir)*ir_prev[0] + filt_ir*IR_get_dist(IRL))/10;
    ir_new[1] = ((10-filt_ir)*ir_prev[1] + filt_ir*IR_get_dist(IRFL))/10;
    ir_new[2] = ((10-filt_ir)*ir_prev[2] + filt_ir*IR_get_dist(IRFR))/10;
    ir_new[3] = ((10-filt_ir)*ir_prev[3] + filt_ir*IR_get_dist(IRR))/10;
    ir_new[4] = ((10-filt_ir)*ir_prev[4] + filt_ir*IR_get_dist(IRB))/10;
    ir_new[5] = ((10-filt_ir)*ir_prev[5] + filt_ir*IR_get_dist(IRBL))/10;
    ir_new[6] = ((10-filt_ir)*ir_prev[6] + filt_ir*IR_get_dist(IRBR))/10;
    ir_new[7] = ((10-filt_ir)*ir_prev[7] + filt_ir*IR_get_dist(IRBRUSH))/10;
   
    for(int i = 0; i < 8; i++)
    {
        if(ir_new[i] > 80)
          ir_new[i] = 80;
    }
    
    print_value(SENSOR_IR_L, ir_new[0]);
    print_value(SENSOR_IR_FRONT_L, ir_new[1]);
    print_value(SENSOR_IR_FRONT_R, ir_new[2]);
    print_value(SENSOR_IR_R, ir_new[3]);
    print_value(SENSOR_IR_BACK, ir_new[4]);
    print_value(SENSOR_IR_BACK_L, ir_new[5]);
    print_value(SENSOR_IR_BACK_R, ir_new[6]);
    print_value(SENSOR_IR_BRUSH , ir_new[7]);

    
    for(int i = 0; i < 8; i++)
    {
        ir_prev[i] = ir_new[i];
    }
    
    if(k)
    {
        dynamixel_move(900);
        delay(3000);
    }
    else
    {
        dynamixel_move(500);
        delay(3000);
    }
    k = !k;
    
//    stackAdd(measure_motor_current());
//    print_value(SENSOR_BRUSH_CURRENT, stackAvg());
    
//    print_value(SENSOR_DYMX_SPEED, measure_dymx_speed());
    
    // NOTE for each measurement that is to be sent, use print_value(char id, int val)
//    motor_check();
}

int measure_US(int pin_trig, int pin_echo) //read US values on a pin and interpret as distance of x cm.
{
    long duration, cm;
 
    pinMode(pin_trig, OUTPUT);
    pinMode(pin_echo, INPUT);
    
    digitalWrite(pin_trig, LOW );
    delayMicroseconds(2);
    //send a 10microsecond impulse to the trigger (it makes the sensor send a wave): I keep it high for 10microsec
    digitalWrite( pin_trig, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( pin_trig, LOW );
     
    
    int timeout = 90000;
    duration = pulseIn( pin_echo, HIGH, timeout );
    if (duration  == 0) duration = timeout;
     
    cm = 0.034 * duration / 2;
     
    return (int)cm;
}

int measure_motor_current() //reads motor current in mA
{
    int raw = analogRead(CS); //returns value from 0 to 1023
    int mV = map(raw, 0, 1023, 0, 5000); //returns voltage ( 0 to 5000 mV)
    int current = 1000.0f*((float)mV)/140.0f; //voltage is 140mv per amp
    
    if(current > 999) //cannot transmit more than 999
        current = 999;
    return current;
}

void handle_motor_command(int command) // handles input from 0 to 999
{
    int value = command - 500;
    if(value >= 0)
       set_motor_direction(CLOCKWISE);
    else
       set_motor_direction(ANTICLOCKWISE);
                    
    int amplitude = abs(value);
    int speed = map(amplitude, 0, 500, 0, 450); //get the voltage from the amplitude
    set_motor_speed(speed);
    // TODO remove maps
}

void set_motor_speed(float speed) //speed from 0 V to 450 cV (WARNING VOLTAGE IS 7.2VOLT INPUT) 
{
    int pwm = map(speed, 0, 720, 0, 100);
    if(pwm >= 100)//4.5/7.2*255 because max voltage is 4.5
      pwm = 100;
    analogWrite(PWM_, pwm);
}

int measure_dymx_speed()
{
  int speed = Dynamixel.readSpeed(DYMX_ID);
  if( speed > 999) speed = 999;
  if(speed < 0) speed = 0;
   return speed;
}

void set_motor_direction(bool clockwise)
{
    if(clockwise)
    {
        digitalWrite(INA, HIGH);
        digitalWrite(INB, LOW);
    }
    else
    {
        digitalWrite(INA, LOW);
        digitalWrite(INB, HIGH);
    }
}

void reverse_motor_direction()
{
  
}

void motor_check()
{
   if(measure_motor_current() > 800) //800 mA is the limit
   {
       set_motor_speed(0);
       Serial.println("motor going to sleep");
       //TODO cooldown time
   }
}

void dynamixel_move(int val)
{
    if(val < 0 || val > 1023) return;
    Dynamixel.moveSpeed(DYMX_ID, val, 100);
    //Dynamixel.moveSpeed(DYMX_ID2, 1023-val, 100);
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
        case CMD_BRUSH:
            //command the motor speed from -500 to 499 (input from 0 to 999)
            handle_motor_command(val);    
            break;
        case CMD_LIFT:
            dynamixel_move(val);
            break;
        default:
            // this shouldn't happen but anyway... just don't react
            // for testing one could just set the command as a reading
            // (if you want to read the commands you sent)
            //print_value(i + 'a', val);
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

//      sprintf(message + idx + 1, "%03d", (val + old_val) / 2); // compute average of this and previous value
      sprintf(message + idx + 1, "%03d", val); // no filter
      message[idx + 4] = nextID; // re-write this ID (got ovewritten by a null character in sprintf
      return true;
    }
    
    char* target = message + current_message_end;
    target[0] = id; // set ID fst
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

