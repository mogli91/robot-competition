/*
 * Arduino micro 1 : the one with the sensors...
 * has 5 IR sensors, 
 * 4 Ultrasound sensors
 * controls the brush motor
 * extra pins available for serial transfer
 */
 
/* /////////////////
 * Pin configuration 
 *//////////////////
 
// infrared sensors
#define IRL A0  //left        a
#define IRFL A1 //front left  b
#define IRFR A2 //front right c
#define IRR A3  //right       d
#define IRB A4  //back        e

//ultrasound sensors
#define USL 2  //left
#define USFL 3 //front left
#define USFR 4 //front right
#define USR 5  //right

//brush motor
#define CS A11   //current sensor : 140mv per amp
#define PWM_ 11  //pwm pin
#define INA 9    //clockwise
#define INB 10   //counter clockwise

//serial protocol..
#define TX 1
#define RX 0

#define CLOCKWISE 1
#define ANTICLOCKWISE 0

/* /////////////////
 * Other defines
 *//////////////////
 
#define FORWARD 1 //motor direction
#define BACKWARD 0 //motor direction

#define READING_SIZE 5      // number of bytes per package from microcontroller -> odroid
#define NUM_READINGS 10     // buffer size for packages (will be referenced by 'a'-'z' (read IR, US, and CS)
#define READING_ID_MIN 'a'
#define READING_ID_MAX 'a' + NUM_READINGS - 1
#define IS_READING(c) (c >= READING_ID_MIN && c <= READING_ID_MAX)

#define COMMAND_SIZE 5      // number of bytes per command from odroid -> microcontroller
#define COMMAND_ID_MIN 'A'
#define COMMAND_ID_MAX 'B'
#define NUM_COMMANDS (COMMAND_ID_MAX - COMMAND_ID_MIN + 1)      // buffer size for commands (only 2 : brush speed, between 0 to 9999 ... 0 to 500 is negative)

#define DELAY 50            // adjust to reach ~20 Hz when everything is fully implemented
int buf = -1;
char sensors[NUM_READINGS][READING_SIZE];   // stores measurements to be sent as lines of <ID, measured value>
char commands[NUM_COMMANDS][COMMAND_SIZE];  // stores commands to be executed as lines of <ID, desired value>

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
    // initialize table for sensor values to be sent
    memset(sensors[0], '\0', READING_SIZE * NUM_READINGS);
    for (int s = 0; s < NUM_READINGS; ++s)
    {
        sensors[s][0] = s + READING_ID_MIN; // sets chars 'a', 'b', 'c', ... as ID of a reading
        sensors[s][1] = 0; // init value always 0
    }

    // initialize table for received commands
    memset(commands[0], '\0', COMMAND_SIZE * NUM_COMMANDS);
    for (int s = 0; s < NUM_COMMANDS; ++s)
    {
        commands[s][0] = s + COMMAND_ID_MIN; // sets chars 'A', 'B', 'C', ... as ID of a command
        commands[s][1] = 0;
    }

    Serial.begin(115200);
    
    //setup infrareds
    IR_setup(IRL);
    IR_setup(IRL);
    IR_setup(IRL);
    IR_setup(IRL);
    IR_setup(IRL);
    
    //no need for setup the ultrasound
    
    //setup motor
    motor_setup(CS, PWM_, INA, INB);
    
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
//    print_value('a', measure_IR(IRL));
//    print_value('b', measure_IR(IRFL));
//    print_value('c', measure_IR(IRFR));
//    print_value('d', measure_IR(IRR));
//    print_value('e', measure_IR(IRB));
//    
//    print_value('f', measure_US(USL));
//    print_value('g', measure_US(USFL));
//    print_value('h', measure_US(USFR));
//    print_value('i', measure_US(USR));
//    
//    print_value('j', measure_motor_current(CS));
    
    // NOTE for each measurement that is to be sent, use print_value(char id, int val)
//    motor_check();

    // send new measurements
//    write_readings();

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
        if (buf == -1 || !isupper(buf) || buf < COMMAND_ID_MIN || buf > COMMAND_ID_MAX)
        {
//            if (count > 0) Serial.println(count);
            return count;
        }
        ++count;

        char cmd_id = buf - COMMAND_ID_MIN;
        
        // reset old command value
        memset(commands[cmd_id] + 1, '\0', COMMAND_SIZE - 1);
        for (int i = 1; i < COMMAND_SIZE - 1; ++i)
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
        char address = i + COMMAND_ID_MIN;
        switch (address)
        {
            case 'A': //command the motor speed from -500 to 499 (input from 0 to 999)
                handle_motor_command(value);
                
                break;
            case 'B':
                // another actuator
                break;
            default:
                // this shouldn't happen but anyway... just don't react
                break;
        }
        Serial.write(address);
        Serial.println(value);
        memset(commands[i] + 1, '\0', COMMAND_SIZE - 1); // reset this command because it's been executed already
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
    if (id < COMMAND_ID_MIN || id > COMMAND_ID_MAX) return -1;  // checks address in table
    if (commands[id - COMMAND_ID_MIN][1] == '\0') return -1;    // there is currently no command
    return atoi(commands[id - COMMAND_ID_MIN] + 1);
}

void IR_init()
{
   pinMode (IRL, INPUT);
   pinMode (IRFL, INPUT);
   pinMode (IRFR, INPUT);
   pinMode (IRR, INPUT);
}

int measure_IR(int pin) //read the analog pin and interpret as distance of x cm.
{
   int raw=analogRead(pin);
   int volt=map(raw, 0, 1023, 0, 5000);
   int cm=(21.61/(volt-0.1696))*1000;
   
   return cm;
}

int measure_US(int pin) //read US values on a pin and interpret as distance of x cm.
{
    long duration, cm;

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW );
    delayMicroseconds(2);
    //send a 10microsecond impulse to the trigger (it makes the sensor send a wave): I keep it high for 10microsec
    digitalWrite( pin, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( pin, LOW );
     
    pinMode(pin, INPUT);
    duration = pulseIn( pin, HIGH );
     
    cm = 0.034 * duration / 2;
     
    return (int)cm;
}

int measure_motor_current(int cs) //reads motor current in mA
{
    int raw = analogRead(cs); //returns value from 0 to 1023
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
}

void set_motor_speed(float speed) //speed from 0 V to 450 cV (WARNING VOLTAGE IS 7.2VOLT INPUT) 
{
    int pwm = map(speed, 0, 720, 0, 100);
    if(pwm >= 100)//4.5/7.2*255 because max voltage is 4.5
      pwm = 100;
    analogWrite(PWM_, pwm);
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

void IR_setup(int pin)
{
    pinMode(pin, INPUT);
}

void motor_check()
{
   if(measure_motor_current(CS) > 800) //800 mA is the limit
   {
       set_motor_speed(0);
       Serial.println("motor going to sleep");
       //TODO cooldown time
   }
}
