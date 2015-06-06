/*
  Template for code that runs on microcontrollers
 * TODO fill part in loop() and add own needed functions
 */

// COMMUNICATION parameters
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


void setup()                    // run once, when the sketch starts
{
    // reset serial message buffer
    memset(message, '\0', MSG_MAX);
    current_message_end = 0;
    Serial.begin(BAUDRATE);
    Serial.flush();
    t0 = millis();
}

void loop()                       // run over and over again
{
    t1 = millis();
    if ((t1 - t0) >= communication_interval) 
    {
        // check for new commands
        handle_instructions();
        // send collected data
        write_readings();

        t0 = t1;
    }
    
    // TODO perform measurements and calculations here
    // NOTE for each measurement that is to be sent, use print_value(char id, int val)

}

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
        Serial.println("");
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
    print_value(id - 'A' + 'a', val);
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
