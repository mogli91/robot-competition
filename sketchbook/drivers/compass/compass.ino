#define BAUD_COMPASS 9600
#define BAUD_PC 9600

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


void setup()
{
    Serial1.begin(BAUD_COMPASS);
    Serial.begin(BAUD_PC);
    //Serial.write(restore_default, 4);
    delay(100);
}

void loop()
{
    read();
    delay(100);
}

void read()
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
    Serial.println(angle);
}
