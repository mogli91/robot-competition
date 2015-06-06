#include "MultiLinearCameraARD.h"

#define BAUD_PC 115200
#define BAUD_COMPASS 9600

/* compass facts
 *
 * - values between 0.0 and 359.9
 * - frequency response 25Hz
 */
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

// linear camera
const unsigned int num_px = 612;
const unsigned int num_cams = 6;
const unsigned int num_px_per_cam = 102;
unsigned char *image;
int calibration[612];
const unsigned int exposure = 2000; // exposure time in microseconds

int angle;

void setup()
{
    angle = 0;
    lcam_setup();
    memset(calibration, 0, 612);
    Serial1.begin(BAUD_COMPASS);
    Serial.begin(BAUD_PC);
}

void loop()
{   
    capture();
    correct();
    Serial.write('a');
    Serial.write(image, num_px);

    int r = read();
    if (r < 5)
    {
       Serial.print(r); Serial.println(" bytes"); 
    }
    else
    {
        Serial.write('b');
        Serial.write(value, 5);
        Serial.println("");
    }
    int buf;
    if ((buf = Serial.read()) == 'c')
    {
        calibrate();
    }
    delay(40);
}

void capture()
{
    lcam_reset();
    lcam_integrate(exposure);
    lcam_read();
    image = lcam_getdata();
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

int read()
{
    Serial1.write(first_angle);
//    delay(); // time to send 1 byte and receive 1 byte at 9600bps
//    // wait for beginning of return command
//    int i = 0;
//    
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
    angle = 100*(value[0]-'0') + 10*(value[1]-'0') + value[2]-'0';
    
    return i;
}
