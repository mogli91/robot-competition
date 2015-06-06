#include "MultiLinearCameraARD.h"

#define BAUD_PC 115200

const unsigned int num_px = 612;
const unsigned int num_cams = 6;
const unsigned int num_px_per_cam = 102;
unsigned char *image;
char img[612];

const unsigned int exposure = 3000; // exposure time in microseconds
void setup()
{
    lcam_setup();
    memset(img, 155, 612);
    
    Serial.begin(BAUD_PC);
}

void loop()
{
//    Serial.write('a');
//    Serial.write(img, num_px);  
    
    capture();
    Serial.write('a');
    Serial.write(image, num_px);

    delay(100);
}

void capture()
{
    lcam_reset();
    lcam_integrate(exposure);
    lcam_read();
    image = lcam_getdata();
}
