/*
 *  LinearCamerARD.cpp
 *  
 *	Creatded by ROBOPOLY
 *  Modified by Arnaud GArnier on 05.03.13.
 *  Copyright 2013 EPFL. All rights reserved.
 *	 
 *
 */

#include "MultiLinearCameraARD.h"
#include "Arduino.h"
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>

unsigned char lcam_buffer[612];
/*unsigned char lcam_buffer1[102];
unsigned char lcam_buffer2[102];
unsigned char lcam_buffer3[102];
unsigned char lcam_buffer4[102];
unsigned char lcam_buffer5[102];*/

void lcam_pulse()
{
    digitalWrite(LCAM_SCLK, HIGH);
    digitalWrite(LCAM_SCLK, LOW);
}

void lcam_pulse_clock(unsigned char times)
{
    for(unsigned char i = 0; i < times; i++)
    {
		lcam_pulse();
    }
}

// Send a byte to the linear camera

void lcam_send(unsigned char value)
{
    // Start bit
    digitalWrite(LCAM_SDIN, LOW);
    lcam_pulse();
    // Send 8 bits to camera
    for(unsigned char i = 0; i < 8; i++)
    {
		digitalWrite(LCAM_SDIN, ((value >> i) & 1));
		lcam_pulse();
    }
    // Stop bit
    digitalWrite(LCAM_SDIN, HIGH);
    lcam_pulse();
}

// Initialization Sequence
void lcam_reset(void)
{
    // allow a minimum of 1 millisecond for the internal analog circuitry to settle
    delay(100);

    // 10 clock impulsions with SDIN held high to clear the receiver logic
    digitalWrite(LCAM_SDIN, HIGH);
    lcam_pulse_clock(50);

    // 3 reset instructions to clear the control logic
    lcam_send(0x1b);
    lcam_send(0x1b);
    lcam_send(0x1b);

    // 30 clock impulsions to assure the state of SDOUT
    lcam_pulse_clock(30);

    // register write mode
    lcam_send(0X5F);

    // Clear mode register (single chip, not sleep)
    lcam_send(0x00);
}

void lcam_setup(void)
{
    lcam_reset();
lcam_pulse_clock(10);
    // set inputs and outputs
    pinMode(LCAM_SCLK, OUTPUT);
    pinMode(LCAM_SDIN, OUTPUT);
    pinMode(LCAM_SDOUT0, INPUT);
    pinMode(LCAM_SDOUT1, INPUT);
	pinMode(LCAM_SDOUT2, INPUT);
	pinMode(LCAM_SDOUT3, INPUT);
	pinMode(LCAM_SDOUT4, INPUT);
	pinMode(LCAM_SDOUT5, INPUT);
    // make sure the clock is cleared
    digitalWrite(LCAM_SCLK, LOW);
    // Left offset
    lcam_send(0x40);
    lcam_send(0);
lcam_pulse_clock(10);
    // Left gain
    lcam_send(0x41);
    lcam_send(15);
lcam_pulse_clock(10);
    // Middle offset
    lcam_send(0x42);
    lcam_send(0);
lcam_pulse_clock(10);
    // Middle gain
    lcam_send(0x43);
    lcam_send(15);
lcam_pulse_clock(10);
    // Right offset
    lcam_send(0x44);
    lcam_send(0);
lcam_pulse_clock(10);
    // Right gain
    lcam_send(0x45);
    lcam_send(15);
lcam_pulse_clock(10);
}

void lcam_startintegration(void)
{
    // Send start integration command
    lcam_send(0x08);
    // delayed until the pixel reset cycle has been completed (22-clock delay)
    lcam_pulse_clock(22);
}

void lcam_endintegration(void)
{
    // Sample int command
    lcam_send(0x10);
    // pixel reset sequence is initiated, requires 22 clocks
    lcam_pulse_clock(22);
}

// shortcut for starting and ending integration
void lcam_integrate(unsigned int microseconds)
{
    lcam_startintegration();
    delayMicroseconds(microseconds);
    lcam_endintegration();
}

// Tell the camera to be ready to send data
void lcam_read(void)
{
	int j;
    	unsigned char i, pixel_bit, pixel0, pixel1, pixel2, pixel3, pixel4, pixel5;

    	// Read pixel command
    	lcam_send(0x02);
    // 44-clock cycle delay until the first pixel data is output
    lcam_pulse_clock(44);
    // Read the 102 pixels from the camera
    for(i = 0; i < 102; i++)
    {
		pixel0 = 0;
		pixel1 = 0;
		pixel2 = 0;
		pixel3 = 0;
		pixel4 = 0;
		pixel5 = 0;

		// pulse the pixel start bit (SDOUT = 0)
		lcam_pulse();
		// read a byte, bit by bit
		for(pixel_bit = 0; pixel_bit < 8; pixel_bit++)
		{
			digitalWrite(LCAM_SCLK, HIGH);
			// read pin while clock is at 1
			//Serial.print(digitalRead(LCAM_SDOUT));
			
			pixel0 |= (digitalRead(LCAM_SDOUT0) << pixel_bit);
			pixel1 |= (digitalRead(LCAM_SDOUT1) << pixel_bit);
			pixel2 |= (digitalRead(LCAM_SDOUT2) << pixel_bit);
			pixel3 |= (digitalRead(LCAM_SDOUT3) << pixel_bit);
			pixel4 |= (digitalRead(LCAM_SDOUT4) << pixel_bit);
			pixel5 |= (digitalRead(LCAM_SDOUT5) << pixel_bit);

			digitalWrite(LCAM_SCLK, LOW);
		}

		// store byte to buffer
		lcam_buffer[0+i] = pixel0;
		lcam_buffer[1*NPIXELS+i] = pixel1;
		lcam_buffer[2*NPIXELS+i] = pixel2;
		lcam_buffer[3*NPIXELS+i] = pixel3;
		lcam_buffer[4*NPIXELS+i] = pixel4;
		lcam_buffer[5*NPIXELS+i] = pixel5;
		// pulse the pixel stop bit (SDOUT = 1)
		lcam_pulse();
    }
}

unsigned char* lcam_getdata()
{
    return lcam_buffer;
}
/*
unsigned char* lcam_getdata1()
{
    return lcam_buffer1;
}

unsigned char* lcam_getdata2()
{
    return lcam_buffer2;
}

unsigned char* lcam_getdata3()
{
    return lcam_buffer3;
}

unsigned char* lcam_getdata4()
{
    return lcam_buffer4;
}

unsigned char* lcam_getdata5()
{
    return lcam_buffer5;
}
*/

// Divide the 100 first pixels into 25 4-byte averages and return the highest average index
/*unsigned char lcam_getpic(void)
{
    unsigned char i, value, highest = 0, max_region = 0;
	unsigned char average;
    for(i = 0; i < 25; i++)
    {
		// take 4-byte average and divide by 4 (shift to right by 2)
		value = ((lcam_buffer[i*4] + lcam_buffer[i*4+1] + lcam_buffer[i*4+2] + lcam_buffer[i*4+3]) >> 2);
		if(value > highest)
		{
			highest = value;
			max_region = i;
		}
		average += value;
		
    }
	average /= 25;

	//return max_region;
	if(highest > average + 30)
	{
		return max_region;
	}
	else
	{
		return 0;
	}
}*/

