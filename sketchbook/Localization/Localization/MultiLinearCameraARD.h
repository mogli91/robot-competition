/*
 *  LinearCamerARD.h
 *  
 *	Creatded by ROBOPOLY
 *  Modified by Arnaud GArnier on 05.03.13.
 *  Copyright 2013 EPFL. All rights reserved.
 *
 */

#include <avr/io.h>


#ifndef LinearCameraARD_h
#define LinearCameraARD_h

#define NPIXELS 102

#define LCAM_SDIN  	3  // SDIN: master out, camera in
#define LCAM_SDOUT0 	4  // SDOUT: camera out, master in
#define LCAM_SDOUT1	5
#define LCAM_SDOUT2	8
#define LCAM_SDOUT3	11
#define LCAM_SDOUT4	A0
#define LCAM_SDOUT5	A3

#define LCAM_SCLK  	2 // camera clock

void lcam_pulse(void);
void lcam_pulse_clock(uint8_t times);

void lcam_send(uint8_t value);

void lcam_reset(void);
void lcam_setup(void);

void lcam_startintegration(void);
void lcam_endintegration(void);
void lcam_integrate(unsigned int microseconds);

unsigned char* lcam_getdata(void);
/*unsigned char* lcam_getdata1(void);
unsigned char* lcam_getdata2(void);
unsigned char* lcam_getdata3(void);
unsigned char* lcam_getdata4(void);
unsigned char* lcam_getdata5(void);*/

void lcam_read(void);

unsigned char lcam_getpic(void);


#endif

