/************************************************************
Header Information
************************************************************/

/*
 * main.c
 *
 * Authors: Wyatt Shapiro, Pete Furlong, Bahram Banisadr
 * MEAM 510
 * Lab 6: Acrobat
 */ 


/*****************************************************************
==================================================================
### INTERNAL README: ADD TO EVERYTIME BEFORE SENDING TO GITHUB ###

Modified by Bahram on 10/29/15 to create framework

==================================================================
*****************************************************************/


/************************************************************
Included Files & Libraries
************************************************************/
#include <avr/io.h>
#include "m_general.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_usb.h"
#include "m_imu.h"

/************************************************************
Definitions
************************************************************/

#define ALPHA_LOW 0.95
#define ALPHA_HIGH 0.5
#define AX_OFFSET -136
#define AZ_OFFSET 39
#define GY_OFFSET -121

/************************************************************
Prototype Functions
************************************************************/

void init(void); //Setup I/O, clockspeed, IMU, Interrupts
void usb_enable(void); //Setup USB
int lowpass(float alpha, int previous_output, int reading); //Lowpass filter
int highpass(float alpha, int previous_output, int previous_reading, int reading); //Highpass filter


/************************************************************
Global Variables
************************************************************/

unsigned char accel_scale = 1; // +/-1g
unsigned char gyro_scale = 1;// +/- 125 degrees
int data[9]={0}; // IMU data buffer
int ax = 0;
int az = 0;
int gy = 0;


/************************************************************
Main Loop
************************************************************/
int main(void)
{
	/* Confirm Power */
	m_red(ON);

	/* Initializations */
	init();
	usb_enable();
	int gy_previous_reading = 0;

	/* Confirm successful initialization(s) */
	m_green(ON);

	/* Run */
	while (1){
		if (m_imu_raw(data))
		{
			m_green(ON);
			m_red(OFF);
			
			
			ax = lowpass(0.85,ax,data[0])+AX_OFFSET;
			az = lowpass(0.85,az,data[2])+AZ_OFFSET;
			gy = lowpass(ALPHA_LOW,gy,data[4])+GY_OFFSET;
			gy = highpass(ALPHA_HIGH,gy,gy_previous_reading,data[4]);
			gy_previous_reading = data[4];
			
			/*
			m_usb_tx_string("ax= ");
			m_usb_tx_int(ax);
			m_usb_tx_string("     az=");
			m_usb_tx_int(az);
			m_usb_tx_string("     gy=");
			m_usb_tx_long(gy);
			m_usb_tx_string("\n");
			*/
			
			
			int angle = ax/sqrt(ax*ax+az*az);
			
			/*
			m_usb_tx_string("ax= ");
			m_usb_tx_int(data[0]);
			m_usb_tx_string("     ay= ");
			m_usb_tx_int(data[1]);
			m_usb_tx_string("     az= ");
			m_usb_tx_int(data[2]);
			m_usb_tx_string("     gx= ");
			m_usb_tx_int(data[3]);
			m_usb_tx_string("     gy= ");
			m_usb_tx_int(data[4]);
			m_usb_tx_string("     gz= ");
			m_usb_tx_int(data[5]);
			m_usb_tx_string("\n");
			*/
			
		} 
		else
		{
			m_green(OFF);
			m_red(ON);
		}
	}
}

/************************************************************
Initialization of Subsystem Components
************************************************************/

/* Initialization of Pins and System Clock */
void init(void){
	
	m_clockdivide(3); // Set to 2 MHz
	
	//Set to Input
	clear(DDRD,0); // D0
	clear(DDRD,1); // D1
	clear(DDRD,2); // D2
	
	//Set to Output
	
	while(!m_imu_init(accel_scale,gyro_scale)); //Initialize IMU
	
	sei(); // Enable global interrupts
}

/* Setup USB */
void usb_enable(void)
{
	m_usb_init();
	while(!m_usb_isconnected());
}

/* Lowpass Filter using Alpha_low */
int lowpass(float alpha, int previous_output, int reading)
{
	return (int)((float)reading*alpha +(1-alpha)*(float)previous_output);
}

/* Highpass Filter using Aplha_high */
int highpass(float alpha, int previous_output, int previous_reading, int reading)
{
	return (int)((float)previous_output*alpha + alpha*(float)(reading-previous_reading));
}

/************************************************************
Interrupts
************************************************************/



/************************************************************
End of Program
************************************************************/