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

#define MAX 32767

/************************************************************
Prototype Functions
************************************************************/

void init(void); //Setup I/O, clockspeed, IMU, Interrupts
void usb_enable(void); //Setup USB
int average_array(int a[][], int column ,int num_rows);


/************************************************************
Global Variables
************************************************************/

unsigned char accel_scale = 0; // +/-1g
unsigned char gyro_scale = 0;// +/- 125 degrees
int data[9]={0}; // IMU data buffer
int steady_state[100][6] = 0; //
int count = 0;
//char str[256]=" "; // Terminal printing buffer
//char testStr[50] = "Hello, world!";

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

	/* Confirm successful initialization(s) */
	m_green(ON);

	/* Run */
	while (1){
		if (m_imu_raw(data))
		{
			m_green(ON);
			m_red(OFF);
			
			/*
			steady_state[count][0] = data[0];
			steady_state[count][1] = data[1];
			steady_state[count][2] = data[2];
			steady_state[count][3] = data[3];
			steady_state[count][4] = data[4];
			steady_state[count][5] = data[5];
			
			count++;
			
			m_wait(100);
			
			if (count == 100)
			{
				count = 0;
				m_usb_tx_string("ax= ");
				m_usb_tx_int(average_array(steady_state,0,100));
				m_usb_tx_string("     ay= ");
				m_usb_tx_int(average_array(steady_state,1,100));
				m_usb_tx_string("     az= ");
				m_usb_tx_int(average_array(steady_state,2,100));
				m_usb_tx_string("     gx= ");
				m_usb_tx_int(average_array(steady_state,3,100));
				m_usb_tx_string("     gy= ");
				m_usb_tx_int(average_array(steady_state,4,100));
				m_usb_tx_string("     gz= ");
				m_usb_tx_int(average_array(steady_state,5,100));
				m_usb_tx_string("\n");			
			}
			*/
			
			
			
			/*
			//sprintf(str, "ax = %f", data[0]/(float)MAX);
			m_usb_tx_int(data[0]/(float)3.2767);
			m_usb_tx_string("       ");
			m_usb_tx_int(data[1]/(float)MAX);
			m_usb_tx_string("       ");
			m_usb_tx_int(data[2]/(float)MAX);
			m_usb_tx_string("\n");
			*/
			
			/*
			m_usb_tx_int(data[3]);
			m_usb_tx_string("       ");
			m_usb_tx_int(data[4]);
			m_usb_tx_string("       ");
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


/* Function to return average of column in 2D array */
int average_array(int a[][], int column ,int num_rows)
{
	int i, sum=0;
	for (i=0; i<num_rows; i++)
	{
		sum = sum + a[i][column];
	}
	return(sum/num_rows);
}


/************************************************************
Interrupts
************************************************************/



/************************************************************
End of Program
************************************************************/