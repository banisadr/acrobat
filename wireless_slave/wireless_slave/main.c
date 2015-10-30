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

Modified by Bahram on 10/29/15 to recieve ADC data via RF

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

#define CHANNEL 7
#define RXADDRESS 0x7C
#define PACKET_LENGTH 3

/************************************************************
Prototype Functions
************************************************************/

void init(void); // Initialize system clock & pins
void usb_enable(void); // Enable USB
void wireless_enable(void); // Initialize the wireless system
void wireless_recieve(void); // Send data to slave

/************************************************************
Global Variables
************************************************************/

char buffer[PACKET_LENGTH] = {0,0,0}; // Wifi output
char Kp = 0; // Proportional Gain
char Ki = 0; // Integral Gain
char Kd = 0; // Derivative Gain

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
	wireless_enable();

	/* Confirm successful initialization(s) */
	m_green(ON);
	
    while (1) 
    {}
}

/************************************************************
Subroutines and Functions
************************************************************/

/* Initialization of Pins and System Clock */
void init(void){
	m_clockdivide(3); // Set to 2 MHz
	
	sei(); // Enable global interrupts
}

/* Setup USB */
void usb_enable(void)
{
	m_usb_init();
	while(!m_usb_isconnected());
}

/* Initialize the Wireless System */
void wireless_enable(void)
{
	m_bus_init(); // Enable mBUS
	m_rf_open(CHANNEL,RXADDRESS,PACKET_LENGTH); // Configure mRF
}

/* Recieve Wireless Data */
void wireless_recieve(void)
{
	m_rf_read(buffer,PACKET_LENGTH); // Read RF Signal
	Kp = buffer[0];
	Ki = buffer[1];
	Kd = buffer[2];
	
	m_usb_tx_string("Kp= ");
	m_usb_tx_int(Kp);
	m_usb_tx_string("     Ki= ");
	m_usb_tx_int(Ki);
	m_usb_tx_string("     Kd= ");
	m_usb_tx_int(Kd);
	m_usb_tx_string("\n");
}

/************************************************************
Interrupts
************************************************************/

ISR(INT2_vect){
	wireless_recieve();
}

/************************************************************
End of Program
************************************************************/