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

Modified by Bahram on 10/29/15 to transmit ADC data via RF

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
#define TXADDRESS 0x7C
#define RXADDRESS 0x6C
#define PACKET_LENGTH 3

/************************************************************
Prototype Functions
************************************************************/

void init(void); // Initialize system clock & pins
void adc_start(void); // Initialize ADC subsystem
void usb_enable(void); // Enable USB
void adc_switch(void); // Switch ADC read pin
void wireless_send(void); // Send data to slave

/************************************************************
Global Variables
************************************************************/

int state = 0; // Used to control ADC switching
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
	adc_start();
	//usb_enable();

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
	
	m_clockdivide(4); // Set to 1 MHz
	
	m_bus_init(); // Enable mBUS
	
	m_rf_open(CHANNEL,RXADDRESS,PACKET_LENGTH); // Configure mRF
	
	
}


/* Setup ADC */
void adc_start(void){
	
	clear(ADMUX,REFS1); // Set reference voltage to Vcc
	set(ADMUX,REFS0);
	
	clear(ADCSRA,ADPS2); // Set prescaler to /8
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);
	
	set(DIDR2,ADC8D); // Disable Digital input to: ADC8
	set(DIDR2,ADC9D); // ADC9
	set(DIDR2,ADC10D); // ADC10
	
	sei(); // Enable global interrupts
	
	set(ADCSRA,ADIE); // Enable interrupt for when conversion is finished
	
	clear(ADCSRA,ADATE); // Turn off 'free-running' mode
	
	set(ADCSRB,MUX5); // Select ADC0 at pin D4
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	
	set(ADCSRA,ADEN); // Enable ADC subsystem
	
	set(ADCSRA,ADSC); // Begin first conversion
}

/* Setup USB */
void usb_enable(void)
{
	m_usb_init();
	while(!m_usb_isconnected());
}

/* Control ADC Pin Switching */
void adc_switch(void)
{
	clear(ADCSRA,ADEN); // Disable ADC subsystem
	
	switch(state){
		case 0:
			state = 1;
			set(ADCSRB,MUX5); // Select ADC0 at pin D6
			clear(ADMUX,MUX2);
			clear(ADMUX,MUX1);
			set(ADMUX,MUX0);
			Kp = ADC/4;
			break;
		case 1:
			state = 2;
			set(ADCSRB,MUX5); // Select ADC0 at pin D7
			clear(ADMUX,MUX2);
			set(ADMUX,MUX1);
			clear(ADMUX,MUX0);
			Ki = ADC/4;
			break;
		case 2:
			state = 0;
			set(ADCSRB,MUX5); // Select ADC0 at pin D4
			clear(ADMUX,MUX2);
			clear(ADMUX,MUX1);
			clear(ADMUX,MUX0);
			Kd = ADC/4;
			wireless_send();
			break;
	}
	
	set(ADCSRA,ADEN); // Enable ADC subsystem
	set(ADCSRA,ADSC); // Begin new conversion
}

/* Send Wireless Data */
void wireless_send(void)
{
	buffer[0] = Kp;
	buffer[1] = Ki;
	buffer[2] = Kd;
	
	m_rf_send(TXADDRESS,buffer,PACKET_LENGTH); // Send RF Signal
}

/************************************************************
Interrupts
************************************************************/

ISR(ADC_vect){
	adc_switch();
}

/************************************************************
End of Program
************************************************************/