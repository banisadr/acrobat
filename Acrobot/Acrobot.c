/*
 * Acrobot.c
 *
 * Created: 10/29/2015 2:00:39 PM
 *  Author: Pete
 */ 
#define F_CPU 2700000000

#include <m_general.h>
#include "m_usb.h"

//Define global constants 
#define CLOCK_DIVIDE 0 
#define CLOCK_SPEED 16000000
#define PWM_FREQ 400
#define DUTY_CYCLE 0.4
#define INVERT 1 

void init(); //function prototyping 


int main(void)
{
	init();
	set(TCCR1B,CS10);
    while(1)
    {
        if(INVERT){
			set(PORTC,6);
		}
    }
}

void init() {
	m_usb_init(); //initialize USB communication 
	m_clockdivide(CLOCK_DIVIDE); //set clock divide 
	m_green(ON); 
	
	//Timer initialization
	clear(TCCR1B,CS12);	//start with timer1 off
	clear(TCCR1B,CS11);
	clear(TCCR1B,CS10);
	
	set(TCCR1B,WGM13);	//Use timer mode 15 (up to OCR1A, PWM mode)
	set(TCCR1B,WGM12);
	set(TCCR1A,WGM11);
	set(TCCR1A,WGM10);
	
	set(DDRB,6);	//enable digital output on pin B6
	
	set(TCCR1A,COM1B1);		//clear at OCR1B, set at OCR1A
	clear(TCCR1A,COM1B0);
	
	OCR1A = CLOCK_SPEED/PWM_FREQ;
	OCR1B = (float)OCR1A*DUTY_CYCLE;
	
	//IO initalization 
	set(DDRC,6);	//enable digital output on pin C6 (invert state)
	//set(DDRC,7);	//enable digital output on pin C7 
	clear(PORTC,6);	//start with both pins low 
	//clear(PORTC,7); 
	
}