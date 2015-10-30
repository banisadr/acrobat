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
Modified by Pete on 10/30/15 to add angle output
Modified by Bahram on 10/30/15 to integrate motor control
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

/* Clock Values */
#define CLOCK_DIVIDE 3
#define CLOCK 2000000
#define TIM3_PRESCALE 1
#define TIMESTEP 0.001

/* Filter Values */
#define ALPHA_LOW 0.95
#define ALPHA_HIGH 0.1
#define AX_OFFSET -136
#define AZ_OFFSET 39
#define GY_OFFSET -121

/* Motor Driver Values */
#define PWM_FREQ 400 

/* PID Controller Values */
#define SETPOINT 0
#define Kp 1.0
#define Ki 1.0
#define Kd 1.0

/* Wireless Communication Values */
#define CHANNEL 7
#define RXADDRESS 0x7C
#define PACKET_LENGTH 3

/* Other */
#define RAD2DEG 57.30

/************************************************************
Prototype Functions
************************************************************/

void init(void); //Setup I/O, clockspeed, IMU, Interrupts
void print_axazgy(void); //Print values to usb
void print_all(void); //Print all 6 IMU values
void print_angle(int angle); //Print angle
void usb_enable(void); //Setup USB
void wireless_enable(void); // Initialize the wireless system
void wireless_recieve(void); // Send data to slave
void timer1_init(void); //Setup timer1 for motor PWM control
void timer3_init(void); //Setup timer3 for fixed timestep calculations
void update_angle(void); //Get IMU data, filter, update angle, update control
void run_control_loop(void); //Run Control Loop
int lowpass(float alpha, int previous_output, int reading); //Lowpass filter
int highpass(float alpha, int previous_output, int previous_reading, int reading); //Highpass filter


/************************************************************
Global Variables
************************************************************/

/* IMU Shenanigans */
unsigned char accel_scale = 1; // +/-1g
unsigned char gyro_scale = 1;// +/- 125 degrees
int data[9]={0}; // IMU data buffer
int ax = 0;
int az = 0;
int gy = 0;

/* Motor Control */
float duty_cycle = 0.4;

/* Angle Stuff */
int gy_previous_reading = 0;
int angleFast = 0;
int angleSlow;
int angle=0;

/* PID Controler Values */
int previous_error = 0;
float integral = 0.0;

/* Wireless Communciation Values */
char buffer[PACKET_LENGTH] = {0,0,0}; // Wifi output
char Kp_adjust = 0; // Proportional Gain
char Ki_adjust = 0; // Integral Gain
char Kd_adjust = 0; // Derivative Gain

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
	timer1_init();
	timer3_init();

	/* Confirm successful initialization(s) */
	m_green(ON);

	/* Run */
	while (1){
		update_angle();
	}
}

/************************************************************
Initialization of Subsystem Components
************************************************************/

/* Initialization of Pins and System Clock */
void init(void){
	
	m_clockdivide(CLOCK_DIVIDE); // Set to 2 MHz
	
	//Set to Input
	clear(DDRD,0); // D0
	clear(DDRD,1); // D1
	clear(DDRD,2); // D2
	
	//Set to Output
	set(DDRB,6); // B6
	set(DDRC,6);	//enable digital output on pin C6 (invert state)
	
	//Set pin low
	clear(PORTC,6);	//start with both pins low 
	
	while(!m_imu_init(accel_scale,gyro_scale)); //Initialize IMU
	
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
	sei();
	m_bus_init(); // Enable mBUS
	m_rf_open(CHANNEL,RXADDRESS,PACKET_LENGTH); // Configure mRF
}

/* Recieve Wireless Data */
void wireless_recieve(void)
{
	m_rf_read(buffer,PACKET_LENGTH); // Read RF Signal
	Kp_adjust = buffer[0];
	Ki_adjust = buffer[1];
	Kd_adjust = buffer[2];
	
	/*	
	m_usb_tx_string("Kp= ");
	m_usb_tx_int(Kp);
	m_usb_tx_string("     Ki= ");
	m_usb_tx_int(Ki);
	m_usb_tx_string("     Kd= ");
	m_usb_tx_int(Kd);
	m_usb_tx_string("\n");
	*/
}

/* Timer1 Initialization for PWM Motor Control */
void timer1_init(void)
{
	//Timer initialization
	clear(TCCR1B,CS12);	//Set timer1 prescaler to /1
	clear(TCCR1B,CS11);
	set(TCCR1B,CS10);
	
	set(TCCR1B,WGM13);	//Use timer mode 15 (up to OCR1A, PWM mode)
	set(TCCR1B,WGM12);
	set(TCCR1A,WGM11);
	set(TCCR1A,WGM10);

	set(TCCR1A,COM1B1);		//clear at OCR1B, set at OCR1A
	clear(TCCR1A,COM1B0);

	OCR1A = CLOCK/PWM_FREQ;
	OCR1B = (float)OCR1A*duty_cycle;
}

/* Timer3 Initialization for fixed timestep calculations */
void timer3_init(void) 
{
	clear(TCCR3B,CS32); // prescale /1
	clear(TCCR3B,CS31);
	set(TCCR3B,CS30);

	clear(TCCR3B,WGM33); // Up to OCR3A (mode 4)
	set(TCCR3B,WGM32);
	clear(TCCR3A,WGM31);
	clear(TCCR3A,WGM30);
	
	OCR3A = TIMESTEP*(CLOCK/TIM3_PRESCALE); // initalize OCR3A or duration
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

/* Get IMU data, filter, update angle, update control */
void update_angle(void)
{
	if (m_imu_raw(data))
	{
		m_green(ON);
		m_red(OFF);		
		
		ax = lowpass(0.7,ax,data[0])+AX_OFFSET;
		az = lowpass(0.7,az,data[2])+AZ_OFFSET;
		gy = lowpass(ALPHA_LOW,gy,data[4])+GY_OFFSET;
		gy = highpass(ALPHA_HIGH,gy,gy_previous_reading,data[4]);
		gy_previous_reading = data[4];
		
		if (check(TIFR3,OCF3A)){	//check if timestep has completed
			angleSlow = ((float)ax*RAD2DEG)/sqrt(((float)ax*ax+(float)az*az));
			angleFast += gy*TIMESTEP;	//add thetadot*timestep to angle
			angle = -angleSlow + angleFast;
			set(TIFR3,OCF3A);		//reset flag
			run_control_loop(); // Update control
		}
		
		print_angle(angle);
	}
	else
	{
		m_green(OFF);
		m_red(ON);
	}
}

/* PID Control */
void run_control_loop(void)
{
	int error = SETPOINT - angle;
	integral += error*TIMESTEP;
	float derivative = (error - previous_error)/TIMESTEP;
	float output = (Kp_adjust/255.0)*Kp*error + (Ki_adjust/255.0)*Ki*integral + (Kd_adjust/255.0)*Kd*derivative;
	previous_error = error;
	
	duty_cycle = abs(output)/(45.0*Kp);
	
	if (output<0){set(PORTC,6);}
	else{clear(PORTC,6);}
}

/* Print Functions for Funzies */
void print_axazgy(void) //Print values to usb
{
	m_usb_tx_string("ax= ");
	m_usb_tx_int(ax);
	m_usb_tx_string("     az=");
	m_usb_tx_int(az);
	m_usb_tx_string("     gy=");
	m_usb_tx_long(gy);
	m_usb_tx_string("\n");
}

void print_all(void)//Print all 6 IMU values
{
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
}

void print_angle(int angle)//Print angle
{
	m_usb_tx_string("angle= ");
	m_usb_tx_int(angle);
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