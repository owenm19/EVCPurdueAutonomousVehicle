/************************************************************************************************************
*																											*
*	ATmega2560 in Arduino Mega																				*
*																											*
*	description					direction		type		special			Arduino			ATMEL			*
*																											*
*	angle_in					input			analog						A0				PORTF.0			*
*	speed_in					input			digital		ICP4			D49				PORTL.0			*
*																											*
*	Jetson_TX					input			UART		RXD1			D19				PORTD.2			*
*	Jetson_RX					output			UART		TXD1			D18				PORTD.3			*
*																											*
*	drive motor controller		output			PWM			OC3C			D3				PORTE.5			*
*	steering motor controller	output			PWM			OC3B			D2				PORTE.4			*
*	brake servo					output			PWM			OC3A			D5				PORTE.3			*
*																											*
************************************************************************************************************/

//program setup
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

//global constant definitions
#define PWM_MIN				1000	//defines minimum PWM pulse width in us
#define PWM_CENTER			1500	//defines center PWM pulse width in us
#define PWM_MAX				2000	//defines maximum PWM pulse width in us
#define STEERING_AGGRESSION	10		//defines rate at which steering motor approaches desired steering value
#define STEERING_MARGIN		80		//defines upper and lower limits of steering range

//global variable definitions
uint8_t desired_steering_in;														//FIXME: inputs and outputs are not yet in desired units and will require conversions
uint8_t desired_acceleration_in;			//FIXME:	speed
volatile uint8_t steering_sensor_in;		//FIXME:	steering_setpoint
uint8_t position;
volatile uint16_t drive_sensor_in;
uint16_t velocity;
uint16_t drive_control_out;
uint16_t steering_control_out;				//FIXME:	steering_actual
uint16_t brake_control_out;

//main function
int main(void)
{
	//general setup
	DDRD = 0b00001000;
	PORTD = 0b00000000;
	DDRE = 0b00111000;
	PORTE = 0b00000000;
	DDRF = 0b00000000;
	PORTF = 0b00000000;
	DDRL = 0b00000000;
	PORTL = 0b00000000;
	
	sei();	//enable global interrupts
	
	//ADC setup (steering potentiometer input)
	/****************************************************
	*													*
	*	REFS	= AVCC with external capacitor at AREF	*
	*	ADLAR	= left adjusted for 8-bit conversion	*
	*	MUX		= channel 0								*
	*	ADEN	= ADC enabled							*
	*	ADSC	= start conversion enabled				*
	*	ADATE	= auto trigger off						*
	*	ADIF	= interrupt flag false (not applicable)	*
	*	ADIE	= interrupt enabled						*
	*	ADPS	= prescaler = 128						*
	*													*
	****************************************************/
	ADMUX = 0b01100000;
	ADCSRA = 0b11001111;
	
	//timer 3 setup (drive motor controller output, steering motor controller output, brake servo output)
	/************************************************************
	*															*
	*	WGM3	= fast PWM mode 14								*
	*	COM3C	= clear OC3C on compare match					*
	*	COM3B	= clear OC3B on compare match					*
	*	COM3A	= clear OC3A on compare match					*
	*	ICNC3	= noise canceler off							*
	*	ICES3	= ICP3 negative edge trigger (not applicable)	*
	*	CS3		= prescaler 8									*
	*															*
	************************************************************/
	TCCR3A = 0b10101010;
	TCCR3B = 0b00011010;
	ICR3 = 40000;
	
	//timer 4 setup (RPM sensor input)
	/****************************************************
	*													*
	*	WGM4	= fast PWM mode 15						*
	*	COM4C	= normal port operation					*
	*	COM4B	= normal port operation					*
	*	COM4A	= normal port operation					*
	*	ICNC4	= noise canceler off					*
	*	ICES4	= ICP4 negative edge trigger			*
	*	CS4		= prescaler 8							*
	*	ICIE4	= input capture interrupt enable on		*
	*	OCIE4C	= output compare C interrupt enable off	*
	*	OCIE4B	= output compare B interrupt enable off	*
	*	OCIE4A	= output compare A interrupt enable off	*
	*	TOV4	= overflow interrupt enable off			*
	*													*
	****************************************************/
	TCCR4A = 0b00000011;
	TCCR4B = 0b00011010;
	TIMSK4 = 0b00100000;
	OCR4A = 0xFFFF;
	
	//UART setup
	//<>
	
	//loop
    while (1)
    {
		//throttle (not yet done)
		//some comparison between rpm sensor and desired acceleration, which should really be desired speed
		velocity = (56000 / drive_sensor_in);	//measured in m/s
		OCR3C = drive_control_out;
		
		//steering
		if (desired_steering_in < STEERING_MARGIN)																		//limit desired_steering_in to range
			desired_steering_in = STEERING_MARGIN;																		//
		if (desired_steering_in > (255 - STEERING_MARGIN))																//
			desired_steering_in = (255 - STEERING_MARGIN);																//
		steering_control_out = ((PWM_CENTER * 2) - (STEERING_AGGRESSION * (desired_steering_in - steering_sensor_in)));	//perform percent control to target value
		if (steering_control_out > (PWM_MAX * 2))																		//limit steering_control_out to range
			steering_control_out = (PWM_MAX * 2);																		//
		if (steering_control_out < (PWM_MIN * 2))																		//
			steering_control_out = (PWM_MIN * 2);																		//
		OCR3B = steering_control_out;																					//send value to OCR3B
		
		//brake (not yet done)
		if (desired_acceleration_in == 0)	//if desired acceleration is 0
			brake_control_out = 2000;			//fully extend brake actuator (FIXME: 4000)
		else								//else
			brake_control_out = 1000;			//fully retract brake actuator (FIXME: 4000)
		OCR3A = brake_control_out;			//send to OCR3A
    }
}

//ADC complete ISR (runs when ADIF in ADCSRA is set)
ISR (ADC_vect)
{
	if ((ADMUX & 0b00000001) == 0)	//if channel 0 is selected
	{
		steering_sensor_in = ADCH;		//store result in steering_sensor_in
		ADMUX |= 0b00000001;			//select channel 1
		ADCSRA |= 0b01000000;			//start conversion
	}
	else							//else if channel 1 is selected
	{
		desired_steering_in = ADCH;		//store result in desired_steering			FIXME: change to Jetson Nano, remove ADC multiplexer
		ADMUX &= 0b11111110;			//select channel 0							(desired_steering is temporarily controlled by ADC1, PORT and DDR are unchanged)
		ADCSRA |= 0b01000000;			//start conversion
	}
}

//timer 4 input capture ISR (runs when ICF4 in TIFR4 is set)
ISR (TIMER4_CAPT_vect)
{
	drive_sensor_in = ICR4;
	TCNT4 = 0;
}

//CTRL+F "FIXME" to find remaining errors