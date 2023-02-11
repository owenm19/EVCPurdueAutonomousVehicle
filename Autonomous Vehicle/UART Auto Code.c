#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

#define BAUD				115200  //defines baud rate 

//make sure all settings (including BAUD RATE) are set
//add transmit function
//write main function to initialize UART, then figure out a way to poll for receiving 
//and transmitting

uint8_t USART_Init(void)
{
	UCSR0A |= 0b00100000; 
	UCSR0B |= 0b00000000; 
	UCSR0C |= 0b00000110; 
}	

uint8_t USART_Receive(void)
{
	uint8_t status;
	while ((UCSR0A & (0b00100000)) == 1);
	status = UCSR0A;
	return status;
}

int main(void)
{
	
}