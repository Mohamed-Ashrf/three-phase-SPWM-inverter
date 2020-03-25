/*
 * variable_freq_3phase.c
 *
 * Created: 3/22/2020 1:20:48 PM
 * Author : Mohamed Ashrf
 */ 

#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BAUD 9600                               // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)        // set baud rate value for UBRR


// function to initialize UART

void uart_init ()
{
	UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
	UBRR0L = BAUDRATE;                           // set baud rate
	UCSR0B|= (1<<TXEN)|(1<<RXEN);                // enable receiver and transmitter
	UCSR0C|= (0<<UMSEL)|(1<<UCSZ0)|(1<<UCSZ1);   // 8bit data format
}
void uart_transmit (unsigned char data)
{	
	while (!( UCSR0A & (1<<UDR0)));               // wait while register is free
	UDR0 = data; 									  // load data in the register
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void adc_init()
{

	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
uint16_t adc_read(int channel)
{

	channel &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|channel;     // clears the bottom 3 bits before ORing
	
	// start single conversion
	// write '1' to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes '0' again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}


float mod_idx = .1;	
	
float universal_lookup[] = { 0.00 , 0.01 , 0.02 , 0.03 , 0.04 , 0.05 , 0.06 , 0.07 , 0.08 , 0.09 , 0.10 , 0.11 , 0.13 , 0.14 , 0.15 , 0.16 , 0.17 , 0.18 , 0.19 , 0.20 , 0.21 , 0.22 , 0.23 , 0.24 , 0.25 , 0.26 , 0.27 , 0.28 , 0.29 , 0.30 , 0.31 , 0.32 , 0.33 , 0.34 , 0.35 , 0.36 , 0.37 , 0.38 , 0.39 , 0.40 , 0.41 , 0.42 , 0.43 , 0.44 , 0.44 , 0.45 , 0.46 , 0.47 , 0.48 , 0.49 , 0.50 , 0.51 , 0.52 , 0.53 , 0.54 , 0.54 , 0.55 , 0.56 , 0.57 , 0.58 , 0.59 , 0.60 , 0.60 , 0.61 , 0.62 , 0.63 , 0.64 , 0.65 , 0.65 , 0.66 , 0.67 , 0.68 , 0.68 , 0.69 , 0.70 , 0.71 , 0.71 , 0.72 , 0.73 , 0.74 , 0.74 , 0.75 , 0.76 , 0.76 , 0.77 , 0.78 , 0.78 , 0.79 , 0.80 , 0.80 , 0.81 , 0.82 , 0.82 , 0.83 , 0.83 , 0.84 , 0.84 , 0.85 , 0.86 , 0.86 , 0.87 , 0.87 , 0.88 , 0.88 , 0.89 , 0.89 , 0.90 , 0.90 , 0.90 , 0.91 , 0.91 , 0.92 , 0.92 , 0.93 , 0.93 , 0.93 , 0.94 , 0.94 , 0.94 , 0.95 , 0.95 , 0.95 , 0.96 , 0.96 , 0.96 , 0.97 , 0.97 , 0.97 , 0.97 , 0.98 , 0.98 , 0.98 , 0.98 , 0.98 , 0.99 , 0.99 , 0.99 , 0.99 , 0.99 , 0.99 , 0.99 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 1.00 , 0.99 , 0.99 , 0.99 , 0.99 , 0.99 , 0.99 , 0.99 , 0.98 , 0.98 , 0.98 , 0.98 , 0.98 , 0.97 , 0.97 , 0.97 , 0.97 , 0.96 , 0.96 , 0.96 , 0.95 , 0.95 , 0.95 , 0.94 , 0.94 , 0.94 , 0.93 , 0.93 , 0.93 , 0.92 , 0.92 , 0.91 , 0.91 , 0.90 , 0.90 , 0.90 , 0.89 , 0.89 , 0.88 , 0.88 , 0.87 , 0.87 , 0.86 , 0.86 , 0.85 , 0.84 , 0.84 , 0.83 , 0.83 , 0.82 , 0.82 , 0.81 , 0.80 , 0.80 , 0.79 , 0.78 , 0.78 , 0.77 , 0.76 , 0.76 , 0.75 , 0.74 , 0.74 , 0.73 , 0.72 , 0.71 , 0.71 , 0.70 , 0.69 , 0.68 , 0.68 , 0.67 , 0.66 , 0.65 , 0.65 , 0.64 , 0.63 , 0.62 , 0.61 , 0.60 , 0.60 , 0.59 , 0.58 , 0.57 , 0.56 , 0.55 , 0.54 , 0.54 , 0.53 , 0.52 , 0.51 , 0.50 , 0.49 , 0.48 , 0.47 , 0.46 , 0.45 , 0.44 , 0.44 , 0.43 , 0.42 , 0.41 , 0.40 , 0.39 , 0.38 , 0.37 , 0.36 , 0.35 , 0.34 , 0.33 , 0.32 , 0.31 , 0.30 , 0.29 , 0.28 , 0.27 , 0.26 , 0.25 , 0.24 , 0.23 , 0.22 , 0.21 , 0.20 , 0.19 , 0.18 , 0.17 , 0.16 , 0.15 , 0.14 , 0.13 , 0.11 , 0.10 , 0.09 , 0.08 , 0.07 , 0.06 , 0.05 , 0.04 , 0.03 , 0.02 , 0.01 };

int steps = 300;
long int pot_value = 100;

uint16_t adc_result0, adc_result1;

int switchphaseA = 0;
int switchphaseB = 0;
int switchphaseC = 0;


int frequancy = 1;
long long int BASE_icr = 0;


long int counter = 300;
long int counterB = 100;
long int counterC = 200;


void action(){
	
	counter++;
	counterB++;
	counterC++;
	if( counter >= steps ){
		bit_flip(PORTD, BIT(0));
		counter = 0;
	}
	if( counterB >= steps ){
		bit_flip(PORTD, BIT(1));
		counterB = 0;
	}
	if( counterC >= steps ){
		bit_flip(PORTD, BIT(2));
		counterC = 0;
	}
}


int main(void){		



DDRD=0b00001111;  //setting PD0-PD3 as outputs

DDRE=0b00000010;
TCCR1A=0b10101010;   //TIMER1 Fast PWM non invert
TCCR1B=0b00011001;   //per-scaler 1
TIMSK=0b00000100;   //enabling overflow INT

bit_set(PORTD, BIT(2));   //phase increment

sei();


OCR1A  = 0;
OCR1B  = 0;
OCR1C  = 0;
while (1) 
{

	//mod_idx = (380-(7.52*(50-frequancy)))/380;

	BASE_icr = floor(((16e6)/(300*frequancy))-1);
	ICR1=BASE_icr/2;
	mod_idx = 1;
	mod_idx = ICR1*mod_idx;

	OCR1A= universal_lookup[counter]*ICR1;
	OCR1B= universal_lookup[counterB]*ICR1;
	OCR1C= universal_lookup[counterC]*ICR1;
}
}


ISR(TIMER1_OVF_vect){
	bit_flip(PORTD, BIT(3));
	action();
}
		

		  

