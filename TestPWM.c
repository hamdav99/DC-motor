/*
 * TestPWM.c
 *
 * Created: 16/10/2019 22:11:10
 *  Author: FranM
 */ 

#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>

int initPWM(void);
int updatePWM(int);

int main(void)
{
	DDRD =  (1<<PD3)| (1<<PD2);
	PORTD = PORTD | (1<<PD3);
	PORTD = PORTD | (1<<PD2);
	int duty = 10;
	initPWM();
	updatePWM(duty);
	while(1)
    {
			/*PORTD = PORTD | (1<<PD3);
			PORTD = PORTD | (1<<PD2);*/
			_delay_ms(500);
			//PORTD = PORTD & ~(1<<PD2); 
			
			
       /_delay_ms(500);
		duty+=10;
		if (duty>255)
		{
			duty = 10;
		}
		if(duty>126){
			PORTD = PORTD & ~(1<<PD2); 
			PORTD = PORTD | (1<<PD3);
		} else {
			PORTD = PORTD & ~(1<<PD3);
			PORTD = PORTD | (1<<PD2);
		}
		updatePWM(duty); 
    }
}

int initPWM(void)
{
	DDRD |= (1<<DDD6);	//Set PIND5 and PIND6 as outputs
	TCCR0A |= 0b10110011;			//Configure fast PWM mode, non-inverted output on OCA and inverted output on OCB
	TCCR0B |= 0x01;					//Internal clock selector, no prescaler
	return 1;
}

int updatePWM(int value)
{
	OCR0A = value;
	//OCR0B = value;
	OCR0B = 125;
	return value;
}