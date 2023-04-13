/*
 * GccApplication2.cpp
 *
 * Created: 2022-11-22 11:28:26
 * Author : tmk22ce
 */ 
 #define F_CPU 1000000
#include <avr/interrupt.h>
#include <avr/io.h>


int initPWM(void);
int updatePWM(int);
int duty=100;
int AB;
int ABnew;
int P10;
int P11;

void MCUInit(void){
	DDRC |=(1<<PC2);
	DDRC |=(1<<PC3);
	//Enable pin change interrupt bits
	PCICR |=(1<<PCIE1);
	//Set PCINT10 bit in pin change mask register PCMSK0
	PCMSK1 |=(1<<PCINT10);
	PCMSK1 |=(1<<PCINT11);
	//Enable global interrupts
	sei();
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
ISR(PCINT1_vect)
{
	
	
	P10 =PINC & (1<<PCINT10);
	P11 =PINC & (1<<PCINT11);
	
	if (P10 & P11){
		ABnew=11;
	} else if(!P10 & P11){
		ABnew=01;
	}else if(!(P10 & P11)){
			ABnew=00;
	} else {
		ABnew=10;
	}
	//PINC =(1<<PC0);
	switch (ABnew){
		case 00 : if (AB==01)duty++; else /* AB==1 */ duty--; break;
		case 01 : if (AB==11)duty++; else /* AB==3 */ duty--; break;
		case 11 : if(AB==10)duty++;else/* AB==3 */n--;break;
		case 10: if(AB==00)duty++; else/*AB==*/ duty--;break;
		updatePWM(duty);
		AB=ABnew;
	//PINC =(1<<PC0);
}


int main(void)
{
    /* Replace with your application code */
	initPWM();
  MCUInit();
  while(1)
  {
	 DDRD = DDRD | (1<<DDD3)| (1<<DDD2);
	 PORTD = PORTD | (1<<PD3);
	 PORTD = PORTD | (1<<PD2);
	 int duty = 10;
	 initPWM();
	/* updatePWM(duty);
	 while(1)
	 {
		 _delay_ms(500);
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
		 updatePWM(duty);*/
	 }
  }
}

