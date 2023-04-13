/*
 * GccApplication23.c
 *
 * Created: 2023-04-13 11:26:16
 * Author : tmk22ce
 */ 

#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define FOSC 1843200 // Clock Speed
#define BAUD 2400
#define MYUBRR FOSC/16/BAUD-1 



volatile uint8_t duty = 255; //PWM-signal
volatile uint16_t interrupts; // interrupts from encoder
volatile int8_t AB; // local variable in encoder code

int16_t array[8] = {0,0,0,0,0,0,0,0}; // array to collect interrupts from encoder
volatile int8_t i = 0; // determines index of array 
volatile int16_t sum=0; // the sum of interrupts in array

int stop = 1; // in order to stop (put PWM to 255)

volatile uint16_t deltaRPM = 0; // the potentiometer's change in RPM

volatile uint16_t currentRPM = 0; // the current value of rpm

int timeBoolean =0; // a way to control the speed of the timer

int PIBoolean = 0; // a way to control the speed of the timer


#define NUM_FRACTION_BITS 6  // Number of fraction bits
int16_t Kp = (1 << NUM_FRACTION_BITS); // Proportional gain
int16_t Ki = (3 << NUM_FRACTION_BITS); // Integral gain

int16_t error_sum = 0; // integral for the PI-controller
volatile uint16_t targetValue = 0; // the value of RPM set by the user
uint16_t refOffset = 6;
uint16_t tempOffset =3;
int PI_control(uint16_t setpoint, uint16_t feedback);


int initPWM(void) { 
	DDRD |= (1<<DDD6);

	TCCR0A |= 0b10110011;
	TCCR0B |= 0x01;
	return 1;
}

int init_INTs(void)
{
	
	PORTC &= (1<<PC3)|(1<<PC2); // initializes pins for encoder code 
	PCMSK1 |= (1<<PCINT11)|(1<<PCINT10);   // PCINT10 and PCINT11  enabled (encoder)
	PCICR |= (1<<PCIE1);	// The PC interrupt group 1 (PCINT8 -> PCINT14) (encoder)
	
	PCMSK2 |= (1<<PCINT16); // sets up Rx for interrupts with USART
	
	//AB = PINC & ((1<<PINC3) | (1<<PINC2)); //sets value AB to help with the encoder code
	return 1;
}

int init_ADC(void){
	// Configure ADC to be left justified, use AVCC as reference, and select ADC4  ADC input
	ADMUX = 0b01100100;

	// Enable the ADC and set the prescaler to max value (128)
	ADCSRA = 0b10000111;
	return 1;
}


//controls timer1, timer1 interrupts when its time to enter number of interrupts into the array
int init_Timer1(void){ 
	TCCR1B |= (1 << CS11)| (1 << CS10);
	TCCR1B &= ~(1 << WGM13) & ~(1 << WGM12) & ~(1 << WGM11);
	OCR1A=1953;// This makes the interrupt trigger 8 times/sec. 1M/64/8array
	
	TIMSK1 |= (1 << OCIE1A); //Enable timer interrupts
	return 1;
}

//controls timer2, timer 2 interrupts when its time to use the PI-controller to change the duty
int init_Timer2(void){
	TCNT2 = 0x00;   /* Timer 2: Reset counter (periodic timer) */
	TCCR2B = 0x0f;   /* Clock / 1024, clear after compare match (CTC) */
	OCR2A = 144;     /* Set the compare value, corresponds to ~100 Hz */

	// Enable the compare match interrupt
	TIMSK2 = (1 << OCIE2A);
	return 1;
}

//what happens when timer2 interrupts
ISR(TIMER2_COMPA_vect) {
	cli();
	if(PIBoolean > 2){ // limits how often the pi is checked 
		
		PI_control(targetValue + deltaRPM , currentRPM); //uses the users set rpm + the impact of the potentiometer in comparison to actual value
		PIBoolean = 0;
		
	} else {
		PIBoolean++;
	}
	sei();
}
//what happens when timer1 interrupts
ISR(TIMER1_COMPA_vect) {
	
	cli();
	if (timeBoolean == 1) { // run every other time 
		
		sum = sum - array[i]; // deletes old value in place i of array
		array[i] = interrupts; // adds new value in array in place i
		sum = sum + array[i]; // adds new value in sum
		interrupts=0; // changes interrupts to 0 again to restart the count
		
		i++; // changes interrupts to make sure that the next value in array changes next time

		if(i > 8) { // if i is larger than size of array, restart with i = 0
			i=0;
		}
		
		timeBoolean = 0;
		} else {
		timeBoolean = 1;
	}
	sei();
}

//---------------------USART_BEGIN--------------------------------

//following methods sets up the USART to be able to receive and transmit information
void USART_Init(void)
{
	UBRR0H = (F_CPU /(BAUD * 8L) - 1) >> 8;
	UBRR0L = (unsigned char)(F_CPU / (BAUD * 8L)-1);
	UCSR0A|= (1 << U2X0);
	UCSR0C = (0<<USBS0)|(1<<UCSZ01)|(3<<UCSZ00);
	// Enable the USART Recieve Complete interrupt (USART_RXC)
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) ;
	UCSR0A |= (1 << RXC0);   //Datasheet says that needed for RXCIE0 works
}

//gets data from PC
unsigned char USART_Receive( void ){
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) ) {}
	/* Get and return received data from buffer */
	return UDR0;
}
//sends data to PC
void USART_Transmit( unsigned char data ){
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	{ }
	UDR0 = data; /* Put data into buffer, sends the data */
}
//clears buffer
void USART_Flush( void ){
	//Assumed to be ready to use, as described in the datasheet
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0))
	
	dummy = UDR0;
	
}

//when receiving something on the Rx- pin this interrupts. Handles all the data that is being sent between PC and AVR
ISR(USART_RX_vect)
{
	cli();
	unsigned char option=USART_Receive();
	
	// s is used to turn the motor off. when stop = 1 the PI-controller runs different code. (See Pi-controller)
	if(option =='s'){
		stop = 1;
		error_sum=0;// for antiwind up after stop
	}
	// b changes an int to 3 chars that are sent to the computer when requested to display current RPM
	else if(option == 'b'){
		
	uint16_t temp = (uint16_t) currentRPM; //temp is a dummy so that I don't have to change any value of the global variable currentRPM when sending data
	temp >>= NUM_FRACTION_BITS; //bit shifts value back so that the decimal points go away and the value can be read in normal way
	temp-=tempOffset;// changes the write out .change this according to motor 
	//following lines changes the value to chars
	char char1 = (temp/100) + '0'; // +'0' casts a char to int
	char char2 = (temp%100)/10 + '0';
	char char3 = (temp%10) + '0';
	
		//sends the chars to PC
		USART_Transmit(char1); 
		USART_Transmit(char2);
		USART_Transmit(char3);
		PORTD = PORTD |(1<<PD2);
		PORTD = PORTD | (1<<PD3);
	}
	//waits for three (or less) chars to transform into an int
	else if(option == 'c'){
		
		// receives data through USART
		char num1 = USART_Receive();
		char num2 = USART_Receive();
		char num3 = USART_Receive();
		
		stop = 0; //makes sure the PI-controller runs the correct code again
		
		targetValue =  (num1 - '0') * 100 + (num2 - '0') * 10 + ((num3) - '0'); // -'0' casts int to chars 
		if(targetValue>=100){
			refOffset=3;
			tempOffset=8;
		} else if (targetValue<100 && targetValue>30)
		{
			refOffset=6;
			tempOffset=3;
		}else {
			refOffset=8;
			tempOffset=1;
		}
		targetValue=targetValue-refOffset;// change the reference for differences in motors
		targetValue <<= NUM_FRACTION_BITS; // bit shifts value to be in same format as other values
	}
	
	
	USART_Flush();
	sei();
}

//---------------------USART_END--------------------------------

//uses timer0 to send PWM signal (8 bit-timer -> value between 0 and 255. Since PWM is inverted here max is 0 and min is 255
int updatePWM(uint8_t value)
{
	OCR0A = value; //sets the PWM to inverted mode
	
	return value;
}

//Interrupts every time a pin-change occurs on the encoder pins. The code is used to handle unwanted noisy signals from encoder. increases interrupts with 1 everytime
ISR(PCINT1_vect)
{
	/** Given the ISR routine has been triggered, we must have
	  received a new value.
	**/
	cli();
	//interrupts++;
	uint8_t newAB;
	
	
	newAB = PINC & ((1<<PC3) | (1<<PC2));	// Bits 2 and 3 are the ones with the sensor information



	uint8_t state_00 = (0<<PC3) | (0<<PC2);
	uint8_t state_01 = (0<<PC3) | (1<<PC2);
	uint8_t state_11 = (1<<PC3) | (1<<PC2);
	uint8_t state_10 = (1<<PC3) | (0<<PC2);


	switch (newAB){

	case (0<<PC3) | (0<<PC2) :
	
	if (AB==state_01){
	interrupts++;
	} else if (AB == state_10) { // AB==1 
	interrupts++;
	}
	break;
	
	case (0<<PC3) | (1<<PC2) :
	
	if (AB==state_11){
	interrupts++;
	} else if (AB == state_00) { // AB==3 
	interrupts++;
	}
	break;
	
	case (1<<PC3) | (1<<PC2) :
	
	if (AB==state_10){
	interrupts++;
	} else if (AB == state_01) { // AB==3 
	interrupts++;
	}
	break;
	
	case (1<<PC3) | (0<<PC2) :
	
	if (AB==state_00){
	interrupts++;
	} else if (AB == state_11) { // AB==3 
	interrupts++;
	}
	break;
	}
	// When the duty cycle has been altered, update the PWM signal.
	
	AB = newAB;
	
	sei();
	}

// method used for every division with fixed point (every division) ??	
uint16_t fixedPointDivision(uint16_t a, uint16_t b){
	uint32_t quotient = ((uint32_t)a << NUM_FRACTION_BITS)/b; // when multiplying two 16bits we need a 32bit int in order to bit shift again to remove the extra numfractionalbits
	return (uint16_t)quotient;
}

// method used for every multiplication with fixed point (every multiplication) ??			
uint16_t fixedPointMultiplication(uint16_t a, uint16_t b){
			
	uint16_t res;
	uint32_t temp = (uint32_t)a * b; //same reasoning as above
	temp = temp >> NUM_FRACTION_BITS;
	res =(uint16_t) temp;
	return res;
}

// method to calculate the current RPM. Is called from main and is done often
uint16_t RPM_calculations(void){
	
	// a calculation is done by hand that gives us that the RPM = sum * 0,62 which can be rewritten as sum*31/51
	// sum =ticks/sek
	// 60* sum = ticks / min
	// 96 ticks per revolution => 60/96*sum = rev/min 
	//60/96= 5/8
	uint16_t divisor = 8;
	divisor <<= NUM_FRACTION_BITS; // bit shift to match format of other values
	
	uint16_t multiplier = 5;
	multiplier <<= NUM_FRACTION_BITS; // bit shift to match format of other values
	
	
	uint16_t temp = sum; // it's safer to not change sum but to use a temp as a dummy instead
	temp <<= NUM_FRACTION_BITS; // bit shift to match format of other values
	
	uint16_t quotinent = fixedPointDivision(temp, divisor);
	uint16_t calculated_RPM = fixedPointMultiplication(quotinent, multiplier);
	
	
	
	
	return calculated_RPM;
	 
}

// PI_control determines if and how much the duty cycle should change 
int PI_control(uint16_t setpoint, uint16_t feedback) {
	
	int16_t error = (int16_t) setpoint -(int16_t) feedback; // The actual error between target value and current value in RPM
	if(stop==0){// dont add erros when stopped
		error_sum += error>>3; // the integrated part that is multiplied later in the calculation
	}
	
	
	if(error_sum > 2<<NUM_FRACTION_BITS) { // Avoid saturation of the integral part, multiplication with 2 and power to match format of other values
		
		error_sum =  2*2^NUM_FRACTION_BITS;
		
		} 
	else if (error_sum < -2<<NUM_FRACTION_BITS) {
		
		error_sum = - 2*2^NUM_FRACTION_BITS;
	}
	
	
	int16_t control = (int16_t) fixedPointMultiplication(Kp, error) + (int16_t) fixedPointMultiplication(Ki, error_sum); // the value that is used to alter duty
	
	// we don't want to change currentRPM so a temp is used
	uint16_t tempRPM = currentRPM;
	
	//is done to remove fractional bits so that comparisons can be done
	tempRPM  >>= NUM_FRACTION_BITS;
	control >>= NUM_FRACTION_BITS;
	setpoint >>= NUM_FRACTION_BITS;
	
	if(stop == 0){ // here comes the stop part. If stop isn't 1 we run the code as usual
		
		int16_t temp = (int16_t) duty- control; // checks the value before altering duty to make sure we never overflow 8bit int. signed to see if the value is negative
		
		// avoids saturation of duty
		if(temp < 0) { 
			duty = 0;
			} 
		else if(temp > 254) {
			duty = 254;
		}
		else{
			duty = duty - (uint8_t) control; // if "normal" value, just change duty as intended
		}
		updatePWM(duty);
	}
	else{ // if stop == 1 
		updatePWM(255);
		//targetValue=0;
	}
	
	
	
	return 0;
}

// converts the reading of voltage on a pin to an RPM	
int16_t calculate_adc(){
		
	ADCSRA = ADCSRA | (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	deltaRPM = fixedPointDivision(ADCH,12); // ADCH is 8bit -> 256/21 ( 21 steps) is almost 12
		
	return 0;
}

	int main(void)
	{
		//DDRD =  (1<<PD3)| (1<<PD2);
		//PORTD = PORTD | (1<<PD3);
		//PORTD = PORTD | (1<<PD2);
	sei();    // set global interrupt flag -> enable ALL interrupts (but those that are masked won't do anything)
	initPWM();
	init_INTs();
	USART_Init(); // (MYUBRR);
	init_Timer1();
	init_Timer2();
	updatePWM(duty);
	init_ADC();
	
	while (1)
	{
	currentRPM = RPM_calculations();
	calculate_adc();
	}
	return 0;
	}
