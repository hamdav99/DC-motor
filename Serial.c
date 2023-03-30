#include "serialport.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE 2400

int main(void)
{
	/*Declaration of variables*/
	int sp,sl;
	char in1;
	char in2;
	char in3;
	char cout;
	char num1;
	char num2;
	char num3;
	int digit;
	
	/*Initialise serial port */
	sp = serial_init("/dev/ttyS0",0);
	if(sp == 0)
	{
		printf("Error! Serial port could not be opened.\n");
	}
	else
	{
		printf("Serial port open with identifier %d \n",sp);
	}
	
	/*Initialise both strings - they are initially different! */
	/*The max size of the strings is 6 characters (by declaration)*/
	/*Remember to leave space for the termination character!*/

	
	/*Verify that the strings were successfully initialised... */
	
	
	while(1)
	{
		printf("\n");
		printf("Choose option: \n");
		printf("Enter 'c' to change RPM \n");
		printf("Enter 's' to stop motor \n");
		printf("Enter 'b' to receive current speed \n");
		
		// just to make sure we won't miss any input
		sleep(1);
		
		scanf("%c", &cout);
		
		
		
		if(cout == 'b'){
			//sends b to AVR
			write(sp, &cout,1);
			
			/*Read the incoming string */
			read(sp,&in1,1);
			read(sp,&in2,1);
			read(sp,&in3,1);
			
			//print in terminal as chars
			//printf("RPM = %i",receive_int());
			printf("\n");
			if(in1!='0'){
				printf("%c", in1);
			}
			
			printf("%c", in2);
			printf("%c \n", in3);
		}
		else if(cout == 'c'){
			//sends c to AVR
			write(sp, &cout,1);
			
			printf("Enter wanted RPM \n");

			//these lines might be easier to do as chars directly but this works so im keeping it like this
			scanf("%d", &digit);
			
			num1 = digit / 100 + '0';
			num2 = (digit % 100) / 10 + '0';
			num3 = (digit % 10) + '0';
			
			//sends 3 chars to be handled by the AVR
			write(sp, &num1,1);
			write(sp, &num2,1);
			write(sp, &num3,1);
			
			
		}
		else if(cout == 's'){
			//sends s to AVR to turn motor off
			write(sp, &cout,1);
			printf("Motor shutting down \n"); // prints even though something went wrong but o well. Can be easily implemented by sending something back from AVR
											// when signal s was received
		}
		else{
			printf("Please enter valid option \n \n");
		}
		
		
		scanf("%c", &cout); // sometimes the serial port reads empty value and runes code 2 times. With this we just scan an empty value anyway without changing anything
	}
	/*Close the serial port */
	serial_cleanup(sp);
	return 1;
}