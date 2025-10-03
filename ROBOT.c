#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/serial.h"
#include "UART2.h"
#include <math.h>
#include <string.h>


#define F_CPU 32000000L
#define SYSCLK 32000000L
#define DEF_F 100000L

#define PIN_PERIOD (GPIOA->IDR&BIT8)
#define ECO_PIN (GPIOA->IDR&BIT12)

volatile int PWM_Counter = 0;
volatile unsigned char rightf_pwm=0, rightb_pwm=0, leftf_pwm=0, leftb_pwm=0;

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15 (Used for RXD of UART2, connects to TXD of JDY40)
//       PA3 -|9       24|- PA14 (Used for TXD of UART2, connects to RXD of JDY40)
//       PA4 -|10      23|- PA13 (Used for SET of JDY40)
//	 	 PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//pbuttonPA7 -|13      20|- PA10 (Reserved for RXD of UART1)
//       PB0 -|14      19|- PA9  (Reserved for TXD of UART1)
//       PB1 -|15      18|- PA8  
//       VSS -|16      17|- VDD
//             ----------

#define F_CPU 32000000L
// Uses SysTick to delay <us> micro-seconds. 
void Delay_us(unsigned char us)
{
	// For SysTick info check the STM32L0xxx Cortex-M0 programming manual page 85.
	SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}



// Interrupt service routines are the same as normal
// subroutines (or C funtions) in Cortex-M microcontrollers.
// The following should happen at a rate of 1kHz.
// The following function is associated with the TIM2 interrupt 
// via the interrupt vector table defined in startup.c
void TIM2_Handler(void) 
{
	TIM2->SR &= ~BIT0; // clear update interrupt flag
	PWM_Counter++;
	
	if(rightf_pwm>PWM_Counter)
	{
		GPIOA->ODR |= BIT1;
//		GPIOA->ODR |= BIT3;
	}
	else
	{
		GPIOA->ODR &= ~BIT1;
//		GPIOA->ODR &= ~BIT3;
	}
	
	if(leftf_pwm>PWM_Counter)
	{
//		GPIOA->ODR |= BIT1;
		GPIOA->ODR |= BIT3;
	}
	else
	{
//		GPIOA->ODR &= ~BIT1;
		GPIOA->ODR &= ~BIT3;
	}
	
	if(rightb_pwm>PWM_Counter)
	{
		GPIOA->ODR |= BIT4;
//		GPIOA->ODR |= BIT2;
	}
	else
	{
		GPIOA->ODR &= ~BIT4;
//		GPIOA->ODR &= ~BIT2;
	}
	
	if(leftb_pwm>PWM_Counter)
	{
//		GPIOA->ODR |= BIT4;
		GPIOA->ODR |= BIT2;
	}
	else
	{
//		GPIOA->ODR &= ~BIT4;
		GPIOA->ODR &= ~BIT2;
	}
	
	if (PWM_Counter > 255) // THe period is 20ms
	{
		PWM_Counter=0;
		GPIOA->ODR |= (BIT4|BIT1);
		GPIOA->ODR |= (BIT2|BIT3);
	}   
}

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Delay_us(250);
}

void waitus(uint32_t us)
{
    SysTick->LOAD = (F_CPU / 1000000L) * us - 1; // Set reload register for 'us' delay
    SysTick->VAL = 0; // Reset the SysTick counter value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick

    // Wait for the count flag to be set indicating the timer has expired
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0; // Disable SysTick
}

long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	return 0xffffff-SysTick->VAL;
}


long int Get_time (void)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (ECO_PIN==0) // Wait for square wave to be 1
	{
		if(SysTick->CTRL & BIT24 || SysTick->VAL < 0xf00000) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (ECO_PIN!=0) // Wait for square wave to be 0
	{
		if(SysTick->CTRL & BIT24 || SysTick->VAL < 0xf00000) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	return 0xffffff-SysTick->VAL;
}

void Hardware_Init(void)
{
	RCC->IOPENR |= BIT0 | BIT1;// peripheral clock enable for port A and B

	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);

	GPIOA->MODER &= ~(BIT14 | BIT15); // Make pin PA7 input
	// Activate pull up for pin PA8:
	
	GPIOA->MODER = (GPIOA->MODER & ~(BIT8|BIT9)) | BIT8; // Make pin PA4 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT4; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT2; // Make pin PA1 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT1; // Push-pull
	
	GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT6; // Make pin PA3 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT3; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT4; // Make pin PA2 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT2; // Push-pull
	
	GPIOA->MODER &= ~(BIT24 | BIT25); // Make pin PA12 input
	GPIOA->PUPDR |= BIT24; 
	GPIOA->PUPDR &= ~(BIT25);
	
	GPIOA->PUPDR |= BIT14; 
	GPIOA->PUPDR &= ~(BIT15);

	GPIOB->OSPEEDR=0xffffffff; // All pins of port B configured for very high speed! Page 201 of RM0451
	
	GPIOB->MODER = (GPIOB->MODER & ~(BIT9|BIT8)) | BIT8; // Make pin PB4 output, yellow left
    GPIOB->MODER = (GPIOB->MODER & ~(BIT11|BIT10)) | BIT10; // Make pin PB5 output, yellow right
   	GPIOB->MODER = (GPIOB->MODER & ~(BIT13|BIT12)) | BIT12; // Make pin PB6 output, red
   	GPIOB->MODER = (GPIOB->MODER & ~(BIT15|BIT14)) | BIT14; // Make pin PB7 output, white

	// Set up timer
	RCC->APB1ENR |= BIT0;  // turn on clock for timer2 (UM: page 177)
	TIM2->ARR = F_CPU/DEF_F-1;
	NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC
	TIM2->CR1 |= BIT4;      // Downcounting    
	TIM2->CR1 |= BIT7;      // ARPE enable    
	TIM2->DIER |= BIT0;     // enable update event (reload event) interrupt 
	TIM2->CR1 |= BIT0;      // enable counting    
	
	__enable_irq();
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

int main(void)
{
	const char intchars[] = "0123456789"; //used for converting ints to strings
	char x_voltage [40];
	char y_voltage [40];
	char temp_v [40];
	float temp_x;
	float temp_y;
	float y_volts;
    float x_volts;
	
    float y_power;
    float x_power;
    float power;
	
	char metal_reading [40];
	int metal = 555;
	int metal_digits = 3;
	int i = 0;
	
	long int Time;
	long int distance; 
	
	int auto_turning = 0;
	int auto_turning_counter = 0;
	int auto_driving = 0;
	
    int cnt=0;
    float temp;
    char buff[40];
    
    long int count;
	float T, f;
	float ref_freq = 0.0; 
	float freq_change = 0.0;

	char test [40];

	int led_control = 0;
	int blink_left = 0;
	int blink_right = 0;
	int blink_auto = 0;
	
	Hardware_Init();
	initUART2(9600);
	
    GPIOA->MODER = (GPIOA->MODER & ~(BIT23|BIT22)) | BIT22; // Make pin PA11 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->MODER &= ~(BIT25|BIT24); // Make pin PA12 input

	waitms(1000); // Give putty some time to start.
	printf("\r\nJDY-40 test\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVID6058\r\n");  

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID6058\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC119\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	printf("\r\nPress and hold a push-button attached to PA8 (pin 18) to transmit.\r\n");
	
	for(int time = 1; time <= 200; time++)
	{
	
		count=GetPeriod(200);
	
		
		if(count>0)
		{
				T=count/(F_CPU*200.0); // Since we have the time of 100 periods, we need to divide by 100
				f=1.0/T;
				 
		}
		else
		{
				printf("NO SIGNAL                     \r");
		}
		ref_freq = (ref_freq + f); 
		waitus(10); 
	}
	
	ref_freq = ref_freq / 200.0;
	
	cnt=0;
	
	while(1)
	{
		if((GPIOA->IDR & BIT7) == 0) // Check if PA7 is high
		{	
			printf("button pressed\n\r");
		   if(auto_driving == 1) auto_driving = 0; 
		   else auto_driving = 1; 
		   while((GPIOA->IDR & BIT7) == 0); 
		}


		if(ReceivedBytes2()>0) // Something has arrived
		{
			
				//Recieve the Y and X volatges as string from controller
				egets2(y_voltage, sizeof(y_voltage)-1);
				egets2(x_voltage, sizeof(x_voltage)-1);
				
				//Prints voltage values
				//printf("Y%s\n\rX%s\n\r",y_voltage,x_voltage);
				
				//Swaps the strings if X and Y get mixed up
				if(y_voltage[0] != 'Y')
				{
					for(int i = 0;i < 39; i++)
					{
						temp_v[i] = y_voltage[i];
						y_voltage[i] = x_voltage[i];
						x_voltage[i] = temp_v[i];
					}
				}
						
				//Tests if there is an M requesting the metal value
				if(y_voltage[1] == 'M')
				{
					
					//Filter 1: Ignores any strings that are not the correct lengths
					if((strlen(x_voltage) == 8) && (strlen(y_voltage) == 8))
					{
						
						//Temp variables to test if the values are in the range we are looking for
						temp_x = atof(x_voltage+1);
						temp_y = atof(y_voltage+2);
						//temp_x = atof(x_voltage);
						//temp_y = atof(y_voltage+1);
						
						// Filter 2: Ignores any temp values that are not in the correct range
						if(((temp_x<=3.4) && (temp_x>=0)) && ((temp_y<=3.4) && (temp_y>=0)))
						{	
							//If the metal reading is requested		
							//printf("M Recieved");
							
							//printf("Y%s\n\rX%s\n\r",y_voltage,x_voltage);
							
							//Sets the actual voltage values after all the filtering is done
							x_volts = atof(x_voltage+1);
							y_volts = atof(y_voltage + 2);
							
							__disable_irq();
							count=GetPeriod(50);
							__enable_irq();
							
							if(count>0)
							{
								
								T=count/(F_CPU*50.0); // Since we have the time of 100 periods, we need to divide by 100
								f=1.0/T;
								freq_change = ref_freq - f; 
								if(freq_change < 0){
									freq_change = freq_change * -1.0; 
									}
								else freq_change = freq_change; 
								
								if(freq_change >= 600.0){
									//sprintf(test, "metal detected! freq_change=%.3fHz ref_freq=%.3fHz\r", freq_change, ref_freq);
									//printf("freq_change=%i Hz ref_freq=%i Hz freq=%i metal detected!          \r\n", (int)freq_change, (int)ref_freq, (int)f);
									//printf(test); 
								}
								else 
								{
								    //sprintf(test, "freq_change=%.3fHz ref_freq=%.3fHz\r", freq_change, ref_freq);
									//printf(test);
									//printf("freq_change=%i Hz ref_freq=%i Hz freq=%i                           \r\n", (int)freq_change, (int)ref_freq, (int)f);
								}
										
							}
							else
							{
								printf("NO SIGNAL                     \r");
							}
							
							//prints metal reading into a string
							metal_digits = 4;
							metal = freq_change;
							i = 37; //length of metal_reading -1
							
							while((metal > 0) | (metal_digits > 0)){
								metal_reading[i--] = intchars[metal%10];
								metal/=10;
								if(metal_digits != 0) metal_digits--;
							}
							metal_reading[38] = '\n';
							metal_reading[39] = '\0';
							
							//Sends the metal reading
							//waitms(10);
							//printf(&metal_reading[i+1]);
							//printf("%i", strlen(&metal_reading[i+1]));
							eputs2(metal_reading +i+1);
							//sprintf(metal_reading, "%i\n", metal);
							//eputs2(metal_reading);
							//waitms(10);
							//printf("Metal Sent");
							
						}
					}
					else
					{
						//printf("filtered STRING M\r\n");
						//printf("X:%d Y:%d\n\r", strlen(x_voltage), strlen(y_voltage));
					}
					if(auto_driving == 1) 
					{
						GPIOA->ODR ^= BIT11; // Toggle PA11
						waitus(10);
						GPIOA->ODR ^= BIT11; // Toggle PA11
						waitus(10);
						printf("before\r\n");
						//__disable_irq();
						Time = Get_time()/((F_CPU/1000000.0));
						//__enable_irq();
						if(Time > 0) distance = Time / 58.0; //if the measurement works
						else printf("DISTANCE ERROR\n\r");
						printf("\ntime = %i  distance = %i \r\n", Time, distance);  
						if(auto_turning ==1)
						{
							auto_turning_counter++;
							if(auto_turning_counter == 9)
							{
								auto_turning = 0;
								auto_turning_counter = 0;
							}
						}
						
					}
				}
				else
				{
					if(auto_driving == 1)
					{
						//printf("driving loop\r\n");
						
						led_control = 5;
						
						if(freq_change <= 600)
						{
							if(auto_turning != 1)
							{
								if(distance <= 13) //turn 90 degrees
								{
									auto_turning = 1; 
									leftf_pwm = 255;
									leftb_pwm = 0;
							    	rightf_pwm = 0;
							    	rightb_pwm = 0;
							    	//printf("random value: %i\n\r", SysTick->VAL%2);
						  
								}
								else //go forward
								{
									printf("going forward \r\n");
									leftf_pwm = 255;
									leftb_pwm = 0;
						    		rightf_pwm = 255*0.91;
						    		rightb_pwm = 0;							
								}
							}
						}
					
						else{
							auto_driving = 0;
							leftf_pwm = 0; 
							leftb_pwm = 0;
							rightf_pwm = 0; 
							rightb_pwm = 0;
							//auto_driving = 0; 
						}
					}
					
					else if(strlen(x_voltage) == 8) 
					{
						temp_x = atof(x_voltage+1);
						
						if((temp_x<=3.4) && (temp_x>=0)) 
						{
							x_volts = temp_x;
							//printf("X:%s\n\r", x_voltage);	
						}
					}
					else
					{
						//printf("FILTERED X");
					}
					
					if(strlen(y_voltage) == 7) 
					{
						temp_y = atof(y_voltage+1);
							if((temp_y<=3.4) && (temp_y>=0)) 
							{
								y_volts = temp_y;
								//printf("Y:%s\n\r", y_voltage);
							}
					}
					else
					{
						//printf("FILTERED Y");
					}
				
				}
				if(auto_driving == 0)
				{
					//Calculate the power we are sending to the wheels
					y_power = (y_volts-1.65)*154.54;
					x_power = (x_volts-1.65)*154.54;
					
					//If statement to always set the power to the highest of the two values
					if(abs(x_power) <= abs(y_power))
					{
						power = abs(y_power);
					}
					else
					{
						power = abs(x_power);
					}
					
					
					//Test if we are trying to move left
					if (x_volts < 1.60) 
					{
				    	//Test if we are going forward or backwards
				    	if(y_power >= -0.15)
				    	{
				    		leftb_pwm = 0;
			        		rightb_pwm = 0;
				    		rightf_pwm = power;
					    	leftf_pwm = (x_volts * (power) / 1.65);
					    
					    	led_control = 3; 
					}
					else
					{
						leftf_pwm = 0;
			        		rightf_pwm = 0;
						rightb_pwm = power;
					    leftb_pwm = (x_volts * (power) / 1.65); 
					    
					    led_control = 2;
					}
				    }
				    //Test if we are moving to the right
				    else if ( x_volts > 1.63 ) 
				    {
				        //Test if we are going forward or backwards
				        if(y_power >= -0.15)
				        {
				        	leftb_pwm = 0;
			        	rightb_pwm = 0;
				        leftf_pwm = power;
					    rightf_pwm = (power - ((x_volts-1.65) * (power) / 1.65));
					    
					    led_control = 4;
						}
						else
						{
							leftf_pwm = 0;
			        			rightf_pwm = 0;
							leftb_pwm = power;
					   		rightb_pwm = power - ((x_volts-1.65) * (power) / 1.65); 
					    
					   		led_control = 2;
						}
				    }
				    //Test to see if the thumbstick is in the middle
				    else if ((1.60 <= x_volts && x_volts <= 1.63) && (1.62 <= y_volts && y_volts <= 1.67))
				    {
				        // Thumbstick is in the middle, stop both motors
				        leftf_pwm = 0;
			        	rightf_pwm = 0;
			        	leftb_pwm = 0;
			       		rightb_pwm = 0;
			        
			        	led_control = 0;
				    }
				    //Else move fowards or backwards
				    else 
				    {
				    	//Test if we are going forward or backwards
				    	if(y_power >= -0.15)
				        {
				        	leftb_pwm = 0;
			        		rightb_pwm = 0;
				    		leftf_pwm = power;
				    		rightf_pwm = power*0.91;
				    	
				    		led_control = 1;
					    }
					    else
					    {
					    	leftf_pwm = 0;
			        		rightf_pwm = 0;
				    		leftb_pwm = power;
				    		rightb_pwm = power*0.91;
				    	
				    		led_control = 2;
					    }
				    }
			    }
			
		}
			   //printf("\rHIGH, %i, %i, \n", leftf_pwm, rightf_pwm);
			   //printf("LOW, %i, %i, \n", leftb_pwm, rightb_pwm);
			   
			//PB4 yellow left
    		//PB5 yellow right
    		//PB6 red
    		//PB7 white
		
			//stopped
			if(led_control == 0)
			{
				GPIOB->ODR &= ~(BIT6);//Turn On Red
				
				GPIOB->ODR &= ~(BIT4);//Turn On Yellow Left
				GPIOB->ODR &= ~(BIT5);//Turn On Yellow Right
				GPIOB->ODR |= BIT7; //Turn Off White
			}
			//forward
			else if(led_control == 1)
			{
				GPIOB->ODR &= ~(BIT4);//Turn On Yellow Left
				GPIOB->ODR &= ~(BIT5);//Turn On Yellow Right
				
				GPIOB->ODR |= BIT6; //Turn Off Red
				GPIOB->ODR |= BIT7; //Turn Off White
			}
			//backward
			else if(led_control == 2)
			{
				GPIOB->ODR &= ~(BIT7);//Turn On White
				
				GPIOB->ODR &= ~(BIT4);//Turn On Yellow Left
				GPIOB->ODR &= ~(BIT5);//Turn On Yellow Right
				GPIOB->ODR |= BIT6; //Turn Off Red
			}
			//left
			else if(led_control == 3)
			{				
				blink_left++;
				
				if(blink_left == 10000)
				{
					GPIOB->ODR ^= BIT4; // Toggle PB4, Yellow Left
					blink_left = 0;
				}
				
				GPIOB->ODR &= ~(BIT5);//Turn On Yellow Right
				GPIOB->ODR |= BIT7; //Turn Off White
				GPIOB->ODR |= BIT6; //Turn Off Red
			}
			//right
			else if(led_control == 4)
			{
				blink_right++;
				
				if(blink_right == 10000)
				{
					GPIOB->ODR ^= BIT5; // Toggle PB5, Yellow Right
					blink_right = 0;
				}
				
				GPIOB->ODR &= ~(BIT4);//Turn On Yellow Left
				GPIOB->ODR |= BIT7; //Turn Off White
				GPIOB->ODR |= BIT6; //Turn Off Red
			}
			//auto drive
			else if(led_control == 5)
			{
				blink_auto++;
				
				if(blink_auto == 10000)
				{
					GPIOB->ODR ^= BIT4; // Toggle Yellow Left
					GPIOB->ODR ^= BIT5; // Toggle Yellow Right
					GPIOB->ODR ^= BIT6; // Toggle Red
					GPIOB->ODR ^= BIT7;	// Toggle White
					
					blink_auto = 0;
				}
			}
		
	}

}