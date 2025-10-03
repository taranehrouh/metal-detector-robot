#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF 
// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 16000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define BASE_FREQ 2048L // 2Hz or 0.5 seconds interrupt rate
//LCD defines
#define CHARS_PER_LINE 16
#define LCD_D7_ENABLE TRISAbits.TRISA4
#define LCD_D6_ENABLE TRISBbits.TRISB4   
#define LCD_D5_ENABLE TRISAbits.TRISA3   
#define LCD_D4_ENABLE TRISAbits.TRISA2  
#define LCD_E_ENABLE  TRISBbits.TRISB3   
#define LCD_RS_ENABLE TRISAbits.TRISA0
#define LCD_RS LATAbits.LATA0
#define LCD_E LATBbits.LATB3
#define LCD_D4 LATAbits.LATA2
#define LCD_D5 LATAbits.LATA3
#define LCD_D6 LATBbits.LATB4
#define LCD_D7 LATAbits.LATA4

//FREQUENCY CHANGE REF SO WE CAN FIND IT
#define FCHANGE_REF 600


void wait_1us(void);
void delayus(int);
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX
    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	LATAbits.LATA1 = !LATAbits.LATA1; // Blink led on RB6/SPEAKER ON RA1
	IFS0CLR = _IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
}

void SetupTimer1 (int FREQ)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/(FREQ*256))-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 3; // Pre-scaler: 256
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}
// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = c;          // echo
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}
void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET=0x8000;      // Enable ADC
}
int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;        // Begin sampling
    while(AD1CON1bits.SAMP);     // wait until acquisition is done
    while(!AD1CON1bits.DONE);    // wait until conversion done
 
    return ADC1BUF0;             // result stored in ADC1BUF0
}
/////////////////////////////////////////////////////////
// UART1 functions used to communicate with the JDY40  //
/////////////////////////////////////////////////////////
// TXD1 is in pin 26  aka RXD aka RB15
// RXD1 is in pin 24  aka TXD aka RB13
int UART1Configure(int desired_baud)
{
	int actual_baud;
    // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
    // 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2
	// Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
    // priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
    // TRISx register must also be configured for input (set to ‘1’)."
    
    ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB13 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
    U1RXRbits.U1RXR = 3; // SET U1RX to RB13
    // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
    // RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7
    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
	
    U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U1STA = 0x1400;     // enable TX and RX
    U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    actual_baud = SYSCLK / (16 * (U1BRG+1));
    U1MODESET = 0x8000;     // enable UART1
    return actual_baud;
}
 
int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U1STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 	int timeout_counter;
 
    while(num_char < max_size)
    {
    	timeout_counter=0;
        while( !U1STAbits.URXDA)
        {
        	delayus(100);
        	timeout_counter++;
        	if(timeout_counter==1000)
        	{
        		printf("TIMEOUT");
        		*buffer = '\0';
        		return num_char;
        	}
        }   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}
// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count
    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}
void delayms(int len)
{
	while(len--) wait_1ms();
}
void wait_1us(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count
    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000000)) );
}
void delayus(int len)
{
	while(len--) wait_1us();
}
//LCD CODE//
void LCD_pulse(void)
{
	LCD_E = 1;
	delayus(40);
	LCD_E = 0;
}
void LCD_byte(unsigned char x)
{
	LCD_D7=(x&0x80)?1:0;
	LCD_D6=(x&0x40)?1:0;
	LCD_D5=(x&0x20)?1:0;
	LCD_D4=(x&0x10)?1:0;
	LCD_pulse();
	delayus(40);
	LCD_D7=(x&0x08)?1:0;
	LCD_D6=(x&0x04)?1:0;
	LCD_D5=(x&0x02)?1:0;
	LCD_D4=(x&0x01)?1:0;
	LCD_pulse();
}
void WriteData(unsigned char x)
{
	LCD_RS = 1;
	LCD_byte(x);
	delayus(10);
}
void WriteCommand(unsigned char x)
{
	LCD_RS = 0;
	LCD_byte(x);
	delayus(10);
}
void LCD_4BIT(void)
{
	// Configure the pins used to communicate with the LCD as outputs
	LCD_RS_ENABLE = 0;
	LCD_E_ENABLE = 0;
	LCD_D4_ENABLE = 0;
	LCD_D5_ENABLE = 0;
	LCD_D6_ENABLE = 0;
	LCD_D7_ENABLE = 0;
	
	LCD_E = 0; // Resting state of LCD's enable is zero
	// LCD_RW = 0; Not used in this code.  Connect to ground.
	delayms(20);
	// First make sure the LCD is in 8-bit mdode, then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode
	
	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	delayms(20); // Wait for clear screen command to finish
	LATBbits.LATB0 = 	!LATBbits.LATB0;
}
void LCDprint(char * string, unsigned char line, unsigned char clear)
{
	int j;
	
	WriteCommand(line==2?0xc0:0x80);
	delayus(10);
	for(j=0;string[j]!=0;j++)
		WriteData(string[j]); //Write the message character by character
	if(clear)
		for(;j<CHARS_PER_LINE;j++)
			WriteData(' '); //Clear the rest of the line if clear is 1
}
void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1(s);
	SerialReceive1(buff, sizeof(buff)-1);
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	delayms(10);
	printf("Response: %s\n", buff);
}

void DefineCustomCharacter(void)
{
   
    WriteCommand(0x40); // Set CGRAM address to 0
    WriteData(0b11111);
    WriteData(0b11111);
    WriteData(0b11111);
    WriteData(0b11111);
    WriteData(0b11111);
    WriteData(0b11111);
    WriteData(0b11111);
    WriteData(0b11111);
    
      
  
    // Return to home (DDRAM address 0)
    WriteCommand(0x80);
}


void PrintBlack(int offset)
{
    // Display the custom character at position 0
    WriteCommand(0xC0 + offset); // Set DDRAM address to 0
    WriteData(0);       // Display custom character at address 0
    
}
void main(void)
{
	char buff[80];
    int cnt=0;
    volatile unsigned long t=0;
    int adcval;
    float voltage;
    float voltage_x;
    float voltage_y;
    int adcval_x;
    int adcval_y;
    int a;
    int speaker = 1;
    int timeout;
    int timeout_cnt = 0;
    int received_value; //this is a TESTING variable
    int count =0;
    int freq_change = 0;
    int i;
    
    int set_freq = 1;	
    int ref_freq = 1;
	
	DDPCON = 0;
	CFGCON = 0;
	
	//SPEAKER STUFF
	ANSELAbits.ANSA1 = 0; // Disable analog function on RA1
    TRISAbits.TRISA1 = 0; // Set RA1 as output
    LATAbits.LATA1 = 0;   // Initialize RA1 to low
    
    ANSELA &= ~(1<<1);	//Set RA1 as a digital I/O
    TRISA &= ~(1<<1); 	//configure pin RA1 as output (speaker)

    //TRISAbits.TRISA1 = 0; // Set RA1 as output
    //LATAbits.LATA1 = 0;   // Initialize RA1 to low

    INTCONbits.MVEC = 1; // Enable multi-vector interrupts
	SetupTimer1(BASE_FREQ);
	LCD_4BIT();

	// Configure pins as analog inputs
    //ANSELBbits.ANSB1 = 1;   // set RB1 (AN4, pin 5 of DIP28) as analog pin	
    //TRISBbits.TRISB1 = 1;   // set RB1 as an input
    ANSELBbits.ANSB0 = 1;   // set RB0 (AN2, pin 3 of DIP28) as analog pin	
    TRISBbits.TRISB0 = 1;   // set RB0 as an input

    ANSELBbits.ANSB2 = 1;
    TRISBbits.TRISB2 = 1;	//set RB2 as input (x direction)
	
	ADCConf(); // Configure ADC
  
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    UART1Configure(9600);  // Configure UART1 to communicate with JDY40 with a baud rate of 9600
 
	delayms(500); // Give putty time to start before we send stuff.
    printf("JDY40 test program.\r\n");
	// RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB &= ~(1<<14);  // configure pin RB14 as output
	LATB |= (1<<14);    // 'SET' pin of JDY40 to 1 is normal operation mode


	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVID6058\r\n");  
	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");           
    ANSELB &= ~(1<<6); // Set RB6 as a digital I/O
    TRISB |= (1<<6);   // configure pin RB6 as input
    CNPUB |= (1<<6);   // Enable pull-up resistor for RB6



	printf("\r\nPress and hold a push-button attached to RB6 (pin 15) to transmit.\r\n");

	cnt=0;
	LCDprint("big n greedy", 1,1);
	while(1)
	{
		//MEASURE PINS FROM JOYSTICK
		adcval_y = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number
    	voltage_y=adcval_y*3.3/1023.0;
    	adcval_x = ADCRead(2); 	//reading from AN3 (RB1)
    	voltage_x = adcval_x*3.3/1023.0;
   	
		
		//FIRST SECTION SENDS AN M when we want a metal reading--how often depends on count variable and delays
		
		//if((PORTB&(1<<6))==0)
		if(count>=5)
		{
		count=0; //reset count every time M is sent
		sprintf(buff, "YM%.3f\nX%.3f\r\n", voltage_y, voltage_x);
		//printf("%d", strlen(buff));
		//printf("%s\n", buff);
		SerialTransmit1(buff);
		//printf("sent");

		timeout_cnt=0;
		while(1) {
			if(U1STAbits.URXDA) break; //Got something! get out of loop
			delayus(100); //check if smth has arrived
			timeout_cnt++;
			if(timeout_cnt>=1000) break; //timeout after x ms
			//printf("waiting");

		}
		
		if(U1STAbits.URXDA) 	// Something has arrived
		{		
			//printf("arrived");
			//delayms(100);
			SerialReceive1(buff, sizeof(buff)-1);
			//printf("\nReceived_val: %s\r\n", buff);
			//printf("%s\r\n", buff);
			delayms(10); //Important(?)
			//printf("received\r\n");
			if(strlen(buff)==4) //assuming a message from robot is 5 bytes
			{
				//printf("in\n");
				freq_change = atoi(buff);
				if(set_freq == 1)
				{
					ref_freq = freq_change + 70;
					set_freq = 0;
				}
				
				printf("%d\r\n",freq_change);
			}
		}	

		} else {

		sprintf(buff, "Y%.3f\nX%.3f\r\n", voltage_y, voltage_x);
		//printf("%d", strlen(buff));
		//printf("%s\n", buff);
		SerialTransmit1(buff);
		
		
		//delayms(10);            
    	}
    //	printf("%.3f %.3f\r", voltage_y, voltage_x);
    //	}
    	//printf("%.3f %.3f\r", voltage_y, voltage_x);
 	  	 // Makes the printf() above to send without a '\n' at the end	
	//printf("loop");
		//SPEAKER CODE
		//freq_change=100;
		if (freq_change > ref_freq) 
		{
			SetupTimer1(freq_change*5); //(this function also turns speaker on)
			printf("%.3f,%.3f,1\n\r",voltage_x,voltage_y); //python
			//WE NEED TO WRITE AN EQUATION FOR THIS PARAMETER, maybe use BASE_FREQ
			for(i = 1; i <= (freq_change-ref_freq) / 13 && (i < 16); i++)
			{
				DefineCustomCharacter(); 
				PrintBlack(i - 1); 	
			}
			
			   for(; i < 16; i++) 
			   {
		        WriteCommand(0xC0 + i); // Move cursor to position i
		        WriteData(' '); // Print a space to clear the character
		       }
		}
		else
		{
			T1CONbits.ON = 0; //speaker off otherwise
			LCDprint("", 2, 1);
			printf("%.3f,%.3f,0\n\r",voltage_x,voltage_y); //python
		}

		count++;
	}
}