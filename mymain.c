#include "TM4C123GH6PM.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "Nokia5110.h"

void Init_state(){
	
	//POTENTIOMETER CONFIGURATION
	
	SYSCTL->RCGCGPIO |= 0x02;
	SYSCTL->RCGCADC |=0x01;
	
	GPIOB->AFSEL |= (1<<4);
	GPIOB->DIR |= (1<<4);
	GPIOB->DIR &= ~(1<<4);
	GPIOB->AMSEL |= (1<<4);

	ADC0->ACTSS &= ~0x08;
	ADC0->EMUX &= ~0xF000;
	ADC0->SSMUX3 = 0x0A;
	ADC0->SSCTL3 |= 0x06;
	ADC0->PC |= (1<<0);
	ADC0->ACTSS |= 0x08;
}
//ADC VALUE READING 0-4095
static int get_value(void){ 
int x = 0;
ADC0->PSSI |= (1<<3);
	while((ADC0->RIS&(1<<3)) == 0){};
	if(ADC0->RIS==0x08){
	x = ADC0->SSFIFO3;
	}
	return x;
}

	// THESE ARE THE FUNCTIONS NECESSARY FOR CONFIG THE BMP280 AND READ DATA FROM IT
char I2C3_Write_Multiple(int slave_address, char slave_memory_address, int bytes_count, char* data);

char I2C3_Read_Multiple(int slave_address, char slave_memory_address, int bytes_count, char* data);

void initLED(void)
{
	  //LED INIT 
		 SYSCTL->RCGCGPIO|= 0x20;
	   GPIOF->LOCK = 0x4C4F434B;   // unlockGPIOCR register
		 GPIOF->CR = 0x01;           // Enable GPIOPUR register enable to commit
		 GPIOF->PUR |= 0x10; 
		
		 GPIOF->AFSEL &= 0x00; // Disable Alternate function  	
		 GPIOF->DEN |= 0x1F; // Enable digital functions for PAF0,1,2,3,4 
		 GPIOF->DIR &= 0x00; // Set PF0 and PF4  as input  
		 GPIOF->DIR |= 0x0E; // Set PF1 and PF2,PF3  as output
		
		 GPIOF -> IS &= 0x00;
		 GPIOF -> IBE &= 0x00;
		 GPIOF -> IM |= 0x11;
		 GPIOF -> ICR |= 0x11;
	
    // Enable the clock for the GPIOF port
    //SYSCTL->RCGCGPIO|= 0x20;  // Set bit 5 of SYSCTL_RCGCGPIO_R to 1
		//while (!SYSCTL->PRGPIO)
		//{}
    // Set the LED pins (PF1, PF2, PF3) as output pins
    //GPIOF->DIR|= 0x0E;  // Set bits 1, 2, and 3 of GPIO_PORTF_DIR_R to 1
		//GPIOF->AFSEL &= ~(0x0E);
		//GPIOF->PCTL	&= ~(0xFFFFFFFF);	// Enable digital function for the LED pins
		//GPIOF->AMSEL &= ~(0x0E);
    //GPIOF->DEN |= 0x0E;  // Set bits 1, 2, and 3 of GPIO_PORTF_DEN_R to 1
}
void red_on_all_off(){
		GPIOF->DATA &= ~0x0E;  // Set the output of pin PF1 (red LED) to low
		GPIOF->DATA|= 0x02;  // Set the output of pin PF2 (blue LED) to high

}

void blue_on_all_off(){
	GPIOF->DATA &= ~0x0E;  // Set the output of pin PF1 (red LED) to low
  GPIOF->DATA|= 0x04;  // Set the output of pin PF2 (blue LED) to high
    
}


void green_on_all_off(){
	GPIOF->DATA&= ~0x0E;  // Set the output of pin PF1 (red LED) to low
  GPIOF->DATA |= 0x08;  // Set the output of pin PF3 (green LED) to high
}


static int I2C_wait_till_done(void)
{
    while(I2C3->MCS & 1);   /* wait until I2C master is not busy */
    return I2C3->MCS & 0xE; /* return I2C error code, 0 if no error*/
}

char I2C3_Write_Multiple(int slave_address, char slave_memory_address, int bytes_count, char* data)
{   
    char error;
    if (bytes_count <= 0)
        return -1;                  /* no write was performed */
    /* send slave address and starting address */
    I2C3->MSA = slave_address << 1;
    I2C3->MDR = slave_memory_address;
    I2C3->MCS = 3;                  /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C_wait_till_done();   /* wait until write is complete */
    if (error) return error;

    /* send data one byte at a time */
    while (bytes_count > 1)
    {
        I2C3->MDR = *data++;             /* write the next byte */
        I2C3->MCS = 1;                   /* -data-ACK- */
        error = I2C_wait_till_done();
        if (error) return error;
        bytes_count--;
    }
    
    /* send last byte and a STOP */
    I2C3->MDR = *data++;                 /* write the last byte */
    I2C3->MCS = 5;                       /* -data-ACK-P */
    error = I2C_wait_till_done();
             /* wait until bus is not busy */
    if (error) return error;
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
    return 0;       /* no error */
}



char I2C3_Read_Multiple(int slave_address, char slave_memory_address, int bytes_count, char* data)
{
    char error;
    
    if (bytes_count <= 0)
        return -1;         /* no read was performed */

    /* send slave address and starting address */
    I2C3->MSA = slave_address << 1;
    I2C3->MDR = slave_memory_address;
    I2C3->MCS = 3;       /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C_wait_till_done();
    if (error)
        return error;

    /* to change bus from write to read, send restart with slave addr */
    I2C3->MSA = (slave_address << 1) + 1;   /* restart: -R-(saddr+r)-ACK */

    if (bytes_count == 1)             /* if last byte, don't ack */
        I2C3->MCS = 7;              /* -data-NACK-P */
    else                            /* else ack */
        I2C3->MCS = 0xB;            /* -data-ACK- */
    error = I2C_wait_till_done();
    if (error) return error;

    *data++ = I2C3->MDR;            /* store the data received */

    if (--bytes_count == 0)           /* if single byte read, done */
    {
        
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			__ASM("NOP");
			
			
			/* wait until bus is not busy */
        return 0;       /* no error */
    }
 
    /* read the rest of the bytes */
    while (bytes_count > 1)
    {
        I2C3->MCS = 9;              /* -data-ACK- */
        error = I2C_wait_till_done();
        if (error) return error;
        bytes_count--;
        *data++ = I2C3->MDR;        /* store data received */
    }

    I2C3->MCS = 5;                  /* -data-NACK-P */
    error = I2C_wait_till_done();
    *data = I2C3->MDR;              /* store data received */
    //while(I2C3->MCS & 0x40);        /* wait until bus is not busy */
		
    return 0;       /* no error */
}

void I2C3_Init ( void )
{
	
SYSCTL->RCGCGPIO  |= 0x00000008 ; // Enable the clock for D port
SYSCTL->RCGCI2C   |= 0x00000008 ; // Enable the clock for I2C 3
GPIOD->DEN |= 0x03; // for port D
GPIOD->AFSEL |= 0x00000003 ; // Configure Port D pins 0 and 1 as I2C 3 
GPIOD->PCTL |= 0x00000033 ;
GPIOD->ODR |= 0x00000002 ; // SDA (PD1 ) pin as open drain
I2C3->MCR  = 0x0010 ; // Enable I2C 3 master function
	
/* Configure I2C 3 clock frequency
(1 + TIME_PERIOD ) = SYS_CLK /(2*
( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
TIME_PERIOD = 16 ,000 ,000/(2(6+4) *100000) - 1 = 7 */
	
I2C3->MTPR  = 0x07 ;
}


int main() {
	
//*************************************

	
	//******************************
	volatile float valueadc = 0;	
	Init_state();//adc
	int High_TH;
	int Low_TH;
	
	Nokia5110_Init();
	
  double sum=0;
	int average=0;
	int i = 0;
	unsigned short dig_T1=27504;
	short dig_T2=26435;
	short dig_T3= -1000;
	
	
	long signed int t_fine;
	int32_t value = 0;
	uint8_t val3 = 0b00000000; // 4 bits
	double var1,var2; //FOR TEMP CALCULATION
	double T;    
	I2C3_Init();
	
	char data[2] = {0x00,0x27};
	char datareset[1] = {0xB6};
	 
	  I2C3_Write_Multiple(0x76,0X0E,1,datareset); //Could be removed for later, it resets the i2c bus according to bosch datasheet
	  I2C3_Write_Multiple(0x76, 0XF5, 1,  data); // MSBdata
	  I2C3_Write_Multiple(0x76, 0XF4, 1,  data+1); // MSBdata+1

	char str[50];			//FOR PRINTING TEMP TEXT 
	char adchigh[50]; //For print HIGH LIMIT to LCD
	char adclow[50];  //For print LOW LIMIT to LCD
	
	Nokia5110_Clear();
	initLED();
	GPIOF->DATA |= 0x0E;
	GPIOF->DATA &= ~(0x0C);
	 
	//uint32_t button;	 
	while(1){
		
			/*if(GPIOF -> RIS	== 0x10){	
					button=0; //low int
					GPIOF -> ICR = 0xFFFFFFFF;	
				}
			if(GPIOF -> RIS	== 0x01){
				  button=1; //high interrupt
				  GPIOF -> ICR = 0xFFFFFFFF;	
				}

			if(button==0){
		
				High_TH = (get_value()/4095) * 10 + 25;
			
				}
			if(button==1){
	
				Low_TH = 25 - (get_value()/4095) * 10;
			  
				}
				GPIOF -> ICR = 0xFFFFFFFF;	*/
				
		   valueadc = get_value(); // We upload the ADC reading to valueadc
			 
		   High_TH = (valueadc/4095) * 10 + 25;   //High limit arrange
			 
			
			 /*__ASM("NOP");
			 __ASM("NOP");
			 __ASM("NOP");
			 __ASM("NOP");
			 __ASM("NOP");
			 __ASM("NOP");
			 __ASM("NOP");
			 __ASM("NOP");*/
			//valueadc = get_value();
			//Low_TH = 25 - (valueadc/4095) * 10;
		
		   //Low_TH = 25 - (valueadc/4095) * 10;
		  
			
			
		
			while(i!=128)
		{
		  I2C3_Read_Multiple(0x76, 0XFA, 2,  data); // MSB
		  value =(uint32_t) (data[0] << 12) | (data[1] << 4) | val3;
		  var1=(((double)value)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
		  var2=((((double)value)/131072.0 - ((double)dig_T1)/8192.0)*(((double)value)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
		  t_fine=(long signed int)(var1+var2);
		  T=(var1+var2)/5120.0;
			sum += T;
			i++;
		}
		average = sum / 128.0;
		
		
		sum = 0;
		i = 0;
		
		if(average<High_TH && average>22)
		{
		
			green_on_all_off();
		  // turn off the peltier and heatpad, pc4 and pc5 pins are low
			SYSCTL->RCGCGPIO |= (1U << 2);
			GPIOC->DEN |= (1U << 5);
			GPIOC->DIR |= (1U << 5);
			GPIOC->DATA &= ~(1U << 4);
			GPIOC->DATA &= ~(1U << 5);
			
			
		}
		else if(average<22)
		{
		
			red_on_all_off();
			// turn off the peltier and turn on the heatpad, pc4-  low and pc5- high
			SYSCTL->RCGCGPIO |= (1U << 2);
			GPIOC->DEN |= (1U << 5);
			GPIOC->DIR |= (1U << 5);
			GPIOC->DATA |= (1U << 5);
			GPIOC->DATA &= ~(1U << 4);
			
			
		}
		//(average>High_TH)
		else
		{
		
				blue_on_all_off();
			// turn off the heatpad and turn on the peltier pc4- high and pc5-low
				SYSCTL->RCGCGPIO |= (1U << 2);
				GPIOC->DEN |= (1U << 4);
				GPIOC->DIR |= (1U << 4);
				GPIOC->DATA |= (1U << 4);
			  GPIOC->DATA &= ~(1U << 5);
				
		}
		
		Nokia5110_SetCursor(0, 0);
	  sprintf(str, " Temperature : %d", average); 
		Nokia5110_OutString(str);
		
		Nokia5110_SetCursor(1, 3);
		sprintf(adchigh, "High: %d", High_TH);
		Nokia5110_OutString(adchigh);
	  //Nokia5110_OutString("L: 22 H: 30" );
		
		Nokia5110_SetCursor(2, 4);
		Nokia5110_OutString("Low: 22");
		//sprintf(adclow, "Low: %d", Low_TH);
		//Nokia5110_OutString(adclow);
	
		
		
	}
		
}	
	
	
