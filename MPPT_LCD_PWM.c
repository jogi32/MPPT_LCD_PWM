/*
 * MPPT_LCD_PWM.c
 *
 * Created: 2015-07-27 18:55:17
 *  Author: TomaszSzafrański
 */ 

#include "lcd_displ.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

//Define sections
//This should be defined in project properties
//#define F_CPU 16000000

#define ADCIN_SOLAR PC1
#define ADCIN_BAT PC2
#define ADCIN_I PC3
//For future development of solar tracking code
//#define ADCINBOT PC2
//#define ADCINTOP PC3
//#define ADCINBOT PC2
//#define ADCINTOP PC3

#define ADC0			(0<<MUX2) |(0<<MUX1) | (0<<MUX0)
#define ADC1			(0<<MUX2) |(0<<MUX1) | (1<<MUX0)
#define ADC2			(0<<MUX2) |(1<<MUX1) | (0<<MUX0)
#define ADC3			(0<<MUX2) |(1<<MUX1) | (1<<MUX0)
#define ADC4			(1<<MUX2) |(0<<MUX1) | (0<<MUX0)
#define ADC5			(1<<MUX2) |(0<<MUX1) | (1<<MUX0)
#define ADC6			(1<<MUX2) |(1<<MUX1) | (0<<MUX0)
#define ADC7			(1<<MUX2) |(1<<MUX1) | (1<<MUX0)

#define GLUE(x, y) x##y
#define NUM_FOR_MES 10

//Define sections for IN/OUT pin
#define PWM_BOOST (1<<PB3)

//Other definitions and declarations
void ADC_init();
void ADC_mes();
void ADC_RUNTIME(uint16_t* MES);
void Display_I(uint16_t* num);
void Display_P(uint16_t* num);
void Hello();
void PWM_init();
void START_init();

//Other definitions and declarations for global variable


//Const sections
uint16_t const Factor = 264; //0.00488/ 0.185 = 264;    //5/1024 = 0.00488  // Sensitivity = 185mV

/* MAIN FUNCTION */
int main(void)
{
	lcd_init();
	ADC_init();
	Hello();
	
    while(1)
    {
        //TODO:: Please write your application code 
		
		_delay_ms(100);		//TODO: remove or redesign this part later
    }
}

void ADC_init()
{
	//Start ADC, external Vcc, one conversion mode, preskaler 128, input PIN0-7, align to right
	ADCSRA |= 	 (1<<ADEN)					//Bit 7 – ADEN: ADC Enable 
	|(1<<ADPS0)
	|(1<<ADPS1)
	|(1<<ADPS2);							//ADPS2:0: ADC Prescaler Select Bits (set prescaler) preskaler= 128

	ADMUX  =	 (1<<REFS1) | (1<<REFS0)	//Bit 7:6 – REFS1:0: Reference Selection Bits
											//Internal 2.56V Voltage Reference with external capacitor at AREF pin
			//	|(0<<REFS1) | (1<<REFS0)    //External 5.00V Voltage Reference with external capacitor at AREF pin
				|(1<<MUX1) | (1<<MUX0);		//Input Channel Selections (ADC3 - Pin 3 )
	
	DDRC &=~ (1<<PC0);            //Set input ADC
	DDRC &=~ (1<<PC1);            //Set input ADC
	DDRC &=~ (1<<PC2);            //Set input ADC
	DDRC &=~ (1<<PC3);            //Set input ADC
	DDRC &=~ (1<<PC4);            //Set input ADC
	DDRC &=~ (1<<PC5);            //Set input ADC
	DDRC &=~ (1<<PC6);            //Set input ADC
	DDRC &=~ (1<<PC7);            //Set input ADC
}

void ADC_mes()
{
	ADCSRA |= (1<<ADSC);		//Bit 6 – ADSC: ADC Start Conversion (run single conversion)
	while(ADCSRA & (1<<ADSC));	//wait for end of conversion
}

void Display_I(uint16_t* Itemp)
{
	char char_I[] = "I=0.000A";
	char_I[2] = (*Itemp)/1000 + 48;
	char_I[4] = ((*Itemp)/100)%10 + 48;
	char_I[5] = ((*Itemp)/10)%10 + 48;
	char_I[6] = ((*Itemp))%10 + 48;
	lcd_swrite(char_I);
}

void Display_P(uint16_t* P)
{
	char char_P[] = " P=0.00W";
	char_P[3] = (uint16_t)((*P)/100) + 48;
	char_P[5] = (uint16_t)((*P)/10)%10 + 48;
	char_P[6] = (uint16_t)((*P))%10 + 48;
	lcd_swrite(char_P);
}

void Hello()
{
	lcd_swrite_P(PSTR("MPPT BOOST CONTR"));
	lcd_gotoxy(0,1);
	uint8_t count = 0;
	for (count = 0;	count < 16;	count++)
	{
		lcd_swrite("+");
		_delay_ms(50);
	}
	_delay_ms(500);
	lcd_clear();
	lcd_home();
	lcd_swrite_P(PSTR("DESIGNED BY:"));
	lcd_gotoxy(0,1);
	lcd_swrite_P(PSTR("T. SZAFRANSKI"));
	_delay_ms(500);
	lcd_clear();
	lcd_home();
}

void PWM_init()
{
	/* SET OUTPUT */
	DDRB |= (PWM_BOOST);			//PWM OUTPUT
	
	/* INITIALIZACTINS OF PWM - TIMER0 */
	TCCR0	|= (1<<WGM01);			// Fast PWM 8bit
	TCCR0	|= (1<<WGM00);
	TCCR0	|= (1<<COM01) ;			//Clear OC0A/OC0B on Compare Match, set OC0A/OC0B at BOTTOM
	TCCR0	|= (1<<CS01) | (1<<CS00);              // Preksaler = 64 fpwm = 976,5 Hz
	OCR0 = 40;						//Value of compared variable
	TIMSK	|= (1<<TOIE0);			//TODO: set descriptions
	TIFR	|= (1<<OCF0);			//TODO: set descriptions
}

//Set initial status of used PIN
//TODO: CHECK AND SET THIS PIN
void START_init()
{
	DDRC	|= _BV(PC7);
	PORTC	|= _BV(PC7);
	DDRC	|= _BV(PC6);
	PORTC	|= _BV(PC6);
	DDRC	|= _BV(PC5);
	PORTC	|= _BV(PC5);
	DDRC	|= _BV(PC4);
	PORTC	|= _BV(PC4);	
	DDRD	|= (1<<PD2);	//PV enable 
	PORTD	&= ~(1<<PD2);
}