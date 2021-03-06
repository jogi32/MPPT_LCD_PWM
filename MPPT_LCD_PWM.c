/*
 * MPPT_LCD_PWM.c
 *
 * Created: 2015-07-27 18:55:17
 *  Author: TomaszSzafrański
 */ 

//TODO: define MACRO for TURN ON/OFF PIN and replace code
//TODO: VOltage and I are not correctly measure or/and display, this need to be changed fast

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
#define NUM_FOR_MES 100
#define SOLAR_Vcc 13

//Define sections for IN/OUT pin
#define PWM_BOOST		(1<<PB3)
#define SOLAR_RELAY		(1<<PD2)
#define BATERRY_RELAY	(1<<PD3)
#define LOAD_RELAY		(1<<PD4)

//Other definitions and declarations
void ADC_ACS712_Calib();
void ADC_init();
void ADC_mes();
void ADC_RUNTIME(uint32_t* MES);
void Hello();
void Mes_V();
void Mes_I();
void Mes_P();
void PWM_init();
void START_init();

//Other definitions and declarations for global variable
uint32_t Mes_tmp = 0, I_flor = 0;

char bufor[16];

float Vf = 0.0, If = 0.0, Pf = 0.0, Vfp = 0.0, Ifp = 0.0, Pfp = 0.0;

//Const sections
float const Factor = 5.0 / 1024.0 / 0.185; //(0.00488/ 0.185)*10000 = 264;    //5/1024 = 0.00488  // Sensitivity = 185mV

/* MAIN FUNCTION */
int main(void)
{
	lcd_init();
	START_init();
	ADC_init();
	Hello();
	PORTD	&= ~SOLAR_RELAY;
	PORTD	&= ~BATERRY_RELAY;
	PWM_init();
	ADC_ACS712_Calib();
	OCR0 = 85;
	srand(OCR0);
	
	
	
    while(1)
    {
        //TODO:: Please write your application code 
		lcd_clear();
		lcd_home();
		Mes_V();
		Mes_I();
		Mes_P();
		
		
		////////////////
		//MPPT
		///////////////
		if (Pf > Pfp)
		{
			if (Vf > Vfp)
			{
				OCR0 += 10;
			} 
			else
			{
				OCR0 -= 10;
			}
		}
		else
		{
			if (Vf > Vfp)
			{
				OCR0 -= 10;
			}
			else
			{
				OCR0 += 10;
			}	
		}
		
		Vfp = Vf;
		Pfp = Pf;
		
		//////////////////////////////////////////////////////////////////////////
		//ZABEZPIECZENIE PRZED ZWARCIEM
		/////////////////////////////////////////////////////////////////////////
		if ((OCR0 <= 15) || (OCR0 >= 245))
		{
			OCR0 = rand()%190 + 50;
		} 
		
		lcd_swrite(" OCR "); lcd_iwrite(OCR0);
		_delay_ms(100);			//TODO: remove or redesign this part later
    }
}

void ADC_ACS712_Calib()
{
	//calibration of I mes
	ADMUX = (0<<REFS1) | (1<<REFS0) | GLUE(ADC, 2);
	_delay_ms(100);
	
	uint8_t i = 0;
	for (i = 0; i < NUM_FOR_MES; i++)
	{
		ADC_RUNTIME(&Mes_tmp);
		I_flor = I_flor + Mes_tmp;
	}
	lcd_clear();
	lcd_home();
	lcd_swrite("I mes cal");
	lcd_gotoxy(0,1);
	_delay_ms(100);
	I_flor = I_flor / NUM_FOR_MES;
	lcd_swrite("ADC 0: "); lcd_iwrite(I_flor);
	
	_delay_ms(2000);
}

void ADC_init()
{
	//Start ADC, external Vcc, one conversion mode, preskaler 128, input PIN0-7, align to right
	ADCSRA |= 	 (1<<ADEN)					//Bit 7 – ADEN: ADC Enable 
	|(1<<ADPS0)
	|(1<<ADPS1)
	|(1<<ADPS2);							//ADPS2:0: ADC Prescaler Select Bits (set prescaler) preskaler= 128

	ADMUX  =//	 (1<<REFS1) | (1<<REFS0)	//Bit 7:6 – REFS1:0: Reference Selection Bits
											//Internal 2.56V Voltage Reference with external capacitor at AREF pin
				 (0<<REFS1) | (1<<REFS0)    //External 5.00V Voltage Reference with external capacitor at AREF pin
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

void ADC_RUNTIME(uint32_t* MES)
{
	uint8_t i = 0;
	*MES = 0;
	for (i = 0; i<NUM_FOR_MES; i++)
	{
		ADC_mes();
		*MES += ADC;
	}
	*MES /= NUM_FOR_MES;
}

void Mes_V()
{
	ADMUX = (0<<REFS1) | (1<<REFS0) | GLUE(ADC, 0);
	ADC_RUNTIME(&Mes_tmp);
	Vf = Mes_tmp;					//resistor divider scale (7.2 + 11,8) / 7,2 
	Vf = Vf * SOLAR_Vcc / 1023;
	sprintf(bufor,"%.2f",Vf);
	lcd_swrite("V="); lcd_swrite(bufor);
}

void Mes_I()
{
	ADMUX = (0<<REFS1) | (1<<REFS0) | GLUE(ADC, 2);
	ADC_RUNTIME(&Mes_tmp);

	if (Mes_tmp <= I_flor) //bias for eliminate of minus mA
	{
		I_flor = Mes_tmp;
	}
	If = (Mes_tmp-I_flor)*Factor;
	sprintf(bufor,"%.3f",If);
	lcd_swrite(" I="); lcd_swrite(bufor);
}

void Mes_P()
{
	Pf = Vf * If;
	sprintf(bufor,"%.3f",Pf);
	lcd_gotoxy(0,1);
	lcd_swrite("P="); lcd_swrite(bufor);
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
	OCR0 = 245;						//Value of compared variable
	TIMSK	|= (1<<TOIE0);			//TODO: set descriptions
	TIFR	|= (1<<OCF0);			//TODO: set descriptions
}

//Set initial status of used PIN
//TODO: DEFINE THIS PIN
void START_init()
{	
	DDRB	|= PWM_BOOST;	//MOSFET enable
	//PORTB	&= ~PWM_BOOST;
	PORTB	|= PWM_BOOST;	//MOSFET disable

	DDRD	|= SOLAR_RELAY;	//PV enable 
	//PORTD	&= ~SOLAR_RELAY;
	PORTD	|= SOLAR_RELAY;	//PV disable 
	
	DDRD	|= BATERRY_RELAY;	//Battery enable
	//PORTD	&= ~BATERRY_RELAY;
	PORTD	|= BATERRY_RELAY;	//Battery disable 
	
	
	DDRD	|= LOAD_RELAY;	//Load enable
	//PORTD	&= ~LOAD_RELAY;
	PORTD	|= LOAD_RELAY;	//Battery disable 
}