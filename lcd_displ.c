// 6 kabelków
// LCD_D4,LCD_D5,LCD_D6,LCD_D7  magistrala 4 bity
// EN -  enable signal
// RS -  register select (dane albo rozkazy)
//       RW - zapis/odczyt u nas tylko zapis = 0



#include <avr/io.h>
#include <util/delay.h>

#include <stdlib.h>

#include "lcd_displ.h"
#include <avr/pgmspace.h>

// procedura wysyla 4 bity na wyswietlacz
void lcd_send_4(unsigned char b)
{
	set_lcd_en;

	// wysterowanie poszczególnych linii danych
	if (b & 1)	set_lcd_d4;	else clr_lcd_d4;
	if (b & 2)	set_lcd_d5;	else clr_lcd_d5;
	if (b & 4)	set_lcd_d6;	else clr_lcd_d6;
	if (b & 8)	set_lcd_d7;	else clr_lcd_d7;
	
	_delay_us(1);   // min 230ns

	// zapis danych opadajacym zboczem EN
	clr_lcd_en;

	_delay_us(1);   //  min 500ns
}

void lcd_send_8(unsigned char b)
{
	lcd_send_4(b>>4);  // najpierw czêœæ bardziej znacz¹ca
	lcd_send_4(b);
}


void lcd_send_com(unsigned char b)
{
	// rs = 0  wysy³anie rozkazu
	clr_lcd_rs;
	lcd_send_8(b);
	_delay_us(100);
}



void lcd_send_data(unsigned char b)
{
	// rs = 1  wysy³anie danej (do pamiêci)
	set_lcd_rs; // SET(PORT,LCD_RS);
	lcd_send_8(b);
	_delay_us(100);
}



void lcd_init(void)
{
	//kierunek portów = 1  (wyjscie)
	ddr_d4; ddr_d5; ddr_d6; ddr_d7; ddr_en; ddr_rs; ddr_light;
	
	_delay_ms(5);
	lcd_send_4(3); _delay_ms(1); lcd_send_4(3); _delay_ms(1);  // prze³¹czanie na magistral 4 bit
	lcd_send_4(3); _delay_ms(1); lcd_send_4(2); _delay_ms(1);
	lcd_send_com(0x28); _delay_ms(1); // szerokosc magistrali 4 bity,  2 linie
	lcd_send_com(0x01); _delay_ms(1); // clear
	lcd_send_com(0x06); _delay_ms(1); // inkrementacja DDRAM,
	lcd_send_com(0x0C); _delay_ms(1); // w³acz, bez kursora i migania
}


void lcd_control( unsigned char on, unsigned char cur, unsigned char blink)
{
	unsigned char v=0;
	v = 1<<3 |     (on!=0)<<2 |  (cur!=0)<<1 | (blink!=0);
	lcd_send_com( v );
}

void lcd_clear(void)
{
	lcd_send_com(1);
}

void lcd_home(void)
{
	lcd_send_com(2);
	_delay_ms(2);
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
	lcd_send_com( 128 |   ((y==1)? 64:0) | x  );
}

void lcd_swrite(char *s)
{int i=0;
	while(s[i] && i<1000)  { lcd_send_data(s[i]);	++i; }
}

void lcd_swrite_P(const char *s)
{
	register char znak;
	while ((znak=pgm_read_byte(s++)))
	{
		lcd_send_data(znak);
	}
}

void lcd_iwrite(int i)
{
	char s[10];
	itoa(  i, s, 10); lcd_swrite(s) ;
}