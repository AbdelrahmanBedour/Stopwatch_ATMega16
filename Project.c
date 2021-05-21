/*
 * Project.c
 *
 *  Created on: Jan 31, 2021
 *      Author: admin
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char sec1=0,sec2=0,min1=0,min2=0,hour1=0,hour2=0;

void TIMER1_INIT()
{
	TCCR1A = 0;
	TCCR1A |= (1<<3);
	TCCR1B |= (1<<3) | (1<<2) |(1<<0) ; /* activating CTC mode with N=1024*/
	TCNT1 = 0 ; /* Reset counter */
	OCR1A = 977; /* setting compare register to 977 ticks  */
	TIMSK |= (1<<4); /* setting module interrupt */

}

void INT0_INIT()
{
	DDRD &= ~(1<<2) ; /* PD2 set to input pin*/
	PORTD |= (1<<2) ; /* activate internal pull up*/
	MCUCR |= (1<<ISC01);
	MCUCR &= ~(1<<ISC00); /* assign interrupt at falling edge*/
	GICR |= (1<<INT0) ; /* activating module interrupt */
}

void INT1_INIT()
{
	DDRD &= ~(1<<3) ; /* PD3 set to input pin*/
	MCUCR |= (1<<ISC10);
	MCUCR |= (1<<ISC11); /* assign interrupt at raising edge*/
	GICR |= (1<<INT1) ; /* activating module interrupt */
}

void INT2_INIT()
{
	DDRB &= ~(1<<2) ; /* PB2 set to input pin*/
	PORTB |= (1<<2) ; /* activate internal pull up*/
	MCUCSR |= (1<<ISC2); /* assign interrupt at falling edge*/
	GICR |= (1<<INT2) ; /* activating module interrupt */
}

ISR(INT0_vect)
{
	sec1=0;
	sec2=0;
	min1=0;
	min2=0;
	hour1=0;
	hour2=0;
}

ISR(INT1_vect)
{
	TCCR1B &= ~(0x07); /* stop the clock*/
}

ISR(INT2_vect)
{
	TCCR1B |= 5; /* resume the clock */
}

ISR(TIMER1_COMPA_vect)
{
	sec1++;
	if(sec1==10)
		{
			sec2++;
			sec1=0;

			if(sec2==6)
					{
						min1++;
						sec2=0;

						if(min1==10)
								{
									min2++;
									min1=0;

									if(min2==6)
											{
												hour1++;
												min2=0;

												if(hour1==10)
														{
															hour2++;
															hour1=0;

															if(hour2==10)
																	{
																		hour2=0;
																	}
														}
											}
								}
					}
		}





}

int main ()
{
	INT0_INIT();
	INT1_INIT();
	INT2_INIT();
	TIMER1_INIT();
	SREG |= (1<<7); /* activating global interrupt */
	DDRC |= 0x0F; /* making first 4 pins of C output*/
	DDRA |= 0x3F; /* making first 6 pins of A output*/
	PORTC &= 0xF0;
	PORTA &= ~(0x3F);

for (;;)
{
	PORTA |= (1<<0);
	PORTC = (PORTC & 0xF0) | (sec1 & 0x0F) ;
	_delay_ms(3);
	PORTA &= ~(0x3F);

	PORTA |= (1<<1);
	PORTC = (PORTC & 0xF0) | (sec2 & 0x0F) ;
	_delay_ms(3);
	PORTA &= ~(0x3F);

	PORTA |= (1<<2);
	PORTC = (PORTC & 0xF0) | (min1 & 0x0F) ;
	_delay_ms(3);
	PORTA &= ~(0x3F);

	PORTA |= (1<<3);
	PORTC = (PORTC & 0xF0) | (min2 & 0x0F) ;
	_delay_ms(3);
	PORTA &= ~(0x3F);

	PORTA |= (1<<4);
	PORTC = (PORTC & 0xF0) | (hour1 & 0x0F) ;
	_delay_ms(3);
	PORTA &= ~(0x3F);

	PORTA |= (1<<5);
	PORTC = (PORTC & 0xF0) | (hour2 & 0x0F) ;
	_delay_ms(3);
	PORTA &= ~(0x3F);
}

}
