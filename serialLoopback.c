/*
A simple test of serial-port functionality.
Takes in a character at a time and sends it right back out,
 displaying the ASCII value on the LEDs.
*/
#define __AVR_ATmega328P__

// ------- Preamble -------- //
#include <avr/io.h>
#include <util/delay.h>
#include "pinDefines.h"
#include "USART.h"
#include "scale16.h"

#define LCD_DATA PORTB          // port B is selected as LCD data port
#define color_port PORTC
#define en PC5                 // enable signal is connected to port D pin 7
#define rs PC4                  // register select signal is connected to port D pin 5
#define NOTE_DURATION     0xF000  

void LCD_cmd(unsigned char cmd);
void init_LCD(void);
void LCD_write(unsigned char data);
void playNote(uint16_t period, uint16_t duration);
void rest(uint16_t duration);
void LCD_toggle_color();

unsigned int color = 0b00000000;

main()
{
  DDRB=0xFF;              // set LCD data port as output
  DDRC=0xFF;              // RGB
  DDRD=0b11111011;              // set LCD control port as output

  PORTD |= (1 << PD2);

  uint8_t buttonWasPressed;

  //char serialCharacter;
  init_LCD();             // initialize LCD
  _delay_ms(20);   

  LCD_cmd(0x0C);          // display on, cursor off 
  LCD_writestr(">  ");
  LCD_cmd(0xC0);          // move cursor to the start of 2nd line
  LCD_cmd(0x0C);          // display on, cursor off

  while (1) {
      
    if (bit_is_clear(PIND, PD2)) {
      if (!buttonWasPressed) {
        buttonWasPressed = 1;
        LCD_writestr("P "); 
        LCD_toggle_color();
      }
    }  
    else {
      if (buttonWasPressed) {
        LCD_writestr("R ");
      }
      buttonWasPressed = 0;
    }

    LCD_cmd(0x0E);          // make display ON, cursor ON
  }
}

void init_LCD(void)
{
  LCD_cmd(0x38);           // initialization in 8bit mode of 16X2 LCD
  _delay_ms(1);

  LCD_cmd(0x01);          // make clear LCD
  _delay_ms(1);

  LCD_cmd(0x02);          // return home
  _delay_ms(1);

  LCD_cmd(0x06);          // make increment in cursor
  _delay_ms(1);

  LCD_cmd(0x80);          // “8” go to first line and “0” is for 0th position
  _delay_ms(1);

  return;
}

 

//**************sending command on LCD***************//
void LCD_cmd(unsigned char cmd)
{
  LCD_DATA = cmd;      // data lines are set to send command

  PORTC  &= ~(1<<rs);  // RS sets 0
  PORTC  |= (1<<en);   // make enable from high to low
  _delay_ms(1);
  PORTC  &= ~(1<<en);

  return;
}

 

//*****************write data on LCD*****************//

void LCD_write(unsigned char data)
{
  LCD_DATA= data;       // data lines are set to send command

  PORTC  |= (1<<rs);    // RS sets 1
  PORTC  |= (1<<en);    // make enable from high to low
  _delay_ms(1);
  PORTC &= ~(1<<en);

  return ;
}

void LCD_writestr(char *str)
{
  int i=0;
  while(str[i]!=0)
  {
    LCD_write(str[i]);
    //_delay_ms(30);
    //LCD_toggle_color();
    i++;
  }
}

void LCD_toggle_color()
{
  color++;

  if (color > 7)
  {
    color = 0;
  }

  color_port = color;
  //set color to navy
  //color_port = 0b00000001;

  //set color to purple
  //color_port = 0b00000010;

  //set color to blue
  //color_port = 0b00000011;

  //set color to yellow
  //color_port = 0b00000100;

  //set color to green
  //color_port = 0b00000101;

  //set color to red
  //color_port = 0b00000110;

  //set color to Off
  //color_port = 0b00000111;

  //set color to white
  //color_port = 0b00000000; 
}

void playNote(uint16_t period, uint16_t duration) {
  uint16_t elapsed;
  uint16_t i;
  for (elapsed = 0; elapsed < duration; elapsed += period) {
                     /* For loop with variable delay selects the pitch */
    for (i = 0; i < period; i++) {
      _delay_us(1);
    }
    SPEAKER_PORT ^= (1 << SPEAKER);
  }
}

void rest(uint16_t duration) {
  do {
    _delay_us(1);
  } while (--duration);
}
