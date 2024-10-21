//********************************************************************
//*                      EEE2046F STM32F0                            *
//*                         LCD MODULE                               *
//*==================================================================*
//* WRITTEN BY:    Copyright (C) Samuel Ginsberg 2004                *
//* PORTED TO STM32F0 dev board by James Gowans, 2014                *
//* MODIFIED BY:   Robyn Verrinder                                   *
//* MODIFIED BY:   Justin Pead for HAL                               *
//* DATE CREATED:  2004                                              *
//* PORTED:	   2014						                             *
//* MODIFIED:      20-09-2024                                        *
//*==================================================================*
//* PROGRAMMED IN: VSC                                               *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//* TARGET:	   STMicroelectronics STM32F051C8                        *
//*==================================================================*
//* DESCRIPTION:   This code contains common functions to communicate*
//*                with the LCD module connected to the STM32 uC.    *
//*==================================================================*
//* LCD SETUP:     - 4 bit mode      (Upper 4 data lines D4-D7 used) *
//*                - Two lines used                                  *
//*                - Flashing cursor                                 *
//*==================================================================*
//* CONNECTIONS:                                                     *
//*------------------------------------------------------------------*
//* LCD PINS   | NAME                    | CONNECTED TO              *
//*------------------------------------------------------------------*
//* 1............VSS.......................GND                       *
//* 2............VDD.......................+5V                       *
//* 3............CONTRAST..................POT 2                     *
//* 4............RS  - Register Select.....PC14 (LCD_RS)             *
//* 5............RW  - Read/Write..........GND                       *
//* 6............E   - Enable..............PC15 (LCD_E)              *
//* 7............D0  - Data line 0.........GND                       *
//* 8............D1  - Data line 1.........GND                       *
//* 9............D2  - Data line 2.........GND                       *
//* 10...........D3  - Data line 3.........GND                       *
//* 11...........D4  - Data line 4.........PB8  (LCD_D4)             *
//* 12...........D5  - Data line 5.........PB9  (LCD_D5)             *
//* 13...........D6  - Data line 6.........PA12 (LCD_D6)             *
//* 14...........D7  - Data line 7.........PA15 (LCD_D7)             *
//* 15...........CATHLED...................NC                        *
//* 16...........ANODELED..................NC                        *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f4xx_hal.h"
#include "main.h"
//====================================================================
// SEND COMMAND CODE TO LCD - LCD_Command(command)
//====================================================================
// DESCRIPTION: This function sends a command to the LCD. Care is taken
//              not to interfere with the other lines on the port.
//
//              As we are using a microcontroller to control the LCD
//              we use 4-bit mode to save on number of lines used to
//              connect to the LCD. This means that an 8-bit command
//              must be split into two sets of 4-bits (upper and lower)
//              These sets must be transmitted
//====================================================================
// USEFUL COMMANDS:
//                  - POWER_UP:      Power up initialization for the lcd
//                  - FOURBIT_MODE:  Sets LCD for 4-bit mode
//                  - TWOLINE_MODE:  Sets up 2 lines and character size
//                  - SETUP_CURSOR:  Turn display on and set up cursor
//                  - CLEAR:         Clear screen
//                  - CURSOR_HOME:   Cursor home
//                  - LINE_TWO:      Line 2
//
//====================================================================

lcd mylcd;

void lcd_command(unsigned char command) {
    HAL_GPIO_WritePin(mylcd.RS_port,mylcd.RS_pin,0);
    // Register Select (RS)line low (data sent will now be read as commands)
    write_byte(command);    
    HAL_Delay(3);
}

//====================================================================
// INITIALISE LCD - LCD_Init()
//====================================================================
// DESCRIPTION: This function sets up the port lines for the LCD and
//              intialises the module for use.
//====================================================================
// LCD SETUP:     - 4 bit mode      (Upper 4 data lines D4-D7 used)
//                - Two lines used
//                - Flashing cursor
//====================================================================

__WEAK void init_LCD_pins(void) {
    mylcd.data_pin_ports[3] = LCD_D7_GPIO_Port;
    mylcd.data_pin_ports[2] = LCD_D6_GPIO_Port;
    mylcd.data_pin_ports[1] = LCD_D5_GPIO_Port;
    mylcd.data_pin_ports[0] = LCD_D4_GPIO_Port;

    mylcd.data_pin_pins[3] = LCD_D7_Pin;
    mylcd.data_pin_pins[2] = LCD_D6_Pin;
    mylcd.data_pin_pins[1] = LCD_D5_Pin;
    mylcd.data_pin_pins[0] = LCD_D4_Pin;
    
    mylcd.RS_port = LCD_RS_GPIO_Port;
    mylcd.RS_pin = LCD_RS_Pin;

    mylcd.E_port = LCD_E_GPIO_Port;
    mylcd.E_pin = LCD_E_Pin;
}

void init_LCD(void)
{
    HAL_Delay(30);			// Allow the LCD some power up time (~30ms)

    volatile uint32_t freq = HAL_RCC_GetHCLKFreq();
    //Therefore 1us is 
    mylcd.nops_per_ten_us = ((freq/100000)/CYCLES_PER_NOP_IN_FOR);

    /*
    mylcd.size[0] = 16;
    mylcd.size[1] = 2;

    mylcd.cursor[0] = 0;
    mylcd.cursor[1] = 0;

    mylcd.position[0] = 0;
    mylcd.position[1] = 0;
    */

    lcd_command(POWER_UP);		// Power up initialization for the lcd
    lcd_command(FOURBIT_MODE);		// Set LCD into 4 bit mode
    lcd_command(DISPLAY_ON);		// Turn display on and set up cursor
    lcd_command(TWOLINE_MODE);		// Set up 2 lines and character size
    lcd_command(CLEAR);			// Clear display
}

//====================================================================
// WRITE A SINGLE CHARACTER TO THE LCD - LCD_PutChar(character)
//====================================================================
// DESCRIPTION: Puts a single character on the LCD at the next position
//              on the screen. The character to be printed is in the input
//              parameter. For numbers, letters and other common characters
//              the ASCII code will produce correct display.
//
//              Refer to the Hitachi HD44780 datasheet for full character
//              set information.
//====================================================================

void lcd_putchar(unsigned char character)
{
    HAL_GPIO_WritePin(mylcd.RS_port,mylcd.RS_pin,1);//	GPIOC->BSRR |= LCD_RS_SET;	// Register Select (RS) line HIGH (data sent will now be read as text);
    write_byte(character);
    //mylcd.position[mylcd.cursor[1]]++;
}

/*
void lcd_clear_screen(void) {
    lcd_command(CLEAR);
    mylcd.position[0] = 0;
    mylcd.position[1] = 0;

    lcd_cursor_home();
}

void lcd_cursor_home(void) {
    lcd_command(CURSOR_HOME);
    mylcd.cursor[0] = 0;
    mylcd.cursor[1] = 0;
}

void lcd_line_two(void) {
    lcd_command(LINE_TWO);
    mylcd.cursor[1] = 0;
    mylcd.cursor[1] = 1;
}

*/

//====================================================================
// WRITE A STRING TO THE LCD - LCD_PutString(ptr_String)
//====================================================================
// DESCRIPTION: Writes a string to the LCD
//====================================================================

void lcd_putstring(char *instring) {
    unsigned char count = 0;

    while (instring[count]) { // Until the null terminator is reached
    	lcd_putchar(instring[count]);	// Write each character to LCD
	    count++;
	}
}


//====================================================================
// PULSE STROBE - Pulse_Strobe()
//====================================================================
// DESCRIPTION: Pulse the strobe line of the LCD to indicate that data is ready.
//====================================================================

void pulse_strobe(void)
{    
    delay(2);				// Delay
    HAL_GPIO_WritePin(mylcd.E_port,mylcd.E_pin,1);
    //GPIOC->BSRR |= LCD_EN_SET;		// pull E (PC15) HIGH

    delay(2);				// Delay
    HAL_GPIO_WritePin(mylcd.E_port,mylcd.E_pin,0);
    //GPIOC->BSRR |= LCD_EN_RESET;	// Take EN LOW

    delay(2);				// Delay
    HAL_GPIO_WritePin(mylcd.E_port,mylcd.E_pin,1);
    //GPIOC->BSRR |= LCD_EN_SET;		// Take EN HIGH
}


//====================================================================
// LOOP DELAY - delay(microseconds)
//====================================================================
// DESCRIPTION: A delay used by the LCD functions.
//====================================================================

void delay(unsigned int tens_of_microseconds) {
    
    volatile unsigned int counter;
    for(counter = 0; counter<(tens_of_microseconds*mylcd.nops_per_ten_us); counter++)
    {
        __asm("nop");
    }
   //HAL_Delay(1);
}

int8_t check_bit(uint8_t value,uint8_t bit_number) {
    if(bit_number > 7 || bit_number < 0) {
        return -1;
    }
    if ((value & (1<<bit_number)) != 0) {
        return 1;
    }
    return 0;
}

void write_byte(uint8_t byte_in) {
    for (int8_t i = 7;i>=4;i--) {
        if (check_bit(byte_in,i)) { // Select bit 7-4 of byte_in, if HIGH set Data line 
            HAL_GPIO_WritePin(mylcd.data_pin_ports[i-4],mylcd.data_pin_pins[i-4],1);
        } else {// else RESET
            HAL_GPIO_WritePin(mylcd.data_pin_ports[i-4],mylcd.data_pin_pins[i-4],0);
        }
    }
    pulse_strobe ();		// Send data
    for (int8_t i = 3;i>=0;i--) {
        if (check_bit(byte_in,i)) { // Select bit 3-0 of command, if HIGH set Data line 
            HAL_GPIO_WritePin(mylcd.data_pin_ports[i],mylcd.data_pin_pins[i],1);
        } else {// else RESET
            HAL_GPIO_WritePin(mylcd.data_pin_ports[i],mylcd.data_pin_pins[i],0);
        }
    }
    pulse_strobe();			// Send data
}

//********************************************************************
// END OF PROGRAM
//********************************************************************

