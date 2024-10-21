#ifndef LCD_STM32F0_H
#define LCD_STM32F0_H
//********************************************************************
//*                         STM32F0                                  *
//*                         LCD HEADER                               *
//*==================================================================*
//* WRITTEN BY:    J. Pead                                           *
//* MODIFIED:      20-09-2024                                        *
//*==================================================================*

#include "stm32f4xx_hal.h"

//====================================================================
// GLOBAL CONSTANTS - LCD command codes
//====================================================================
#define    CLEAR           0x01
#define    CURSOR_HOME     0x02
#define    DISPLAY_ON      0x0C
#define    DISPLAY_OFF     0x08
#define    LINE_TWO        0xC0

#define    POWER_UP        0x33
#define    FOURBIT_MODE    0X32
#define    TWOLINE_MODE    0x28

#define    CYCLES_PER_NOP_IN_FOR     16

typedef struct {
   GPIO_TypeDef* data_pin_ports[4];
   uint16_t data_pin_pins[4];
   GPIO_TypeDef* RS_port;
   uint16_t RS_pin;
   GPIO_TypeDef* E_port;
   uint16_t E_pin;
   uint32_t nops_per_ten_us;
   /* This turned out to be pointless, as I cannot do this without knowing whats on my screen, 
   but then i need screen size etc
   uint8_t size[2];
   uint8_t position[2];
   uint8_t cursor[2];
   */
} lcd;

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_LCD(void);

void lcd_command(unsigned char command);
void lcd_putchar(unsigned char character);
void lcd_putstring(char *instring);

/*
These methods were made to help move to positions, but useless.
void lcd_clear_screen(void);
void lcd_cursor_home(void);
void lcd_line_two(void);
*/

void delay(unsigned int microseconds);
void write_byte(uint8_t byte_in);
int8_t check_bit(uint8_t value,uint8_t bitNumber);
void pulse_strobe(void);

//====================================================================

#endif

//********************************************************************
// END OF PROGRAM
//********************************************************************
