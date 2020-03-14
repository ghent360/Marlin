/*
  
  u8g_com_stm32_st7920_spi.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

  A special SPI interface for ST7920 controller

  Update for ATOMIC operation done (01 Jun 2013)
    U8G_ATOMIC_OR(ptr, val)
    U8G_ATOMIC_AND(ptr, val)
    U8G_ATOMIC_START();
    U8G_ATOMIC_END();


*/

#include "../HAL.h"
#if (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)

#include <U8glib.h>
#include <Arduino.h>
#include "../../../inc/MarlinConfig.h"

static GPIO_TypeDef *mosiPort;
static uint32_t mosiPin;
static GPIO_TypeDef *sckPort;
static uint32_t sckPin;

static inline void u8g_com_stm32_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  mosiPort = digitalPinToPort(dataPin);
  mosiPin = STM_LL_GPIO_PIN(digitalPinToPinName(dataPin));
  sckPort = digitalPinToPort(clockPin);
  sckPin = STM_LL_GPIO_PIN(digitalPinToPinName(clockPin));
}

static inline void u8g_com_stm32_do_shift_out_msb_first(uint8_t b) {
  for (uint8_t i = 0; i < 8;) {
    if (b & 0x80) {
      LL_GPIO_SetOutputPin(mosiPort, mosiPin);
    } else {
      LL_GPIO_ResetOutputPin(mosiPort, mosiPin);
    }
    b <<= 1;
    LL_GPIO_SetOutputPin(sckPort, sckPin);
    i++;
    u8g_MicroDelay();
    LL_GPIO_ResetOutputPin(sckPort, sckPin);
    u8g_MicroDelay();
  }
}

static inline void u8g_com_stm32_st7920_write_byte_seq(uint8_t rs, uint8_t *ptr, uint8_t len)
{
  u8g_com_stm32_do_shift_out_msb_first((rs == 0) ? 0x0f8 : 0xfa);

  while( len > 0 ) {
    u8g_com_stm32_do_shift_out_msb_first(*ptr & 0x0f0);
    u8g_com_stm32_do_shift_out_msb_first(*ptr << 4);
    ptr++;
    len--;
    u8g_10MicroDelay();
  }
  
  for(uint8_t i = 0; i < 4; i++ )
    u8g_10MicroDelay();
}

static inline void u8g_com_stm32_st7920_write_byte(uint8_t rs, uint8_t val)
{
  u8g_com_stm32_do_shift_out_msb_first((rs == 0) ? 0x0f8 : 0xfa);
  
  u8g_com_stm32_do_shift_out_msb_first(val & 0x0f0);
  u8g_com_stm32_do_shift_out_msb_first(val << 4);
  
  for(uint8_t i = 0; i < 4; i++ )
    u8g_10MicroDelay();
    
}

uint8_t u8g_com_HAL_stm32f4_st7920_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      u8g_com_arduino_assign_pin_output_high(u8g);
      u8g_com_arduino_digital_write(u8g, U8G_PI_CS, LOW);
      // u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, LOW);
      u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, HIGH);
      u8g_com_arduino_digital_write(u8g, U8G_PI_MOSI, LOW);
      u8g_com_stm32_init_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK]);
      u8g->pin_list[U8G_PI_A0_STATE] = 0;       /* inital RS state: command mode */
      break;
    
    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      if ( u8g->pin_list[U8G_PI_RESET] != U8G_PIN_NONE )
      	u8g_com_arduino_digital_write(u8g, U8G_PI_RESET, arg_val);
      break;
      
    case U8G_COM_MSG_CHIP_SELECT:
      if ( arg_val == 0 ) {
        /* disable, note: the st7920 has an active high chip select */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, arg_val);
      } else {
        /* enable */
        //u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, HIGH);
	/* 28 Dec 2013 reassign pins, fixes issue with more than one display */
	/* issue 227 */
	      u8g_com_stm32_init_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK]);
      }
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      u8g_com_stm32_st7920_write_byte( u8g->pin_list[U8G_PI_A0_STATE], arg_val);
      break;
    
    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P:
      u8g_com_stm32_st7920_write_byte_seq(u8g->pin_list[U8G_PI_A0_STATE], (uint8_t *)arg_ptr, arg_val);
      break;

    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g->pin_list[U8G_PI_A0_STATE] = arg_val;
      break;
  }
  return 1;
}

#endif
