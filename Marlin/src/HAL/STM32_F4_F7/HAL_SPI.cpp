/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Adapted to the Marlin STM32F4/7 HAL
 */

#include "../HAL.h"
#if (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)

#include "../../inc/MarlinConfig.h"

#include <SPI.h>
#include "../shared/HAL_SPI.h"
#include "spi_pins.h"

// ------------------------
// Public Variables
// ------------------------

SPISettings spiConfig;

// ------------------------
// Public functions
// ------------------------

#if ENABLED(SOFTWARE_SPI)
  #error "Software SPI not supported for STM32F4/7. Use Hardware SPI."
#endif

// ------------------------
// Hardware SPI
// ------------------------
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
SPIClass SPI_2;
SPISettings spiConfig2;
#endif

/**
 * @brief  Begin SPI port setup
 *
 * @return Nothing
 */
void spiBegin(SPIClass& spiInstance) {
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiInstance.setMOSI(MOSI_PIN);
    spiInstance.setMISO(MISO_PIN);
    spiInstance.setSCLK(SCK_PIN);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiInstance.setMOSI(SPI2_MOSI_PIN);
    spiInstance.setMISO(SPI2_MISO_PIN);
    spiInstance.setSCLK(SPI2_SCK_PIN);
  }
#endif
}

/** Configure SPI for specified SPI speed */
void spiInit(uint8_t spiRate, SPIClass& spiInstance) {
  // Use datarates Marlin uses
  uint32_t clock;
  switch (spiRate) {
    case SPI_FULL_SPEED:    clock = 20000000; break; // 13.9mhz=20000000  6.75mhz=10000000  3.38mhz=5000000  .833mhz=1000000
    case SPI_HALF_SPEED:    clock =  5000000; break;
    case SPI_QUARTER_SPEED: clock =  2500000; break;
    case SPI_EIGHTH_SPEED:  clock =  1250000; break;
    case SPI_SPEED_5:       clock =   625000; break;
    case SPI_SPEED_6:       clock =   300000; break;
    default:                clock =  4000000; // Default from the SPI libarary
  }
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiConfig = SPISettings(clock, MSBFIRST, SPI_MODE0);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiConfig2 = SPISettings(clock, MSBFIRST, SPI_MODE0);
  }
#endif
  spiInstance.begin();
}

/**
 * @brief  Receives a single byte from the SPI port.
 *
 * @return Byte received
 *
 * @details
 */
uint8_t spiRec(SPIClass& spiInstance) {
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiInstance.beginTransaction(spiConfig);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiInstance.beginTransaction(spiConfig2);
  }
#endif    
  uint8_t returnByte = SPI.transfer(0xFF);
  spiInstance.endTransaction();
  return returnByte;
}

/**
 * @brief  Receives a number of bytes from the SPI port to a buffer
 *
 * @param  buf   Pointer to starting address of buffer to write to.
 * @param  nbyte Number of bytes to receive.
 * @return Nothing
 *
 * @details Uses DMA
 */
void spiRead(uint8_t* buf, uint16_t nbyte, SPIClass& spiInstance) {
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiInstance.beginTransaction(spiConfig);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiInstance.beginTransaction(spiConfig2);
  }
#endif    
  #ifdef STM32GENERIC
    spiInstance.dmaTransfer(0, const_cast<uint8_t*>(buf), nbyte);
  #else
    spiInstance.transfer(buf, nbyte);
  #endif
  spiInstance.endTransaction();
}

/**
 * @brief  Sends a single byte on SPI port
 *
 * @param  b Byte to send
 *
 * @details
 */
void spiSend(uint8_t b, SPIClass& spiInstance) {
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiInstance.beginTransaction(spiConfig);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiInstance.beginTransaction(spiConfig2);
  }
#endif    
  spiInstance.transfer(b);
  spiInstance.endTransaction();
}

/**
 * @brief  Write token and then write from 512 byte buffer to SPI (for SD card)
 *
 * @param  buf   Pointer with buffer start address
 * @return Nothing
 *
 * @details Use DMA
 */
void spiSendBlock(uint8_t token, const uint8_t* buf, SPIClass& spiInstance) {
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiInstance.beginTransaction(spiConfig);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiInstance.beginTransaction(spiConfig2);
  }
#endif    
  spiInstance.transfer(token);
  #ifdef STM32GENERIC
    spiInstance.dmaSend(const_cast<uint8_t*>(buf), 512);
  #else
    spiInstance.transfer((uint8_t*)buf, nullptr, 512);
  #endif
  spiInstance.endTransaction();
}

#if ENABLED(SPI_EEPROM)

// Write buffer to specified SPI channel
void spiSend(const uint8_t* buf, size_t n, SPIClass& spiInstance) {
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  if (&spiInstance == &SPI) {
#endif    
    spiInstance.beginTransaction(spiConfig);
#if PIN_EXISTS(SPI2_MOSI) && PIN_EXISTS(SPI2_MISO) && PIN_EXISTS(SPI2_SCK)
  } else {
    spiInstance.beginTransaction(spiConfig2);
  }
#endif    
  #ifdef STM32GENERIC
    spiInstance.dmaSend(const_cast<uint8_t*>(buf), n);
  #else
    spiInstance.transfer((uint8_t*)buf, nullptr, n);
  #endif
  spiInstance.endTransaction();
}

#endif // SPI_EEPROM
#endif // (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)
