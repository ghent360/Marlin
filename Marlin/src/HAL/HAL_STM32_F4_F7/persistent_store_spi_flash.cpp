/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Copyright (c) 2019 Venelin EFremov
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

/*
 * EEPROM emulation over SPI flash. Because SPI_FLASH has relatively low 
 * erase/write count, this code would do very simple wear leveling over
 * the entire chip.
 * 
 * This code uses the SPIMemory arduino library.
 */
#include "../HAL.h"
#if (HAL_PLATFORM_ID == HAL_ID_STM32_F4_F7)

#include "../../inc/MarlinConfig.h"

#if ENABLED(EEPROM_SETTINGS) && ENABLED(SPI_FLASH)
#include "../shared/persistent_store_api.h"
// The SPIMemory lib requires this define to enable proper constructors.
#define ARCH_STM32
#include <SPIMemory.h>

//#define SPI_FLASH_DEBUG

#ifdef SPI_FLASH_DEBUG
#include "../../core/serial.h"
#endif

#ifndef E2END
  // 4KB - 8 bytes
  #define E2END (0xFFF - 8)
#endif
#define HAL_EEPROM_SIZE (E2END + 1)

// 'EEMU'
#define EEPROM_MARKER 0x45454D55

class EEPromOverFlash {
public:
    EEPromOverFlash(
      SPIFlash& storage,
      uint32_t maxDataSize,
      uint32_t chipCapacity = -1)
      : storage_(storage),
        maxDataSize_(maxDataSize),
        latestDataAddress_(-1),
        oldestDataAddress_(-1),
        chipCapacity_(chipCapacity),
        eepromSectorSize_(-1),
        latestVersion_(0) {
    }

    bool begin() {
      storage_.begin();
      uint32_t JEDEC = storage_.getJEDECID();
      if (!JEDEC) {
#ifdef SPI_FLASH_DEBUG
        SERIAL_ECHOLN("Unable to communicate with the SPI flash chip.");
#endif      
        return false;
      }
      
      if (chipCapacity_ > storage_.getCapacity()) {
        chipCapacity_ = storage_.getCapacity();
      }
      if (maxDataSize_ >= chipCapacity_ / 2) {
        return false;
      }
      eepromSectorSize_ = (8 + maxDataSize_ + 4095) / 4096;
      eepromSectorSize_ *= 4096;
      walkChip();
      return true;
    }

    bool readData(uint8_t* buffer, uint32_t size) {
      if (eepromSectorSize_ >= chipCapacity_ / 2) {
        return false;
      }
      if (latestDataAddress_ >= chipCapacity_ || size > maxDataSize_) {
        // No data was ever written
        return false;
      }
#ifdef SPI_FLASH_DEBUG
      SERIAL_ECHO("Reading ");
      SERIAL_ECHO(size);
      SERIAL_ECHO(" data bytes from ");
      SERIAL_PRINTLN(latestDataAddress_, HEX);
#endif      
      storage_.readByteArray(latestDataAddress_ + 8, buffer, size);
      return true;
    }

    void writeData(uint8_t* buffer, uint32_t size) {
      if (size > maxDataSize_ 
          || eepromSectorSize_ >= chipCapacity_ / 2) {
        return;
      }
      if (!verifyNewData(buffer, size)) {
        return;
      }
      latestVersion_++;
      for(uint32_t eraseAddress = 0;
          eraseAddress < eepromSectorSize_;
          eraseAddress += 4096) {
#ifdef SPI_FLASH_DEBUG    
        SERIAL_ECHO("Erasing sector at ");
        SERIAL_PRINTLN(oldestDataAddress_ + eraseAddress, HEX);
#endif
        storage_.eraseSector(oldestDataAddress_ + eraseAddress);
      }
#ifdef SPI_FLASH_DEBUG    
      SERIAL_ECHO("Writing ");
      SERIAL_ECHO(size);
      SERIAL_ECHO(" data bytes to ");
      SERIAL_PRINTLN(oldestDataAddress_, HEX);
#endif
      storage_.writeULong(oldestDataAddress_, EEPROM_MARKER);
      storage_.writeULong(oldestDataAddress_ + 4, latestVersion_);
      storage_.writeByteArray(oldestDataAddress_ + 8, buffer, size);
      walkChip();
    }

    uint32_t getMaxDataSize() const {
      return maxDataSize_;
    }
private:
    void walkChip() {
      latestVersion_ = 0;
      uint32_t oldestVersion = -1;
      latestDataAddress_ = -1;
      oldestDataAddress_ = 0;
      for(uint32_t address = 0;
          address < chipCapacity_;
          address += eepromSectorSize_) {
        uint32_t marker = storage_.readULong(address);
        uint32_t ver = storage_.readULong(address + 4);
        if (marker == EEPROM_MARKER) {
          if (ver > latestVersion_) {
            latestVersion_ = ver;
            latestDataAddress_ = address;
          }
          if (ver < oldestVersion) {
            oldestVersion = ver;
            oldestDataAddress_ = address;
          }
        } else {
          // Unformatted sector - thread as oldest data
          oldestVersion = 0;
          oldestDataAddress_ = address;
        }
      }
#ifdef SPI_FLASH_DEBUG    
      SERIAL_ECHO("Latest data version ");
      SERIAL_ECHO(latestVersion_);
      SERIAL_ECHO(" at ");
      SERIAL_PRINTLN(latestDataAddress_, HEX);
      SERIAL_ECHO("Oldest data at ");
      SERIAL_PRINTLN(oldestDataAddress_, HEX);
#endif
    }

    bool verifyNewData(uint8_t* buffer, uint32_t size) {
      if (latestDataAddress_ >= chipCapacity_) {
        // No data was ever written
        return true;
      }
      static uint8_t dataBuffer[64];
      uint32_t readSize = sizeof(dataBuffer);
      for(uint32_t readAddress = latestDataAddress_ + 8;
          size != 0;
          size -= readSize) {
        if (size < readSize) {
          readSize = size;
        }
        storage_.readByteArray(readAddress, dataBuffer, readSize);
        if (memcmp(buffer, dataBuffer, readSize)) {
#ifdef SPI_FLASH_DEBUG    
          SERIAL_ECHO("Data difference found at ");
          SERIAL_PRINTLN(readAddress, HEX);
#endif
          return true;
        }
        readAddress += readSize;
        buffer += readSize;
      }
      return false;
    }

    SPIFlash& storage_;
    const uint32_t maxDataSize_;
    uint32_t latestDataAddress_;
    uint32_t oldestDataAddress_;
    uint32_t chipCapacity_;
    uint32_t eepromSectorSize_;
    uint32_t latestVersion_;
};

static uint8_t HAL_eeprom_data[HAL_EEPROM_SIZE];
static SPIFlash flash(SPI_FLASH_CS_PIN, &SPI);
static EEPromOverFlash eeprom(flash, HAL_EEPROM_SIZE);
static bool eepromDataLoaded = false;
static bool eepromDataModified = false;

bool PersistentStore::access_start() {
  if (eepromDataLoaded) {
    return true;
  }
  if (!eeprom.begin()) {
    return false;
  }
  if (!eeprom.readData(HAL_eeprom_data, sizeof(HAL_eeprom_data))) {
    memset(HAL_eeprom_data, 0xff, sizeof(HAL_eeprom_data));
  }
  eepromDataLoaded = true;
  eepromDataModified = false;
  return true;
}

bool PersistentStore::access_finish() {
  if (eepromDataModified) {
    eeprom.writeData(HAL_eeprom_data, sizeof(HAL_eeprom_data));
    eepromDataModified = false;
  }
  return true;
}

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  for (size_t i = 0; i < size; i++)
    HAL_eeprom_data[pos + i] = value[i];
  crc16(crc, value, size);
  pos += size;
  eepromDataModified = true;
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t* value, const size_t size, uint16_t *crc, const bool writing/*=true*/) {
  for (size_t i = 0; i < size; i++) {
    uint8_t c = HAL_eeprom_data[pos + i];
    if (writing) value[i] = c;
    crc16(crc, &c, 1);
  }
  pos += size;
  return false;
}

size_t PersistentStore::capacity() {
  return HAL_EEPROM_SIZE;
}

#endif // EEPROM_SETTINGS

#endif // __STM32F1__
