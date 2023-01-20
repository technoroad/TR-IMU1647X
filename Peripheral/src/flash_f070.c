/*
 The MIT License (MIT)

 Copyright (c) 2023 Techno Road Inc.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */
#if defined(STM32F070xB)

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/*
 *@brief erase sector11
 */
void EraseFlash(void) {
  FLASH_EraseInitTypeDef erase;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;  // select pages
  erase.PageAddress = DATA_ADDR;  // set selector7
  erase.NbPages = 1;  // set to erase one sector

  uint32_t pageError = 0;

  HAL_FLASHEx_Erase(&erase, &pageError);  // erase sector
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint16_t * data write data
 * @param uint32_t size write data size
 */
void WriteFlash(uint32_t address, uint16_t *data, uint32_t size) {
  HAL_FLASH_Unlock(); /* Unlock flash */
  EraseFlash(); /* Erase sector 7 */
  do {
    /* Write to the flash one byte at a time */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data);
    address += 2;
    data += 1;
    size -= 2;
  } while (size);
  HAL_FLASH_Lock(); /* Lock flash */
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
 */
void LoadFlash(uint32_t address, uint8_t *data, uint32_t size) {
  memcpy(data, (uint8_t*) address, size);
}

#endif
