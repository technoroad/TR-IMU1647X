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
extern BoardParameterList gBoard;

/**
 * @brief  Set the board from the value of the configuration switch.
 */
void CheckConfigrationSwitch() {
  StringFormat format = 0;
  if (READ_CSW1) {
    format |= CSW1_MASK;
  }
  if (READ_CSW2) {
    format |= CSW2_MASK;
  }
  if (READ_CSW3) {
    format |= CSW3_MASK;
  }

  if (format == kADI_DRIVER) {
    EnableAdiDriver();
    DisableStringCommand();
  } else {
    DisableAdiDriver();
    EnableStringCommand();
  }

  SetOutputFormat(format);

  if (READ_CSW4) {
    StartupWaitDisable();
  } else {
    StartupWaitEnable();
  }

  if (READ_CSW5) {
    SelectUsartForCommunication();
    USART_TransmitDmaEnable();
    USART_ReceiveDmaEnable();
  } else {
    SelectUsbForCommunication();
  }
}

/**
 * @brief  Reads the saved settings from the flash area.
 * @note  If there is more than one checksum or
 *        the version is different, the initial value is called.
 */
void ReadParameterFromFlash() {
  uint32_t size = sizeof(gBoard);

  // Read parameters from FLASH memory.
  LoadFlash(DATA_ADDR, (uint8_t*) &gBoard, size);
  uint16_t csum = Make16bitChecksum((uint8_t*) &gBoard, size - CSUM_SIZE);

  // If the version is changed or the CSUM value does not match,
  // the parameter is reset to the initial value.
  if ((gBoard.version != VERSION) || (csum != gBoard.csum)) {
    InitializeParameters();
    WriteFlash(DATA_ADDR, (uint16_t*) &gBoard, size);
  }
}

#endif
