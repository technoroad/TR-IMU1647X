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
#define SPIx SPI1
#define TIMEOUT (1000-1)  // us

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t kTimeoutCnt = 0;

/* Private function prototypes -----------------------------------------------*/
static bool SPI_TimeoutReporter();

/* Global variables ----------------------------------------------------------*/

/**
 * @brief  Sets the SPI speed.
 */
void SPI_SetBaudRate(uint32_t ll_baud){
  LL_SPI_SetBaudRatePrescaler(SPIx,ll_baud);
}

/**
 * @brief  Write and read at the same time with SPI.
 */
void SPI_WriteReceive(uint16_t *wb, uint16_t *rb,uint16_t length) {
  if (!LL_SPI_IsEnabled(SPIx)) {
    LL_SPI_Enable(SPIx);
  }

  for (int i = 0; i < length; i++) {
    SPI_ENABLE_CHIPSELECT;
    LL_SPI_TransmitData16(SPIx, wb[i]);
    kTimeoutCnt = 0;
    while (LL_SPI_IsActiveFlag_BSY(SPIx)) {
      if (SPI_TimeoutReporter()) {
        SetSystemError(kSpiTimeoutError);
        break;
      }
    }
    rb[i] = LL_SPI_ReceiveData16(SPIx);
    SPI_DISABLE_CHIPSELECT;

    DelayMicrosecond(TSTALL);
  }
}

/**
 * @brief  Write and read at the same time with SPI.
 */
void SPI_WriteReceiveBurstRead(uint16_t *wb, uint16_t *rb,uint16_t length) {
  if (!LL_SPI_IsEnabled(SPIx)) {
    LL_SPI_Enable(SPIx);
  }
  SPI_ENABLE_CHIPSELECT;
  for (int i = 0; i < length; i++) {
    LL_SPI_TransmitData16(SPIx, wb[i]);
    kTimeoutCnt = 0;
    while (LL_SPI_IsActiveFlag_BSY(SPIx)) {
      if (SPI_TimeoutReporter()) {
        SetSystemError(kSpiTimeoutError);
        break;
      }
    }
    rb[i] = LL_SPI_ReceiveData16(SPIx);
  }
  SPI_DISABLE_CHIPSELECT;
}

/**
 * @brief  Counts the SPI wait time and returns the timeout flag.
 */
bool SPI_TimeoutReporter(){
  if(kTimeoutCnt > TIMEOUT){
    kTimeoutCnt = 0;
    return true;
  }else{
    DelayMicrosecond(1);
    kTimeoutCnt++;
    return false;
  }
}

#endif
