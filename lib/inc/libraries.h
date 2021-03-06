/*
The MIT License (MIT)

Copyright (c) 2019 Techno Road Inc.

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

#include "stm32f0xx_hal.h"
#include "def.h"

#include "initial_parameter.h"

#include "usbd_cdc_if.h"
#include "delay.h"

#include "str_edit.h"
#include "usbprint.h"
#include "USART_CTRL.h"
#include "usb_uart_select.h"

#include "spi.h"
#include "select_sensor.h"

#include "mkae_filter.h"
#include "BinaryCmdParse.h"
#include "StringCmdParse.h"

#include "app.h"

#include "flash_wr.h"
#include "FlashDataStruct.h"
