/*******************************************************************************
* Filename:   interrupt_manager.h
* Author:     Dimitrios Bakoyiannis <d.bakoyiannis@gmail.com>
* License:
*
* MIT License
*
* Copyright (c) [2018] [Dimitrios Bakoyiannis]
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
********************************************************************************/

/*
 * ---------------------------------------------------
 * Registers and Offsets of the Xilinx GPIO Peripheral
 * ---------------------------------------------------
 */

#define XGPIO_CHANNEL_1_OFFSET  0x0 // GPIO Channel 1 Base Offset.
#define XGPIO_CHANNEL_2_OFFSET  0x8 // GPIO Channel 2 Base Offset.

/*
 * GPIO Channel 1 Data Register.
 *
 * The Data Register of GPIO Channel 2 is XGPIO_DATA_OFFSET + XGPIO_CHANNEL_2_OFFSET.
 */
#define XGPIO_DATA_OFFSET  0x0

