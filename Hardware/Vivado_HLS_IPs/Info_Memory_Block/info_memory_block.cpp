/*******************************************************************************
* Filename:   info_memory_block.cpp
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ap_int.h"
#include "ap_utils.h"
#include "ap_cint.h"
#include "ap_utils.h"
#include "ap_int.h"
#include "info_memory_block.h"


/*
 * info_memory_block()
 *
 * The Hardware Funtionality of the Info Memory Block Core.
 *
 * The Info Memory Block Core is Used to Aid the Acceleration Procedure of the Acceleration Groups Indirect (AGIs).
 * It is Accessed by the Acceleration Scheduler Indirect Cores of the AGIs as well as the Fetch and Send Schedulers.
 *
 * It Could be Considered as a Block of 16 Registers.
 * The Registers are Categorized in 4 Groups/Sets with 4 Registers in each Group/Set.
 *
 * Every Set of Registers Refers to one of the 4 AGIs.
 *
 * Set 0 Refers to AGI0.
 * Set 1 Refers to AGI1.
 * Set 2 Refers to AGI2.
 * Set 3 Refers to AGI3.
 *
 * The 4 Registers of Each Set Carry the Following Information:
 *
 * Register 0: Source Address.
 * Register 1: Destination Address.
 * Register 2: Data Size (Transfer Size).
 * Register 3: Address Offset.
 *
 * If an Acceleration Scheduler Indirect Requests a CDMA Transfer it Writes the Information Above to its own Set of Registers inside the Info Memory Block.
 * The Fetch or Send Scheduler Reads the Above Information from the Info Memory Block and Starts a CDMA Transfer Accordingly.
 *
 * The Function Parameters are the Input Ports/Interfaces of the Core:
 *
 * 01 to 16 --> Registers of the Core that are Accessed through the AXI Slave Lite Interface of the Core.
 */
int info_memory_block(/*01*/unsigned int src_address_0,
                     /*02*/unsigned int dst_address_0,
                     /*03*/unsigned int data_size_0,
                     /*04*/unsigned int offset_0,
                     /*05*/unsigned int src_address_1,
                     /*06*/unsigned int dst_address_1,
                     /*07*/unsigned int data_size_1,
                     /*08*/unsigned int offset_1,
                     /*09*/unsigned int src_address_2,
                     /*10*/unsigned int dst_address_2,
                     /*11*/unsigned int data_size_2,
                     /*12*/unsigned int offset_2,
                     /*13*/unsigned int src_address_3,
                     /*14*/unsigned int dst_address_3,
                     /*15*/unsigned int data_size_3,
                     /*16*/unsigned int offset_3
                     )
{

/*
 * Source Address, Destination Address, Data Size and Address Offset Registers of the First Group/Set
 */
#pragma HLS INTERFACE  s_axilite  port=src_address_0 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=dst_address_0 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=data_size_0 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=offset_0 bundle=int_cfg

/*
 * Source Address, Destination Address, Data Size and Address Offset Registers of the Second Group/Set
 */
#pragma HLS INTERFACE  s_axilite  port=src_address_1 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=dst_address_1 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=data_size_1 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=offset_1 bundle=int_cfg

/*
 * Source Address, Destination Address, Data Size and Address Offset Registers of the Third Group/Set
 */
#pragma HLS INTERFACE  s_axilite  port=src_address_2 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=dst_address_2 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=data_size_2 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=offset_2 bundle=int_cfg

/*
 * Source Address, Destination Address, Data Size and Address Offset Registers of the Fourth Group/Set
 */
#pragma HLS INTERFACE  s_axilite  port=src_address_3 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=dst_address_3 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=data_size_3 bundle=int_cfg
#pragma HLS INTERFACE  s_axilite  port=offset_3 bundle=int_cfg

#pragma HLS INTERFACE  s_axilite  port=return bundle=int_cfg


return 1;

}


