/*******************************************************************************
* Filename:   main.c
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

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "platform.h"
#include "xil_exception.h"
#include "xparameters.h"
#include "xstatus.h"

#define KBYTE 1024

/*
 * Functions Declaration
 */
int setup_acceleration_scheduler_sg();
int setup_dma_sg_schedulers();
int setup_acceleration_schedulers_direct();
int setup_acceleration_schedulers_indirect();
int setup_fetch_scheduler();
int setup_send_scheduler();
int setup_scheduler_buffers();
int setup_cdmas();
int setup_dmas();
int setup_apms();
int setup_shared_apm();
int setup_gpio();
int setup_pcie();
int setup_sobel_filters();
int setup_interrupt_manager();
int setup_interrupts();

//The Base Address of the FPGA's BRAM (256K).
int *bram_base_address = (int *)XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR;

int main()
{
	int repeat;

	//Clear the Terminal Screen.
    xil_printf("%c[2J",27);

    //Initialize the Platform.
    init_platform();

    //Clear the FPGA's BRAM.
    for(repeat = 0; repeat < (256 * KBYTE) / 4; repeat++)
    {
    	bram_base_address[repeat] = 0;
    }

    /*
     * Setup ALL the Peripherals of the FPGA.
     */
    setup_acceleration_schedulers_direct();
    setup_acceleration_schedulers_indirect();
    setup_fetch_scheduler();
    setup_send_scheduler();
    setup_scheduler_buffers();
    setup_cdmas();
    setup_dmas();
    setup_apms();
    setup_shared_apm();
    setup_gpio();
    setup_pcie();
    setup_sobel_filters();
    setup_acceleration_scheduler_sg();
    setup_dma_sg_schedulers();
    setup_interrupt_manager();

    //Setup the Interrupt Controller and the Interrupts.
    setup_interrupts();

    print("\r\n-->System is Ready\r\n");


    //Start an Infinite Loop to Keep the System Alive.
	while(1)
	{

	}

    return XST_SUCCESS;
}


