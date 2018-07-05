/*******************************************************************************
* Filename:   setup_system.cpp
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


#include "xparameters.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "platform.h"
#include "mb_interface.h"
#include "xintc.h"
#include "xstatus.h"
#include "structures.h"
#include "xaxipcie.h"
#include "xgpio.h"
#include "xaxipmon.h"
#include "xaxidma.h"
#include "xaxicdma.h"
#include "xsobel_filter.h"
#include "xscheduler_buffer.h"
#include "xfetch_scheduler.h"
#include "xsend_scheduler.h"
#include "xacceleration_scheduler_indirect.h"
#include "xacceleration_scheduler_direct.h"
#include "xacceleration_scheduler_sg_xdma.h"
#include "xdma_sg_pcie_scheduler.h"
#include "xinterrupt_manager.h"


/*
 * The Offsets of the MSI-Request-Registers of the Interrupt Manager.
 */
#define MSI_DATA_0_OFFSET 0x00
#define MSI_DATA_1_OFFSET 0x04
#define MSI_DATA_2_OFFSET 0x08
#define MSI_DATA_3_OFFSET 0x0C
#define MSI_DATA_4_OFFSET 0x10
#define MSI_DATA_5_OFFSET 0x14
#define MSI_DATA_6_OFFSET 0x18


/*
 * The Slots of the AXI Performance Monitor Unit.
 */
#define SLOT0	0
#define SLOT1 	1
#define SLOT2	2


/*
 * Command Value to Start the Shared Timer (Shared APM).
 */
#define OPERATION_START_TIMER		0x18000000


/*
 * The Offsets of the Address Translation Upper and Lower Registers of the 6 AXI BARs of the PCIe Bridge.
 */
#define BAR0_OFFSET_L XAXIPCIE_AXIBAR2PCIBAR_0L_OFFSET
#define BAR0_OFFSET_U XAXIPCIE_AXIBAR2PCIBAR_0U_OFFSET

#define BAR1_OFFSET_L XAXIPCIE_AXIBAR2PCIBAR_1L_OFFSET
#define BAR1_OFFSET_U XAXIPCIE_AXIBAR2PCIBAR_1U_OFFSET

#define BAR2_OFFSET_L XAXIPCIE_AXIBAR2PCIBAR_2L_OFFSET
#define BAR2_OFFSET_U XAXIPCIE_AXIBAR2PCIBAR_2U_OFFSET

#define BAR3_OFFSET_L XAXIPCIE_AXIBAR2PCIBAR_3L_OFFSET
#define BAR3_OFFSET_U XAXIPCIE_AXIBAR2PCIBAR_3U_OFFSET

#define BAR4_OFFSET_L XAXIPCIE_AXIBAR2PCIBAR_4L_OFFSET
#define BAR4_OFFSET_U XAXIPCIE_AXIBAR2PCIBAR_4U_OFFSET

#define BAR5_OFFSET_L XAXIPCIE_AXIBAR2PCIBAR_5L_OFFSET
#define BAR5_OFFSET_U XAXIPCIE_AXIBAR2PCIBAR_5U_OFFSET


/*
 * Define KBYTE and MBYTE Macros for Simpler Data Size Calculations.
 */
#define KBYTE 1024
#define MBYTE 1048576


/*
 * The Size of a Memory Page in Linux.
 */
#define PAGE_SIZE	4096


/*
 * Mask to Isolate the Clear Data from a GPIO Data Register.
 */
#define CLEAR_DATA	0x00FFFFFF


/*
 * Mask to Isolate the Operation Number from a GPIO Data Register.
 */
#define CLEAR_OPERATION	0xFF000000


/*
 * Enable or Disable the xil_printf() Functions that Print Debug Messages.
 */
#define DEBUG_MESSAGES 1



/*
 * ----------------------------------------------------------
 * Instances and Configuration Structures of the Sobel Filters
 * ----------------------------------------------------------
 */

static XSobel_filter sobel_filter_accel_group_direct_0; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_direct_0_config_ptr; //Sobel Filter Configuration Structure

static XSobel_filter sobel_filter_accel_group_direct_1; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_direct_1_config_ptr; //Sobel Filter Configuration Structure

static XSobel_filter sobel_filter_accel_group_indirect_0; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_indirect_0_config_ptr; //Sobel Filter Configuration Structure

static XSobel_filter sobel_filter_accel_group_indirect_1; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_indirect_1_config_ptr; //Sobel Filter Configuration Structure

static XSobel_filter sobel_filter_accel_group_indirect_2; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_indirect_2_config_ptr; //Sobel Filter Configuration Structure

static XSobel_filter sobel_filter_accel_group_indirect_3; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_indirect_3_config_ptr; //Sobel Filter Configuration Structure

static XSobel_filter sobel_filter_accel_group_sg; //Sobel Filter Instance
XSobel_filter_Config *sobel_filter_accel_group_sg_config_ptr; //Sobel Filter Configuration Structure


/*
 * --------------------------------------------------
 * Instances and Configuration Structures of the DMAs
 * --------------------------------------------------
 */

static XAxiDma dma_accel_group_direct_0; //DMA Instance
XAxiDma_Config *dma_accel_group_direct_0_config_ptr; //DMA Configuration Structure

static XAxiDma dma_accel_group_direct_1; //DMA Instance
XAxiDma_Config *dma_accel_group_direct_1_config_ptr; //DMA Configuration Structure

static XAxiDma dma_accel_group_indirect_0; //DMA Instance
XAxiDma_Config *dma_accel_group_indirect_0_config_ptr; //DMA Configuration Structure

static XAxiDma dma_accel_group_indirect_1; //DMA Instance
XAxiDma_Config *dma_accel_group_indirect_1_config_ptr; //DMA Configuration Structure

static XAxiDma dma_accel_group_indirect_2; //DMA Instance
XAxiDma_Config *dma_accel_group_indirect_2_config_ptr; //DMA Configuration Structure

static XAxiDma dma_accel_group_indirect_3; //DMA Instance
XAxiDma_Config *dma_accel_group_indirect_3_config_ptr; //DMA Configuration Structure

static XAxiDma dma_accel_group_sg; //DMA Instance
XAxiDma_Config *dma_accel_group_sg_config_ptr; //DMA Configuration Structure


/*
 * --------------------------------------------------
 * Instances and Configuration Structures of the APMs
 * --------------------------------------------------
 */

static XAxiPmon apm_accel_group_direct_0;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_direct_0_config_ptr;	//AXI Performance Monitor Configuration Structure

static XAxiPmon apm_accel_group_direct_1;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_direct_1_config_ptr;	//AXI Performance Monitor Configuration Structure

static XAxiPmon apm_accel_group_indirect_0;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_indirect_0_config_ptr;	//AXI Performance Monitor Configuration Structure

static XAxiPmon apm_accel_group_indirect_1;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_indirect_1_config_ptr;	//AXI Performance Monitor Configuration Structure

static XAxiPmon apm_accel_group_indirect_2;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_indirect_2_config_ptr;	//AXI Performance Monitor Configuration Structure

static XAxiPmon apm_accel_group_indirect_3;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_indirect_3_config_ptr;	//AXI Performance Monitor Configuration Structure

static XAxiPmon apm_accel_group_sg;	//AXI Performance Monitor Instance
XAxiPmon_Config *apm_accel_group_sg_config_ptr;	//AXI Performance Monitor Configuration Structure


/*
 * --------------------------------------------------------
 * Instances and Configuration Structures of the Schedulers
 * --------------------------------------------------------
 */

static XScheduler_buffer scheduler_buffer_fetch; //Scheduler Buffer Instance
XScheduler_buffer_Config *scheduler_buffer_fetch_config_ptr; //Scheduler Buffer Configuration Structure

static XScheduler_buffer scheduler_buffer_send; //Scheduler Buffer Instance
XScheduler_buffer_Config *scheduler_buffer_send_config_ptr; //Scheduler Buffer Configuration Structure


static XFetch_scheduler fetch_scheduler; //Fetch Scheduler Instance
XFetch_scheduler_Config *fetch_scheduler_config_ptr; //Fetch Scheduler Configuration Structure

static XSend_scheduler send_scheduler; //Send Scheduler Instance
XSend_scheduler_Config *send_scheduler_config_ptr; //Send Scheduler Configuration Structure


static XAcceleration_scheduler_indirect acceleration_scheduler_accel_group_indirect_0; //Acceleration Scheduler Indirect Instance
XAcceleration_scheduler_indirect_Config *acceleration_scheduler_accel_group_indirect_0_config_ptr; //Acceleration Scheduler Indirect Configuration Structure

static XAcceleration_scheduler_indirect acceleration_scheduler_accel_group_indirect_1; //Acceleration Scheduler Indirect Instance
XAcceleration_scheduler_indirect_Config *acceleration_scheduler_accel_group_indirect_1_config_ptr; //Acceleration Scheduler Indirect Configuration Structure

static XAcceleration_scheduler_indirect acceleration_scheduler_accel_group_indirect_2; //Acceleration Scheduler Indirect Instance
XAcceleration_scheduler_indirect_Config *acceleration_scheduler_accel_group_indirect_2_config_ptr; //Acceleration Scheduler Indirect Configuration Structure

static XAcceleration_scheduler_indirect acceleration_scheduler_accel_group_indirect_3; //Acceleration Scheduler Indirect Instance
XAcceleration_scheduler_indirect_Config *acceleration_scheduler_accel_group_indirect_3_config_ptr; //Acceleration Scheduler Indirect Configuration Structure



static XAcceleration_scheduler_direct acceleration_scheduler_accel_group_direct_0; //Acceleration Scheduler Direct Instance
XAcceleration_scheduler_direct_Config *acceleration_scheduler_accel_group_direct_0_config_ptr; //Acceleration Scheduler Direct Configuration Structure

static XAcceleration_scheduler_direct acceleration_scheduler_accel_group_direct_1; //Acceleration Scheduler Direct Instance
XAcceleration_scheduler_direct_Config *acceleration_scheduler_accel_group_direct_1_config_ptr; //Acceleration Scheduler Direct Configuration Structure

static XAcceleration_scheduler_sg_xdma acceleration_scheduler_sg; //Acceleration Scheduler Scatter/Gather Instance
XAcceleration_scheduler_sg_xdma_Config *acceleration_scheduler_sg_config_ptr; //Acceleration Scheduler Scatter/Gather Configuration Structure

static XDma_sg_pcie_scheduler dma_sg_pcie_scheduler; //DMA SG PCIe Scheduler Instance
XDma_sg_pcie_scheduler_Config *dma_sg_pcie_scheduler_config_ptr; //DMA SG PCIe Scheduler Configuration Structure


/*
 * ---------------------------------------------------
 * Instances and Configuration Structures of the CDMAs
 * ---------------------------------------------------
 */

static XAxiCdma cdma_fetch; //CDMA Instance
XAxiCdma_Config *cdma_fetch_config_ptr; //CDMA Configuration Structure

static XAxiCdma cdma_send; //CDMA Instance
XAxiCdma_Config *cdma_send_config_ptr; //CDMA Configuration Structure


/*
 * -----------------------------------------------------------------------------------------------------------------------------
 * Instances and Configuration Structures of the Shared APM, the PCIe Bridge, the Interrupt Manager and the Interrupt Controller
 * -----------------------------------------------------------------------------------------------------------------------------
 */

static XAxiPmon shared_apm; //AXI Performance Monitor Instance
XAxiPmon_Config *shared_apm_config_ptr; //AXI Performance Monitor Configuration Structure

static XAxiPcie pcie_ep; //PCI Express Instance
XAxiPcie_Config *pcie_config_ptr; //PCI Express Configuration Structure

static XInterrupt_manager interrupt_manager; //Interrupt Manager Instance
XInterrupt_manager_Config *interrupt_manager_config_ptr; //Interrupt Manager Configuration Structure

static XIntc interrupt_controller; //Interrupt Controller Instance


/*
 * ---------------------------------
 * Instances of the GPIO Peripherals
 * ---------------------------------
 */

static XGpio gpio_msi; //GPIO Instance
static XGpio gpio_msi_read; //GPIO Instance
static XGpio gpio_pcie_interrupt; //GPIO Instance
static XGpio gpio_ack; //GPIO Instance

u32 interrupt_mask; //Used to Enable the Several Interrupts of the FPGA's Peripherals.

/*
 * The Base Address of the FPGA's BRAM.
 * The Pointer is of Type struct shared_repository in order to Access the BRAM as Fields of that Type of Structure.
 */
struct shared_repository *shared_metrics = (struct shared_repository *)XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR;


/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Direct of the AGD0.
 */
void acceleration_scheduler_direct_group_0_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_direct_InterruptClear(&acceleration_scheduler_accel_group_direct_0, 0xFFFFFFFF);

	print("Acceleration Scheduler Direct 0 Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Direct of the AGD0.
	XAcceleration_scheduler_direct_InterruptEnable(&acceleration_scheduler_accel_group_direct_0, 0xFFFFFFFF);
	XAcceleration_scheduler_direct_InterruptGlobalEnable(&acceleration_scheduler_accel_group_direct_0);
}

/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Direct of the AGD1.
 */
void acceleration_scheduler_direct_group_1_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_direct_InterruptClear(&acceleration_scheduler_accel_group_direct_1, 0xFFFFFFFF);

	print("Acceleration Scheduler Direct 1 Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Direct of the AGD1.
	XAcceleration_scheduler_direct_InterruptEnable(&acceleration_scheduler_accel_group_direct_1, 0xFFFFFFFF);
	XAcceleration_scheduler_direct_InterruptGlobalEnable(&acceleration_scheduler_accel_group_direct_1);
}

/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Indirect of the AGI0.
 */
void acceleration_scheduler_indirect_group_0_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_indirect_InterruptClear(&acceleration_scheduler_accel_group_indirect_0, 0xFFFFFFFF);

	print("Acceleration Scheduler Indirect 0 Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Indirect of the AGI0.
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_0, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_0);
}

/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Indirect of the AGI1.
 */
void acceleration_scheduler_indirect_group_1_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_indirect_InterruptClear(&acceleration_scheduler_accel_group_indirect_1, 0xFFFFFFFF);

	print("Acceleration Scheduler Indirect 1 Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Indirect of the AGI1.
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_1, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_1);
}

/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Indirect of the AGI2.
 */
void acceleration_scheduler_indirect_group_2_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_indirect_InterruptClear(&acceleration_scheduler_accel_group_indirect_2, 0xFFFFFFFF);

	print("Acceleration Scheduler Indirect 2 Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Indirect of the AGI2.
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_2, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_2);
}

/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Indirect of the AGI3.
 */
void acceleration_scheduler_indirect_group_3_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_indirect_InterruptClear(&acceleration_scheduler_accel_group_indirect_3, 0xFFFFFFFF);

	print("Acceleration Scheduler Indirect 3 Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Indirect of the AGI3.
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_3, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_3);
}

/*
 * Interrupt Handler for Interrupts Triggered by the Acceleration Scheduler Scatter/Gather of the AGSG.
 */
void acceleration_scheduler_sg_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XAcceleration_scheduler_sg_xdma_InterruptClear(&acceleration_scheduler_sg, 0xFFFFFFFF);

	print("Acceleration Scheduler Scatter/Gather Interrupt\r\n");

	//Re-Enable the Interrupts for the Acceleration Scheduler Scatter/Gather of the AGSG.
	XAcceleration_scheduler_sg_xdma_InterruptEnable(&acceleration_scheduler_sg, 0xFFFFFFFF);
	XAcceleration_scheduler_sg_xdma_InterruptGlobalEnable(&acceleration_scheduler_sg);
}



/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGD0.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_direct_0_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Direct 0 Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGD1.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_direct_1_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Direct 1 Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGI0.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_indirect_0_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Indirect 0 Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGI1.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_indirect_1_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Indirect 1 Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGI2.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_indirect_2_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Indirect 2 Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGI3.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_indirect_3_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Indirect 3 Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the DMA of the AGSG.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void dma_accel_group_sg_interrupt_handler(void * baseaddr_p)
{
	print("DMA Accel Group Scatter/Gather Interrupt\r\n");
}



/*
 * Interrupt Handler for Interrupts Triggered by the DMA-SG-PCIe-Scheduler of the AGSG.
 */
void dma_sg_pcie_scheduler_interrupt_handler(void * baseaddr_p)
{
	//Clear the Interrupt.
	XDma_sg_pcie_scheduler_InterruptClear(&dma_sg_pcie_scheduler, 0xFFFFFFFF);

	print("DMA SG PCIe Scheduler Interrupt\r\n");

	//Re-Enable the Interrupts for the DMA-SG-PCIe-Scheduler.
	XDma_sg_pcie_scheduler_InterruptEnable(&dma_sg_pcie_scheduler, 0xFFFFFFFF);
	XDma_sg_pcie_scheduler_InterruptGlobalEnable(&dma_sg_pcie_scheduler);
}

/*
 * Interrupt Handler for Interrupts Triggered by the CDMA-Fetch.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void cdma_fetch_interrupt_handler(void * baseaddr_p)
{
	print("CDMA Fetch Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the CDMA-Send.
 * There is no Need to Clear or Acknowledge the Interrupt Since it is Done by the Corresponding Scheduler.
 */
void cdma_send_interrupt_handler(void * baseaddr_p)
{
	print("CDMA Send Interrupt\r\n");
}

/*
 * Interrupt Handler for Interrupts Triggered by the GPIO-PCIe-Interrupt Peripheral.
 */
void gpio_pcie_interrupt_handler(void * baseaddr_p)
{
	u32 operation;

	//Disable the Interrupts of the GPIO-PCIe-Interrupt Peripheral so that this Routine will not be Interrupted.
	XGpio_InterruptDisable(&gpio_pcie_interrupt, XPAR_GPIO_PCIE_INTERRUPT_IP2INTC_IRPT_MASK);

	#if DEBUG_MESSAGES == 1
	print("Interrupt from PCIe\r\n");
	#endif

	//Read the Channel 2 Data Register of the GPIO-PCIe-Interrupt Peripheral which Carries Information about the Operation that the Host System Requests.
	operation = XGpio_DiscreteRead(&gpio_pcie_interrupt, 2);

	#if DEBUG_MESSAGES == 1
	xil_printf("Operation is: 0x%08X\r\n", operation);
	#endif

	//Check If the Host System Requested to Start the Shared Timer (Shared APM).
	if((operation & CLEAR_OPERATION) == OPERATION_START_TIMER)
	{
		#if DEBUG_MESSAGES == 1
		xil_printf("Starting Shared Timer\r\n");
		#endif

		//Disable the Global Clock Counter of the Shared Timer (Shared APM).
		XAxiPmon_DisableGlobalClkCounter(&shared_apm);

		//Reset the Global Clock Counter of the Shared Timer (Shared APM) to Start Over.
		XAxiPmon_ResetGlobalClkCounter(&shared_apm);

		//Re-Enable the Global Clock Counter of the Shared Timer (Shared APM).
		XAxiPmon_EnableGlobalClkCounter(&shared_apm);

		//Start the Fetch Scheduler.
		XFetch_scheduler_Start(&fetch_scheduler);

		//Start the Send Scheduler.
		XSend_scheduler_Start(&send_scheduler);
	}

	//Clear the Data Register of the Channel 1 of the GPIO-PCIe-Interrupt Peripheral in Order to Make Sure that the Next Written Data is Valid Information from the Host System.
	XGpio_DiscreteWrite(&gpio_pcie_interrupt, 1, 0x00);


	//Clear the Interrupt of the GPIO-PCIe-Interrupt Peripheral.
	(void)XGpio_InterruptClear(&gpio_pcie_interrupt, XGPIO_IR_MASK);
	(void)XGpio_InterruptClear(&gpio_pcie_interrupt, XPAR_GPIO_PCIE_INTERRUPT_IP2INTC_IRPT_MASK);

	//Re-Enable the Interrupts of the GPIO-PCIe-Interrupt Peripheral.
	XGpio_WriteReg(XPAR_GPIO_PCIE_INTERRUPT_BASEADDR, XGPIO_IER_OFFSET, XGPIO_IR_CH2_MASK) ;
	XGpio_InterruptEnable(&gpio_pcie_interrupt, XPAR_GPIO_PCIE_INTERRUPT_IP2INTC_IRPT_MASK);

}



/*
 * setup_dma_sg_schedulers()
 *
 * Setup Procedure of the DMA-SG-PCIe-Scheduler.
 *
 * The DMA-SG-PCIe-Scheduler Manages the MM2S and S2MM Channels of the DMA to Make Scatter/Gather Transfers in Pages.
 *
 * @note For Details Check the HLS Code of the DMA-SG-PCIe-Scheduler.
 */
int setup_dma_sg_schedulers()
{
	int status;

	print("Set-Up Process for DMA SG PCIe Scheduler\r\n");

	//Set the Configuration Structure of the DMA-SG-PCIe_Scheduler.
	dma_sg_pcie_scheduler_config_ptr = XDma_sg_pcie_scheduler_LookupConfig(XPAR_ACCEL_GROUP_SG_DMA_SG_PCIE_SCHEDULER_DEVICE_ID);

	if (dma_sg_pcie_scheduler_config_ptr == NULL)
	{
		print("Setting-up DMA SG PCIe Scheduler Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA SG PCIe Scheduler Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA-SG-PCIe_Scheduler.
	status = XDma_sg_pcie_scheduler_CfgInitialize(&dma_sg_pcie_scheduler, dma_sg_pcie_scheduler_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing DMA SG PCIe Scheduler Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA SG PCIe Scheduler Instance: SUCCESS\r\n");
	}

	//Set the Base Address of the DMA that is Used by the AGSG.
    XDma_sg_pcie_scheduler_Set_dma_device_address(&dma_sg_pcie_scheduler, (u32)XPAR_ACCEL_GROUP_SG_DMA_BASEADDR);

    //Set the Page Size in Bytes (Usually 4K).
    XDma_sg_pcie_scheduler_Set_page_size(&dma_sg_pcie_scheduler, PAGE_SIZE);

    //Set the Base Address where the Scatter/Gather List for the Source Data is Located.
    XDma_sg_pcie_scheduler_Set_mm2s_sgl_address(&dma_sg_pcie_scheduler, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR + (64 * KBYTE));

    //Set the Base Address where the Scatter/Gather List for the Destination Data is Located.
    XDma_sg_pcie_scheduler_Set_s2mm_sgl_address(&dma_sg_pcie_scheduler, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR + (128 * KBYTE));

    //Set the Base Address of the PCIe Bridge's AXI BAR that is Used to Read the Source Data.
    XDma_sg_pcie_scheduler_Set_axi_bar_src_address(&dma_sg_pcie_scheduler, XPAR_PCIE_AXIBAR_4);

	//Set the Offset in the PCIe Bridge where the Address Translation Register of the AXI BAR that is Accessed by the MM2S Channel of the DMA is Located.
	//This is Required in Order to Configure the Address Translation Register of the AXI BAR
	//with the Physical Address of the Kernel Memory that the MM2S Channel of the DMA will Use as the Source Address to Read the Image Data.
    XDma_sg_pcie_scheduler_Set_axi_bar_src_cfg_address(&dma_sg_pcie_scheduler, (XPAR_PCIE_BASEADDR + BAR4_OFFSET_L));

    //Set the Base Address of the PCIe Bridge's AXI BAR that is Used to Write the Destination Data.
    XDma_sg_pcie_scheduler_Set_axi_bar_dst_address(&dma_sg_pcie_scheduler, XPAR_PCIE_AXIBAR_5);

	//Set the Offset in the PCIe Bridge where the Address Translation Register of the AXI BAR that is Accessed by the S2MM Channel of the DMA is Located.
	//This is Required in Order to Configure the Address Translation Register of the AXI BAR
	//with the Physical Address of the Kernel Memory that the S2MM Channel of the DMA will Use as the Destination Address to Write the Processed Image Data.
    XDma_sg_pcie_scheduler_Set_axi_bar_dst_cfg_address(&dma_sg_pcie_scheduler, (XPAR_PCIE_BASEADDR + BAR5_OFFSET_L));


	return XST_SUCCESS;
}

/*
 * setup_acceleration_scheduler_sg()
 *
 * Setup Procedure of the Acceleration Scheduler Scatter/Gather.
 *
 * The Acceleration Scheduler Scatter/Gather Manages the whole Acceleration Procedure of the AGSG.
 *
 * @note For Details Check the HLS Code of the Acceleration Scheduler Scatter/Gather.
 */
int setup_acceleration_scheduler_sg()
{
	int status = 0;

	print("Set-Up Process for Acceleration Scheduler SG Block\r\n");

	//Setup the Configuration Structure of the Acceleration Scheduler Scatter/Gather.
	acceleration_scheduler_sg_config_ptr = XAcceleration_scheduler_sg_xdma_LookupConfig(XPAR_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG_XDMA_DEVICE_ID);

	if (acceleration_scheduler_sg_config_ptr == NULL)
	{
		xil_printf("Setting-up Acceleration Scheduler SG Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Setting-up Acceleration Scheduler SG Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the Acceleration Scheduler Scatter/Gather.
	status = XAcceleration_scheduler_sg_xdma_CfgInitialize(&acceleration_scheduler_sg, acceleration_scheduler_sg_config_ptr);

	if (status != XST_SUCCESS)
	{
		xil_printf("Initializing Acceleration Scheduler SG Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Initializing Acceleration Scheduler SG Instance: SUCCESS\r\n");
	}


	//Set the Base Address of the DMA-SG-PCIe-Scheduler that is Used by the AGSG.
	XAcceleration_scheduler_sg_xdma_Set_dma_sg_pcie_scheduler_base_address(&acceleration_scheduler_sg, XPAR_ACCEL_GROUP_SG_DMA_SG_PCIE_SCHEDULER_S_AXI_CFG_BASEADDR);

	//Set the Base Address of the APM that is Used by the AGSG.
	XAcceleration_scheduler_sg_xdma_Set_apm_device_address(&acceleration_scheduler_sg, XPAR_ACCEL_GROUP_SG_APM_BASEADDR);

	//Set the Address Offset in the Interrupt Manager where the MSI Request Registers are Located.
	XAcceleration_scheduler_sg_xdma_Set_interrupt_manager_register_offset(&acceleration_scheduler_sg, XPAR_INTERRUPT_MANAGER_S_AXI_CFG_BASEADDR + XINTERRUPT_MANAGER_CFG_ADDR_MSI_REQUEST_BASE + MSI_DATA_6_OFFSET);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_sg_xdma_Set_shared_apm_device_address(&acceleration_scheduler_sg, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_sg_xdma_Set_shared_metrics_address(&acceleration_scheduler_sg, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the Sobel Filter that is Used by the AGSG.
	XAcceleration_scheduler_sg_xdma_Set_sobel_device_address(&acceleration_scheduler_sg, XPAR_ACCEL_GROUP_SG_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);

	//Set the Number of the Acceleration Group that the Acceleration Scheduler Scatter/Gather Belongs to.
	XAcceleration_scheduler_sg_xdma_Set_accel_group(&acceleration_scheduler_sg, 7);


	return XST_SUCCESS;
}

/*
 * setup_acceleration_schedulers_direct()
 *
 * Setup Procedure of ALL the Acceleration Schedulers Direct.
 *
 * The Acceleration Schedulers Direct Manage the whole Acceleration Procedure of the AGDs.
 *
 * @note For Details Check the HLS Code of the Acceleration Scheduler Direct.
 */
int setup_acceleration_schedulers_direct()
{
	int status;

	print("Set-Up Process for Fetch Scheduler\r\n");

	//*************************************************************************************************//
	// Initialization for Acceleration Scheduler Direct Acceleration Group 0
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Acceleration Scheduler Direct of the AGD0.
	acceleration_scheduler_accel_group_direct_0_config_ptr = XAcceleration_scheduler_direct_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT_DEVICE_ID);

	if (acceleration_scheduler_accel_group_direct_0_config_ptr == NULL)
	{
		print("Setting-up Acceleration Scheduler Direct 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Acceleration Scheduler Direct 0 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the Acceleration Scheduler Direct of the AGD0.
	status = XAcceleration_scheduler_direct_CfgInitialize(&acceleration_scheduler_accel_group_direct_0, acceleration_scheduler_accel_group_direct_0_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Acceleration Scheduler Direct 0 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Acceleration Scheduler Direct 0 Instance: SUCCESS\r\n");
	}

	//Set the Base Address of the APM that is Used by the AGD0.
	XAcceleration_scheduler_direct_Set_apm_device_address(&acceleration_scheduler_accel_group_direct_0, XPAR_ACCEL_GROUP_DIRECT_0_APM_BASEADDR);

	//Set the Base Address of the DMA that is Used by the AGD0.
	XAcceleration_scheduler_direct_Set_dma_device_address(&acceleration_scheduler_accel_group_direct_0, XPAR_ACCEL_GROUP_DIRECT_0_DMA_BASEADDR);

	//Set the Address Offset in the Interrupt Manager where the MSI Request Registers are Located.
	XAcceleration_scheduler_direct_Set_interrupt_manager_register_offset(&acceleration_scheduler_accel_group_direct_0, XPAR_INTERRUPT_MANAGER_S_AXI_CFG_BASEADDR + XINTERRUPT_MANAGER_CFG_ADDR_MSI_REQUEST_BASE + MSI_DATA_0_OFFSET);


	/*
	 * Set the Source and Destination Addresses that the Acceleration Scheduler Direct of the AGD0 will Use to Start a DMA Transfer.
	 *
	 * @note The Functions Below are Commented because the Source and Destination Addresses are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_direct_Set_host_mem_dst_data_address(&acceleration_scheduler_accel_group_direct_0, );
	//XAcceleration_scheduler_direct_Set_host_mem_src_data_address(&acceleration_scheduler_accel_group_direct_0, );

	/*
	 * Set the Image Columns and Rows.
	 *
	 * @note This Functions are Commented because the Columns and Rows are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_direct_Set_image_cols(&acceleration_scheduler_accel_group_direct_0, );
	//XAcceleration_scheduler_direct_Set_image_rows(&acceleration_scheduler_accel_group_direct_0, );

	//Set the Number of the Acceleration Group that the Acceleration Scheduler Direct Belongs to.
	XAcceleration_scheduler_direct_Set_initiator_group(&acceleration_scheduler_accel_group_direct_0, 1);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_direct_Set_shared_apm_device_address(&acceleration_scheduler_accel_group_direct_0, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_direct_Set_shared_metrics_address(&acceleration_scheduler_accel_group_direct_0, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the Sobel Filter that is Used by the AGD0.
	XAcceleration_scheduler_direct_Set_sobel_device_address(&acceleration_scheduler_accel_group_direct_0, XPAR_ACCEL_GROUP_DIRECT_0_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);

	//*************************************************************************************************//
	// Initialization for Acceleration Scheduler Direct Acceleration Group 1
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Acceleration Scheduler Direct of the AGD1.
	acceleration_scheduler_accel_group_direct_1_config_ptr = XAcceleration_scheduler_direct_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT_DEVICE_ID);

	if (acceleration_scheduler_accel_group_direct_1_config_ptr == NULL)
	{
		print("Setting-up Acceleration Scheduler Direct 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Acceleration Scheduler Direct 1 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Acceleration Scheduler Direct of the AGD1.
	status = XAcceleration_scheduler_direct_CfgInitialize(&acceleration_scheduler_accel_group_direct_1, acceleration_scheduler_accel_group_direct_1_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Acceleration Scheduler Direct 1 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Acceleration Scheduler Direct 1 Instance: SUCCESS\r\n");
	}


	//Set the Base Address of the APM that is Used by the AGD1.
	XAcceleration_scheduler_direct_Set_apm_device_address(&acceleration_scheduler_accel_group_direct_1, XPAR_ACCEL_GROUP_DIRECT_1_APM_BASEADDR);

	//Set the Base Address of the DMA that is Used by the AGD1.
	XAcceleration_scheduler_direct_Set_dma_device_address(&acceleration_scheduler_accel_group_direct_1, XPAR_ACCEL_GROUP_DIRECT_1_DMA_BASEADDR);

	//Set the Address Offset in the Interrupt Manager where the MSI Request Registers are Located.
	XAcceleration_scheduler_direct_Set_interrupt_manager_register_offset(&acceleration_scheduler_accel_group_direct_1, XPAR_INTERRUPT_MANAGER_S_AXI_CFG_BASEADDR + XINTERRUPT_MANAGER_CFG_ADDR_MSI_REQUEST_BASE + MSI_DATA_1_OFFSET);


	/*
	 * Set the Source and Destination Addresses that the Acceleration Scheduler Direct of the AGD1 will Use to Start a DMA Transfer.
	 *
	 * @note The Functions Below are Commented because the Source and Destination Addresses are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_direct_Set_host_mem_dst_data_address(&acceleration_scheduler_accel_group_direct_1, );
	//XAcceleration_scheduler_direct_Set_host_mem_src_data_address(&acceleration_scheduler_accel_group_direct_1, );

	/*
	 * Set the Image Columns and Rows.
	 *
	 * @note This Functions are Commented because the Columns and Rows are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_direct_Set_image_cols(&acceleration_scheduler_accel_group_direct_1, );
	//XAcceleration_scheduler_direct_Set_image_rows(&acceleration_scheduler_accel_group_direct_1, );

	//Set the Number of the Acceleration Group that the Acceleration Scheduler Direct Belongs to.
	XAcceleration_scheduler_direct_Set_initiator_group(&acceleration_scheduler_accel_group_direct_1, 2);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_direct_Set_shared_apm_device_address(&acceleration_scheduler_accel_group_direct_1, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_direct_Set_shared_metrics_address(&acceleration_scheduler_accel_group_direct_1, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the Sobel Filter that is Used by the AGD1.
	XAcceleration_scheduler_direct_Set_sobel_device_address(&acceleration_scheduler_accel_group_direct_1, XPAR_ACCEL_GROUP_DIRECT_1_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);


	return(XST_SUCCESS);
}

/*
 * setup_acceleration_schedulers_indirect()
 *
 * Setup Procedure of ALL the Acceleration Schedulers Indirect.
 *
 * The Acceleration Schedulers Indirect Manage the whole Acceleration Procedure of the AGIs.
 *
 * @note For Details Check the HLS Code of the Acceleration Scheduler Indirect.
 */
int setup_acceleration_schedulers_indirect()
{
	int status;

	print("Set-Up Process for Acceleration Scheduler Indirect\r\n");

	//*************************************************************************************************//
	// Initialization for Acceleration Scheduler Indirect 0
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Acceleration Scheduler Indirect of the AGI0.
	acceleration_scheduler_accel_group_indirect_0_config_ptr = XAcceleration_scheduler_indirect_LookupConfig(XPAR_XACCELERATION_SCHEDULER_INDIRECT_0_DEVICE_ID);

	if (acceleration_scheduler_accel_group_indirect_0_config_ptr == NULL)
	{
		print("Setting-up Acceleration Scheduler Indirect 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Acceleration Scheduler Indirect 0 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Acceleration Scheduler Indirect of the AGI0.
	status = XAcceleration_scheduler_indirect_CfgInitialize(&acceleration_scheduler_accel_group_indirect_0, acceleration_scheduler_accel_group_indirect_0_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Acceleration Scheduler Indirect 0 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Acceleration Scheduler Indirect 0 Instance: SUCCESS\r\n");
	}



	//Set the Base Address of the DMA that is Used by the AGI0.
	XAcceleration_scheduler_indirect_Set_dma_base_address(&acceleration_scheduler_accel_group_indirect_0, XPAR_ACCEL_GROUP_INDIRECT_0_DMA_BASEADDR);

	//Set the Base Address of the Sobel Filter(Accelerator) that is Used by the AGI0.
	XAcceleration_scheduler_indirect_Set_sobel_base_address(&acceleration_scheduler_accel_group_indirect_0, XPAR_ACCEL_GROUP_INDIRECT_0_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);


	//Set the Base Address of the Scheduler Buffer that Belongs to the Fetch Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_f(&acceleration_scheduler_accel_group_indirect_0, XPAR_SCHEDULER_BUFFER_FETCH_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_f(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_0_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_f(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_0_DATA);

	/*
	 * Set the Source Address that the CDMA-Fetch will Use to Read the Data from the Kernel Memory for the AGI0.
	 * @note This Function is Commented because the Source Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_src_address_f(&acceleration_scheduler_accel_group_indirect_0, XPAR_AXI_HOST_BRAM_CONTROLLER_0_S_AXI_BASEADDR);

	//Set the Destination Address that the CDMA-Fetch will Use to Write the Data to the FPGA's DDR3 for the AGI0.
	XAcceleration_scheduler_indirect_Set_dst_address_f(&acceleration_scheduler_accel_group_indirect_0, XPAR_MIG_BASEADDR);


	//Set the Base Address of the Scheduler Buffer that Belongs to the Send Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_s(&acceleration_scheduler_accel_group_indirect_0, XPAR_SCHEDULER_BUFFER_SEND_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_s(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_0_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_s(&acceleration_scheduler_accel_group_indirect_0, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_0_DATA);

	//Set the Source Address that the CDMA-Send will Use to Read the Data from the FPGA's DDR3 for the AGI0.
	XAcceleration_scheduler_indirect_Set_src_address_s(&acceleration_scheduler_accel_group_indirect_0, (XPAR_MIG_BASEADDR + (4 * MBYTE)));

	/*
	 * Set the Destination Address that the CDMA-Send will Use to Write the Processed Data to the Kernel's Memory for the AGI0.
	 * @note This Function is Commented because the Destination Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_dst_address_s(&acceleration_scheduler_accel_group_indirect_0, (XPAR_AXI_FPGA_BRAM_CONTROLLER_0_S_AXI_BASEADDR + (4 * KBYTE)));

	/*
	 * Set the Image Columns and Rows.
	 * @note This Functions are Commented because the Columns and Rows are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_image_cols(&acceleration_scheduler_accel_group_indirect_0, 32);
	//XAcceleration_scheduler_indirect_Set_image_rows(&acceleration_scheduler_accel_group_indirect_0, 32);


	//Set the Number of the AGI that this Acceleration Scheduler Indirect Belongs to.
	XAcceleration_scheduler_indirect_Set_accel_group(&acceleration_scheduler_accel_group_indirect_0, 1);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_indirect_Set_shared_apm_base_address(&acceleration_scheduler_accel_group_indirect_0, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_indirect_Set_shared_metrics_base_address(&acceleration_scheduler_accel_group_indirect_0, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the APM that is Used by the AGI0.
	XAcceleration_scheduler_indirect_Set_apm_base_address(&acceleration_scheduler_accel_group_indirect_0, XPAR_ACCEL_GROUP_INDIRECT_0_APM_BASEADDR);


	//*************************************************************************************************//
	// Initialization for Acceleration Scheduler Indirect 1
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Acceleration Scheduler Indirect of the AGI1.
	acceleration_scheduler_accel_group_indirect_1_config_ptr = XAcceleration_scheduler_indirect_LookupConfig(XPAR_XACCELERATION_SCHEDULER_INDIRECT_1_DEVICE_ID);

	if (acceleration_scheduler_accel_group_indirect_1_config_ptr == NULL)
	{
		print("Setting-up Acceleration Scheduler Indirect 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Acceleration Scheduler Indirect 1 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Acceleration Scheduler Indirect of the AGI1.
	status = XAcceleration_scheduler_indirect_CfgInitialize(&acceleration_scheduler_accel_group_indirect_1, acceleration_scheduler_accel_group_indirect_1_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Acceleration Scheduler Indirect 1 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Acceleration Scheduler Indirect 1 Instance: SUCCESS\r\n");
	}


	//Set the Base Address of the DMA that is Used by the AGI1.
	XAcceleration_scheduler_indirect_Set_dma_base_address(&acceleration_scheduler_accel_group_indirect_1, XPAR_ACCEL_GROUP_INDIRECT_1_DMA_BASEADDR);

	//Set the Base Address of the Sobel Filter(Accelerator) that is Used by the AGI1.
	XAcceleration_scheduler_indirect_Set_sobel_base_address(&acceleration_scheduler_accel_group_indirect_1, XPAR_ACCEL_GROUP_INDIRECT_1_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);

	//Set the Base Address of the Scheduler Buffer that Belongs to the Fetch Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_f(&acceleration_scheduler_accel_group_indirect_1, XPAR_SCHEDULER_BUFFER_FETCH_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_1_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_1_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_f(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_1_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_f(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_1_DATA);

	/*
	 * Set the Source Address that the CDMA-Fetch will Use to Read the Data from the Kernel Memory for the AGI1.
	 * @note This Function is Commented because the Source Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_src_address_f(&acceleration_scheduler_accel_group_indirect_1, XPAR_AXI_HOST_BRAM_CONTROLLER_1_S_AXI_BASEADDR);

	//Set the Destination Address that the CDMA-Fetch will Use to Write the Data to the FPGA's DDR3 for the AGI1.
	XAcceleration_scheduler_indirect_Set_dst_address_f(&acceleration_scheduler_accel_group_indirect_1, XPAR_MIG_BASEADDR + (8 * MBYTE));


	//Set the Base Address of the Scheduler Buffer that Belongs to the Send Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_s(&acceleration_scheduler_accel_group_indirect_1, XPAR_SCHEDULER_BUFFER_SEND_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_1_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_1_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_s(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_1_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_s(&acceleration_scheduler_accel_group_indirect_1, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_1_DATA);


	//Set the Source Address that the CDMA-Send will Use to Read the Data from the FPGA's DDR3 for the AGI1.
	XAcceleration_scheduler_indirect_Set_src_address_s(&acceleration_scheduler_accel_group_indirect_1, XPAR_MIG_BASEADDR + (12 * MBYTE));

	/*
	 * Set the Destination Address that the CDMA-Send will Use to Write the Processed Data to the Kernel's Memory for the AGI1.
	 * @note This Function is Commented because the Destination Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_dst_address_s(&acceleration_scheduler_accel_group_indirect_1, XPAR_AXI_HOST_BRAM_CONTROLLER_1_S_AXI_BASEADDR + (4 * KBYTE));


	/*
	 * Set the Image Columns and Rows.
	 * @note This Functions are Commented because the Columns and Rows are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_image_cols(&acceleration_scheduler_accel_group_indirect_1, 32);
	//XAcceleration_scheduler_indirect_Set_image_rows(&acceleration_scheduler_accel_group_indirect_1, 32);


	//Set the Number of the AGI that this Acceleration Scheduler Indirect Belongs to.
	XAcceleration_scheduler_indirect_Set_accel_group(&acceleration_scheduler_accel_group_indirect_1, 2);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_indirect_Set_shared_apm_base_address(&acceleration_scheduler_accel_group_indirect_1, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_indirect_Set_shared_metrics_base_address(&acceleration_scheduler_accel_group_indirect_1, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the APM that is Used by the AGI1.
	XAcceleration_scheduler_indirect_Set_apm_base_address(&acceleration_scheduler_accel_group_indirect_1, XPAR_ACCEL_GROUP_INDIRECT_1_APM_BASEADDR);


	//*************************************************************************************************//
	// Initialization for Acceleration Scheduler Indirect 2
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Acceleration Scheduler Indirect of the AGI2.
	acceleration_scheduler_accel_group_indirect_2_config_ptr = XAcceleration_scheduler_indirect_LookupConfig(XPAR_XACCELERATION_SCHEDULER_INDIRECT_2_DEVICE_ID);

	if (acceleration_scheduler_accel_group_indirect_2_config_ptr == NULL)
	{
		print("Setting-up Acceleration Scheduler Indirect 2 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Acceleration Scheduler Indirect 2 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Acceleration Scheduler Indirect of the AGI2.
	status = XAcceleration_scheduler_indirect_CfgInitialize(&acceleration_scheduler_accel_group_indirect_2, acceleration_scheduler_accel_group_indirect_2_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Acceleration Scheduler Indirect 2 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Acceleration Scheduler Indirect 2 Instance: SUCCESS\r\n");
	}


	//Set the Base Address of the DMA that is Used by the AGI2.
	XAcceleration_scheduler_indirect_Set_dma_base_address(&acceleration_scheduler_accel_group_indirect_2, XPAR_ACCEL_GROUP_INDIRECT_2_DMA_BASEADDR);

	//Set the Base Address of the Sobel Filter(Accelerator) that is Used by the AGI2.
	XAcceleration_scheduler_indirect_Set_sobel_base_address(&acceleration_scheduler_accel_group_indirect_2, XPAR_ACCEL_GROUP_INDIRECT_2_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);

	//Set the Base Address of the Scheduler Buffer that Belongs to the Fetch Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_f(&acceleration_scheduler_accel_group_indirect_2, XPAR_SCHEDULER_BUFFER_FETCH_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_2_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_2_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_f(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_2_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_f(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_2_DATA);

	/*
	 * Set the Source Address that the CDMA-Fetch will Use to Read the Data from the Kernel Memory for the AGI2.
	 * @note This Function is Commented because the Source Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_src_address_f(&acceleration_scheduler_accel_group_indirect_2, XPAR_AXI_HOST_BRAM_CONTROLLER_1_S_AXI_BASEADDR);

	//Set the Destination Address that the CDMA-Fetch will Use to Write the Data to the FPGA's DDR3 for the AGI2.
	XAcceleration_scheduler_indirect_Set_dst_address_f(&acceleration_scheduler_accel_group_indirect_2, XPAR_MIG_BASEADDR + (16 * MBYTE));


	//Set the Base Address of the Scheduler Buffer that Belongs to the Send Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_s(&acceleration_scheduler_accel_group_indirect_2, XPAR_SCHEDULER_BUFFER_SEND_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_2_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_2_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_s(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_2_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_s(&acceleration_scheduler_accel_group_indirect_2, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_2_DATA);


	//Set the Source Address that the CDMA-Send will Use to Read the Data from the FPGA's DDR3 for the AGI2.
	XAcceleration_scheduler_indirect_Set_src_address_s(&acceleration_scheduler_accel_group_indirect_2, XPAR_MIG_BASEADDR + (20 * MBYTE));

	/*
	 * Set the Destination Address that the CDMA-Send will Use to Write the Processed Data to the Kernel's Memory for the AGI2.
	 * @note This Function is Commented because the Destination Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_dst_address_s(&acceleration_scheduler_accel_group_indirect_2, XPAR_AXI_HOST_BRAM_CONTROLLER_1_S_AXI_BASEADDR + (4 * KBYTE));


	/*
	 * Set the Image Columns and Rows.
	 * @note This Functions are Commented because the Columns and Rows are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_image_cols(&acceleration_scheduler_accel_group_indirect_2, 32);
	//XAcceleration_scheduler_indirect_Set_image_rows(&acceleration_scheduler_accel_group_indirect_2, 32);


	//Set the Number of the AGI that this Acceleration Scheduler Indirect Belongs to.
	XAcceleration_scheduler_indirect_Set_accel_group(&acceleration_scheduler_accel_group_indirect_2, 3);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_indirect_Set_shared_apm_base_address(&acceleration_scheduler_accel_group_indirect_2, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_indirect_Set_shared_metrics_base_address(&acceleration_scheduler_accel_group_indirect_2, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the APM that is Used by the AGI2.
	XAcceleration_scheduler_indirect_Set_apm_base_address(&acceleration_scheduler_accel_group_indirect_2, XPAR_ACCEL_GROUP_INDIRECT_2_APM_BASEADDR);


	//*************************************************************************************************//
	// Initialization for Acceleration Scheduler Indirect 3
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Acceleration Scheduler Indirect of the AGI3.
	acceleration_scheduler_accel_group_indirect_3_config_ptr = XAcceleration_scheduler_indirect_LookupConfig(XPAR_XACCELERATION_SCHEDULER_INDIRECT_3_DEVICE_ID);

	if (acceleration_scheduler_accel_group_indirect_3_config_ptr == NULL)
	{
		print("Setting-up Acceleration Scheduler Indirect 3 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Acceleration Scheduler Indirect 3 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Acceleration Scheduler Indirect of the AGI3.
	status = XAcceleration_scheduler_indirect_CfgInitialize(&acceleration_scheduler_accel_group_indirect_3, acceleration_scheduler_accel_group_indirect_3_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Acceleration Scheduler Indirect 3 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Acceleration Scheduler Indirect 3 Instance: SUCCESS\r\n");
	}


	//Set the Base Address of the DMA that is Used by the AGI3.
	XAcceleration_scheduler_indirect_Set_dma_base_address(&acceleration_scheduler_accel_group_indirect_3, XPAR_ACCEL_GROUP_INDIRECT_3_DMA_BASEADDR);

	//Set the Base Address of the Sobel Filter(Accelerator) that is Used by the AGI3.
	XAcceleration_scheduler_indirect_Set_sobel_base_address(&acceleration_scheduler_accel_group_indirect_3, XPAR_ACCEL_GROUP_INDIRECT_3_SOBEL_FILTER_S_AXI_S_AXI4_LITE_BASEADDR);

	//Set the Base Address of the Scheduler Buffer that Belongs to the Fetch Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_f(&acceleration_scheduler_accel_group_indirect_3, XPAR_SCHEDULER_BUFFER_FETCH_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_3_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_f(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_3_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_f(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_3_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Fetch Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_f(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_3_DATA);

	/*
	 * Set the Source Address that the CDMA-Fetch will Use to Read the Data from the Kernel Memory for the AGI3.
	 * @note This Function is Commented because the Source Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_src_address_f(&acceleration_scheduler_accel_group_indirect_3, XPAR_AXI_HOST_BRAM_CONTROLLER_1_S_AXI_BASEADDR);

	//Set the Destination Address that the CDMA-Fetch will Use to Write the Data to the FPGA's DDR3 for the AGI3.
	XAcceleration_scheduler_indirect_Set_dst_address_f(&acceleration_scheduler_accel_group_indirect_3, XPAR_MIG_BASEADDR + (24 * MBYTE));


	//Set the Base Address of the Scheduler Buffer that Belongs to the Send Scheduler.
	XAcceleration_scheduler_indirect_Set_scheduler_buffer_base_address_s(&acceleration_scheduler_accel_group_indirect_3, XPAR_SCHEDULER_BUFFER_SEND_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Source Address Register is Located.
	XAcceleration_scheduler_indirect_Set_src_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_3_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Destination Address Register is Located.
	XAcceleration_scheduler_indirect_Set_dst_address_reg_offset_s(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_3_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Data Size Register is Located.
	XAcceleration_scheduler_indirect_Set_data_size_reg_offset_s(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_3_DATA);

	//Set the Offset in the Scheduler Buffer that Belongs to the Send Scheduler where the First Address Offset Register is Located.
	XAcceleration_scheduler_indirect_Set_offset_reg_offset_s(&acceleration_scheduler_accel_group_indirect_3, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_3_DATA);


	//Set the Source Address that the CDMA-Send will Use to Read the Data from the FPGA's DDR3 for the AGI3.
	XAcceleration_scheduler_indirect_Set_src_address_s(&acceleration_scheduler_accel_group_indirect_3, XPAR_MIG_BASEADDR + (28 * MBYTE));

	/*
	 * Set the Destination Address that the CDMA-Send will Use to Write the Processed Data to the Kernel's Memory for the AGI3.
	 * @note This Function is Commented because the Destination Address is Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_dst_address_s(&acceleration_scheduler_accel_group_indirect_3, XPAR_AXI_HOST_BRAM_CONTROLLER_1_S_AXI_BASEADDR + (4 * KBYTE));


	/*
	 * Set the Image Columns and Rows.
	 * @note This Functions are Commented because the Columns and Rows are Now Set by the Kernel Driver.
	 */
	//XAcceleration_scheduler_indirect_Set_image_cols(&acceleration_scheduler_accel_group_indirect_3, 32);
	//XAcceleration_scheduler_indirect_Set_image_rows(&acceleration_scheduler_accel_group_indirect_3, 32);


	//Set the Number of the AGI that this Acceleration Scheduler Indirect Belongs to.
	XAcceleration_scheduler_indirect_Set_accel_group(&acceleration_scheduler_accel_group_indirect_3, 4);

	//Set the Base Address of the Shared Timer (Shared APM).
	XAcceleration_scheduler_indirect_Set_shared_apm_base_address(&acceleration_scheduler_accel_group_indirect_3, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA's BRAM that is Used as the Shared Metrics Memory.
	XAcceleration_scheduler_indirect_Set_shared_metrics_base_address(&acceleration_scheduler_accel_group_indirect_3, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the APM that is Used by the AGI3.
	XAcceleration_scheduler_indirect_Set_apm_base_address(&acceleration_scheduler_accel_group_indirect_3, XPAR_ACCEL_GROUP_INDIRECT_3_APM_BASEADDR);


	return(XST_SUCCESS);
}

/*
 * setup_fetch_scheduler()
 *
 * Setup Procedure of the Fetch Scheduler.
 *
 * The Fetch Scheduler is Used to Start CDMA Transfers from the Host's Kernel Memory to the FPGA's DDR3.
 *
 * It Checks its Scheduler Buffer for new Transfer Requests and Starts the CDMA-Fetch Engine
 * to Make a new Transfer According to the Transfer Info in the Scheduler Buffer.
 *
 * @note For Details Check the HLS Code of the Fetch Scheduler.
 */
int setup_fetch_scheduler()
{
	int status;

	print("Set-Up Process for Fetch Scheduler\r\n");

	//*************************************************************************************************//
	// Initialization for Fetch Scheduler for Fetch Data Operations
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Fetch Scheduler.
	fetch_scheduler_config_ptr = XFetch_scheduler_LookupConfig(XPAR_FETCH_SCHEDULER_DEVICE_ID);

	if (fetch_scheduler_config_ptr == NULL)
	{
		print("Setting-up Fetch Scheduler Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Fetch Scheduler Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Fetch Scheduler.
	status = XFetch_scheduler_CfgInitialize(&fetch_scheduler, fetch_scheduler_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Fetch Scheduler Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Fetch Scheduler Instance: SUCCESS\r\n");
	}

	//Set the Base Address of the CDMA-Fetch Engine.
	XFetch_scheduler_Set_cdma_base_address(&fetch_scheduler, XPAR_CDMA_FETCH_BASEADDR);

	//Set the Base Address of the Scheduler Buffer that Belongs to the CDMA-Fetch Engine.
	XFetch_scheduler_Set_scheduler_buffer_base_address(&fetch_scheduler, XPAR_SCHEDULER_BUFFER_FETCH_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer where the First Source Address Register is Located.
	XFetch_scheduler_Set_src_address_first_reg_offset(&fetch_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer where the First Destination Address Register is Located.
	XFetch_scheduler_Set_dst_address_first_reg_offset(&fetch_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer where the First Data Size Register is Located.
	XFetch_scheduler_Set_data_size_first_reg_offset(&fetch_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_0_DATA);

	//Set the Offset in the Scheduler Buffer where the First Address Offset Register is Located.
	XFetch_scheduler_Set_offset_first_reg_offset(&fetch_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_0_DATA);

	//Set the Step in Bytes that is Required to Locate the Next Set of Registers in the Scheduler Buffer.
	//@note For Details Check the HLS Code of the Fetch Scheduler.
	XFetch_scheduler_Set_step_offset(&fetch_scheduler, 0x20);

	//Set the Base Address of the Shared Timer (Shared APM).
	XFetch_scheduler_Set_shared_apm_base_address(&fetch_scheduler, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA BRAM which is Used as the Shared Metrics Memory.
	XFetch_scheduler_Set_shared_metrics_base_address(&fetch_scheduler, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the AXI BAR that the Fetch Scheduler will Use to Configure the Source Address Register of the CDMA-Fetch Engine.
	XFetch_scheduler_Set_axi_bar_base_address(&fetch_scheduler, XPAR_PCIE_AXIBAR_2);

	//Set the Offset in the PCIe Bridge where the Address Translation Register of the AXI BAR that is Accessed by the CDMA-Fetch is Located.
	//This is Required in Order to Configure the Address Translation Register of the AXI BAR
	//with the Physical Address of the Kernel Memory that the CDMA-Fetch will Use as the Source Address.
	XFetch_scheduler_Set_pcie_ctl_datr_address(&fetch_scheduler, XPAR_PCIE_BASEADDR + BAR2_OFFSET_L);

	//Enable the PCIe Mode in the Fetch Scheduler.
	//@note For Details Check the HLS Code of the Fetch Scheduler.
	XFetch_scheduler_Set_pcie_mode(&fetch_scheduler, 1);

	//Set the a Value that will be Used to Jump at the Correct Offset of the Shared Metrics Memory (FPGA's BRAM) where the Metrics Structures of the AGIs are Located.
	XFetch_scheduler_Set_accel_group_jump(&fetch_scheduler, 3);

	return(XST_SUCCESS);
}

/*
 * setup_send_scheduler()
 *
 * Setup Procedure of the Send Scheduler.
 *
 * The Send Scheduler is Used to Start CDMA Transfers from the FPGA's DDR3 to the Host's Kernel Memory.
 *
 * It Checks its Scheduler Buffer for new Transfer Requests and Starts the CDMA-Send Engine
 * to Make a new Transfer According to the Transfer Info in the Scheduler Buffer.
 *
 * @note For Details Check the HLS Code of the Send Scheduler.
 */
int setup_send_scheduler()
{
	int status;

	print("Set-Up Process for Send Scheduler\r\n");

	//*************************************************************************************************//
	// Initialization for Fetch Scheduler for Fetch Data Operations
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Send Scheduler.
	send_scheduler_config_ptr = XSend_scheduler_LookupConfig(XPAR_SEND_SCHEDULER_DEVICE_ID);

	if (send_scheduler_config_ptr == NULL)
	{
		print("Setting-up Send Scheduler Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Send Scheduler Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Send Scheduler.
	status = XSend_scheduler_CfgInitialize(&send_scheduler, send_scheduler_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Send Scheduler Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Send Scheduler Instance: SUCCESS\r\n");
	}

	//Set the Base Address of the CDMA-Send Engine.
	XSend_scheduler_Set_cdma_base_address(&send_scheduler, XPAR_CDMA_SEND_BASEADDR);

	//Set the Base Address of the Scheduler Buffer that Belongs to the CDMA-Send Engine.
	XSend_scheduler_Set_scheduler_buffer_base_address(&send_scheduler, XPAR_SCHEDULER_BUFFER_SEND_S_AXI_INT_CFG_BASEADDR);

	//Set the Offset in the Scheduler Buffer where the First Source Address Register is Located.
	XSend_scheduler_Set_src_address_first_reg_offset(&send_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_SRC_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer where the First Destination Address Register is Located.
	XSend_scheduler_Set_dst_address_first_reg_offset(&send_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_DST_ADDRESS_0_DATA);

	//Set the Offset in the Scheduler Buffer where the Data Size Address Register is Located.
	XSend_scheduler_Set_data_size_first_reg_offset(&send_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_DATA_SIZE_0_DATA);

	//Set the Offset in the Scheduler Buffer where the First Address Offset Register is Located.
	XSend_scheduler_Set_offset_first_reg_offset(&send_scheduler, XSCHEDULER_BUFFER_INT_CFG_ADDR_OFFSET_0_DATA);

	//Set the Step in Bytes that is Required to Locate the Next Set of Registers in the Scheduler Buffer.
	//@note For Details Check the HLS Code of the Send Scheduler.
	XSend_scheduler_Set_step_offset(&send_scheduler, 0x20);

	//Set the Base Address of the Shared Timer (Shared APM).
	XSend_scheduler_Set_shared_apm_base_address(&send_scheduler, XPAR_SHARED_APM_BASEADDR);

	//Set the Base Address of the FPGA BRAM which is Used as the Shared Metrics Memory.
	XSend_scheduler_Set_shared_metrics_base_address(&send_scheduler, XPAR_SHARED_METRICS_BRAM_CONTROLLER_S_AXI_BASEADDR);

	//Set the Base Address of the AXI BAR that the Send Scheduler will Use to Configure the Destination Address Register of the CDMA-Send Engine.
	XSend_scheduler_Set_axi_bar_base_address(&send_scheduler, XPAR_PCIE_AXIBAR_3);

	//Set the Offset in the PCIe Bridge where the Address Translation Register of the AXI BAR that is Accessed by the CDMA-Send is Located.
	//This is Required in Order to Configure the Address Translation Register of the AXI BAR
	//with the Physical Address of the Kernel Memory that the CDMA-Send will Use as the Destination Address.
	XSend_scheduler_Set_pcie_ctl_datr_address(&send_scheduler, XPAR_PCIE_BASEADDR + BAR3_OFFSET_L);

	//Enable the PCIe Mode in the Send Scheduler.
	//@note For Details Check the HLS Code of the Send Scheduler.
	XSend_scheduler_Set_pcie_mode(&send_scheduler, 1);

	//Set the Address Offset in the Interrupt Manager that the MSI Request Registers are Located.
	XSend_scheduler_Set_interrupt_manager_register_offset(&send_scheduler, XPAR_INTERRUPT_MANAGER_S_AXI_CFG_BASEADDR + XINTERRUPT_MANAGER_CFG_ADDR_MSI_REQUEST_BASE + MSI_DATA_2_OFFSET);

	//Set the a Value that will be Used to Jump at the Correct Offset of the Shared Metrics Memory (FPGA's BRAM) where the Metrics Structures of the AGIs are Located.
	XSend_scheduler_Set_accel_group_jump(&send_scheduler, 3);

	return(XST_SUCCESS);
}

/*
 * setup_scheduler_buffers()
 *
 * Setup Procedure of the Scheduler Buffers that Belong to the Fetch and Send Schedulers.
 *
 * The Scheduler Buffers are Used by the the Acceleration Schedulers Indirect to Store Transfer Information for new CDMA Transfer Requests.
 * The Fetch and Send Schedulers Read the Transfer Info from the Scheduler Buffers and Start CDMA Transfers Accordingly.
 *
 * @note For Details Check the HLS Code of the Scheduler Buffer.
 */
int setup_scheduler_buffers()
{
	int status;

	print("Set-Up Process for Scheduler Buffers\r\n");


	//*************************************************************************************************//
	// Initialization for Scheduler Buffer for Fetch Data Operations
	//*************************************************************************************************//

	//Setup the Configuration Structure of the Scheduler Buffer that Belongs to the Fetch Scheduler.
	scheduler_buffer_fetch_config_ptr = XScheduler_buffer_LookupConfig(XPAR_SCHEDULER_BUFFER_FETCH_DEVICE_ID);

	if (scheduler_buffer_fetch_config_ptr == NULL)
	{
		print("Setting-up Scheduler Buffer Fetch Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Scheduler Buffer Fetch Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Scheduler Buffer that Belongs to the Fetch Scheduler.
	status = XScheduler_buffer_CfgInitialize(&scheduler_buffer_fetch, scheduler_buffer_fetch_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Scheduler Buffer Fetch Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Scheduler Buffer Fetch Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for Scheduler Buffer for Send Data Operations
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Scheduler Buffer that Belongs to the Send Scheduler.
	scheduler_buffer_send_config_ptr = XScheduler_buffer_LookupConfig(XPAR_SCHEDULER_BUFFER_SEND_DEVICE_ID);

	if (scheduler_buffer_send_config_ptr == NULL)
	{
		print("Setting-up Scheduler Buffer Send Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Scheduler Buffer Send Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Scheduler Buffer that Belongs to the Send Scheduler.
	status = XScheduler_buffer_CfgInitialize(&scheduler_buffer_send, scheduler_buffer_send_config_ptr);

	if (status != XST_SUCCESS)
	{
		print("Initializing Scheduler Buffer Send Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Scheduler Buffer Send Instance: SUCCESS\r\n");
	}


	return(XST_SUCCESS);
}

/*
 * setup_cdmas()
 *
 * Setup Procedure of the CDMA Fetch and CDMA Send Engines.
 *
 * Both CDMAs are Used to Fetch or Send Image Data for the AGIs of the FPGA.
 *
 * The CDMA Fetch is Used to Transfer Image Data through the PCIe Bridge's AXI BARs from the Host's Kernel Memory to the FPGA's DDR3.
 * The CDMA Send is Used to Transfer Processed Image Data through the PCIe Bridge's AXI BARs from the FPGA's DDR3 to the Host's Kernel Memory.
 */
int setup_cdmas()
{
	int cdma_status;

	print("Set-Up Process for CDMAs\r\n");


	//Setup the Configuration Structure of the CDMA-Fetch Engine.
	cdma_fetch_config_ptr = XAxiCdma_LookupConfig(XPAR_CDMA_FETCH_DEVICE_ID);

	if (cdma_fetch_config_ptr == NULL)
	{
		print("Setting-up CDMA Fetch Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up CDMA Fetch Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the CDMA-Fetch Engine.
	cdma_status = XAxiCdma_CfgInitialize(&cdma_fetch, cdma_fetch_config_ptr,cdma_fetch_config_ptr->BaseAddress);

	if (cdma_status != XST_SUCCESS)
	{
		print("Initializing CDMA Fetch Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing CDMA Fetch Instance: SUCCESS\r\n");
	}


	//Setup the Configuration Structure of the CDMA-Send Engine.
	cdma_send_config_ptr = XAxiCdma_LookupConfig(XPAR_CDMA_SEND_DEVICE_ID);

	if (cdma_send_config_ptr == NULL)
	{
		print("Setting-up CDMA Send Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up CDMA Send Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the CDMA-Send Engine.
	cdma_status = XAxiCdma_CfgInitialize(&cdma_send, cdma_send_config_ptr,cdma_send_config_ptr->BaseAddress);

	if (cdma_status != XST_SUCCESS)
	{
		print("Initializing CDMA Send Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing CDMA Send Instance: SUCCESS\r\n");
	}

	return(XST_SUCCESS);

}

/*
 * setup_dmas()
 *
 * Setup Procedure of ALL the DMAs.
 *
 * The DMAs are Used to Make Data Transfers from a Memory to another without Occupying the Processor of the System.
 *
 * This Specific Type of DMA Executes the Following Sequence of Steps:
 *
 *  1 --> Reads Data from a Memory through an AXI Memory Mapped Master Interface (MM2S Channel).
 *  2 --> Forwards the Data through an AXI Stream Master Interface to an Accelerator.
 *  3 --> Receives the Processed Data through an AXI Stream Slave Interface.
 *  4 --> Write the Processed Data to a Memory through a Second AXI Memory Mapped Master Interface (S2MM Channel).
 *
 * Each Acceleration Group is Equipped with a DMA.
 *
 * --> The Acceleration Groups Direct Use the DMA to Read/Write Data through the PCIe Bridge's AXI BARS Directly from/to the Linux Kernel's Memory.
 * --> The Acceleration Groups Indirect Use the DMA to Read/Write Data from/to the FPGA's DDR3 Memory.
 * --> The Acceleration Groups Scatter/Gather Uses the DMA to Read/Write Data through the PCIe Bridge's AXI BARS Directly from/to the Linux Userspace's Memory.
 */
int setup_dmas()
{
	int dma_status;

	print("Set-Up Process for DMA Devices\r\n");


	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group 0 Direct
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGD0.
	dma_accel_group_direct_0_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_0_DMA_DEVICE_ID);

	if (dma_accel_group_direct_0_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Direct 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Direct 0 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGD0.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_direct_0, dma_accel_group_direct_0_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Direct 0 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Direct 0 Instance: SUCCESS\r\n");
	}

	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group 0 Direct
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGD1.
	dma_accel_group_direct_1_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_1_DMA_DEVICE_ID);

	if (dma_accel_group_direct_1_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Direct 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Direct 1 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGD1.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_direct_1, dma_accel_group_direct_1_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Direct 1 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Direct 1 Instance: SUCCESS\r\n");
	}

	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group 0 Indirect
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGI0.
	dma_accel_group_indirect_0_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_0_DMA_DEVICE_ID);

	if (dma_accel_group_indirect_0_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Indirect 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Indirect 0 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGI0.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_indirect_0, dma_accel_group_indirect_0_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Indirect 0 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Indirect 0 Instance: SUCCESS\r\n");
	}

	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group 1 Indirect
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGI1.
	dma_accel_group_indirect_1_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_1_DMA_DEVICE_ID);

	if (dma_accel_group_indirect_1_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Indirect 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Indirect 1 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGI1.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_indirect_1, dma_accel_group_indirect_1_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Indirect 1 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Indirect 1 Instance: SUCCESS\r\n");
	}

	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group 2 Indirect
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGI2.
	dma_accel_group_indirect_2_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_2_DMA_DEVICE_ID);

	if (dma_accel_group_indirect_2_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Indirect 2 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Indirect 2 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGI2.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_indirect_2, dma_accel_group_indirect_2_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Indirect 2 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Indirect 2 Instance: SUCCESS\r\n");
	}

	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group 3 Indirect
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGI3.
	dma_accel_group_indirect_3_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_3_DMA_DEVICE_ID);

	if (dma_accel_group_indirect_3_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Indirect 3 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Indirect 3 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGI3.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_indirect_3, dma_accel_group_indirect_3_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Indirect 3 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Indirect 3 Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for DMA Core of Acceleration Group Scatter/Gather
	//*************************************************************************************************//

	//Setup the Configuration Structure of the DMA of AGSG.
	dma_accel_group_sg_config_ptr = XAxiDma_LookupConfig(XPAR_ACCEL_GROUP_SG_DMA_DEVICE_ID);

	if (dma_accel_group_sg_config_ptr == NULL)
	{
		print("Setting-up DMA Acceleration Group Scatter/Gather Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up DMA Acceleration Group Scatter/Gather Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the DMA of the AGSG.
	dma_status = XAxiDma_CfgInitialize(&dma_accel_group_sg, dma_accel_group_sg_config_ptr);

	if (dma_status != XST_SUCCESS)
	{
		print("Initializing DMA Acceleration Group Scatter/Gather Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing DMA Acceleration Group Scatter/Gather Instance: SUCCESS\r\n");
	}


	return(XST_SUCCESS);
}

/*
 * setup_apms()
 *
 * Setup Procedure of ALL the AXI Performance Monitor Units (APMs).
 *
 * Each Acceleration Group is Equipped with one APM in order to Capture Events when the DMA of the Acceleration Group Makes Transfers.
 *
 * The APM Uses 3 Slots to "Sniff" Transactions on AXI Interfaces.
 *
 * Slot 0 Captures the AXI Memory Mapped Read Transactions and Read Bytes of the MM2S Channel of the DMA.
 * Slot 1 Captures the AXI Memory Mapped Write Transactions and Write Bytes of the S2MM Channel of the DMA.
 * Slot 2 Captures the AXI Stream Packets and Stream Bytes of the AXI Stream Output of the Sobel Filter.
 */
int setup_apms()
{
	int apm_status;

	print("Set-Up Process for AXI Performance Monitor Peripherals\r\n");

	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group 0 Direct
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGD0.
	apm_accel_group_direct_0_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_0_APM_DEVICE_ID);

	if (apm_accel_group_direct_0_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Direct 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Direct 0 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGD0.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_direct_0, apm_accel_group_direct_0_config_ptr,apm_accel_group_direct_0_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Direct 0 Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Direct 0 Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGD0 with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_direct_0, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_direct_0, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_direct_0, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_direct_0, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_direct_0, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_direct_0, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes


	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group 1 Direct
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGD1.
	apm_accel_group_direct_1_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_1_APM_DEVICE_ID);

	if (apm_accel_group_direct_1_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Direct 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Direct 1 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGD1.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_direct_1, apm_accel_group_direct_1_config_ptr,apm_accel_group_direct_1_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Direct 1 Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Direct 1 Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGD1 with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_direct_1, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_direct_1, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_direct_1, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_direct_1, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_direct_1, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_direct_1, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes

	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group 0 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGI0.
	apm_accel_group_indirect_0_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_0_APM_DEVICE_ID);

	if (apm_accel_group_indirect_0_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 0 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGI0.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_indirect_0, apm_accel_group_indirect_0_config_ptr,apm_accel_group_indirect_0_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 0 Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 0 Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGI0 with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_0, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_0, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_0, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_0, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_0, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_0, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes

	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group 1 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGI1.
	apm_accel_group_indirect_1_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_1_APM_DEVICE_ID);

	if (apm_accel_group_indirect_1_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 1 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGI1.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_indirect_1, apm_accel_group_indirect_1_config_ptr,apm_accel_group_indirect_1_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 1 Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 1 Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGI1 with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_1, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_1, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_1, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_1, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_1, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_1, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes


	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group 2 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGI2.
	apm_accel_group_indirect_2_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_2_APM_DEVICE_ID);

	if (apm_accel_group_indirect_2_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 2 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 2 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGI2.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_indirect_2, apm_accel_group_indirect_2_config_ptr,apm_accel_group_indirect_2_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 2 Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 2 Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGI2 with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_2, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_2, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_2, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_2, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_2, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_2, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes


	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group 3 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGI3.
	apm_accel_group_indirect_3_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_3_APM_DEVICE_ID);

	if (apm_accel_group_indirect_3_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 3 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Indirect 3 Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGI3.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_indirect_3, apm_accel_group_indirect_3_config_ptr,apm_accel_group_indirect_3_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 3 Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Indirect 3 Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGI3 with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_3, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_3, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_3, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_3, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_indirect_3, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_indirect_3, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes

	//*************************************************************************************************//
	// Initialization for AXI Performance Monitor Core of Acceleration Group for Scatter/Gather Transfers
	//*************************************************************************************************//


	//Setup the Configuration Structure of the APM of the AGSG.
	apm_accel_group_sg_config_ptr = XAxiPmon_LookupConfig(XPAR_ACCEL_GROUP_SG_APM_DEVICE_ID);

	if (apm_accel_group_sg_config_ptr == NULL)
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Scatter/Gather Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up AXI Performance Monitor Acceleration Group Scatter/Gather Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the APM of AGSG.
	apm_status = XAxiPmon_CfgInitialize(&apm_accel_group_sg, apm_accel_group_sg_config_ptr,apm_accel_group_sg_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing AXI Performance Monitor Acceleration Group Scatter/Gather Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing AXI Performance Monitor Acceleration Group Scatter/Gather Peripheral: SUCCESS\r\n");
	}

	/*
	 * Set the Metrics Counters 0 to 5 of the APM of the AGSG with the Type of Event to Capture.
	 */
	XAxiPmon_SetMetrics(&apm_accel_group_sg, SLOT0, XAPM_METRIC_SET_1, XAPM_METRIC_COUNTER_0); //Read Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_sg, SLOT0, XAPM_METRIC_SET_3, XAPM_METRIC_COUNTER_1); //Read Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_sg, SLOT1, XAPM_METRIC_SET_0, XAPM_METRIC_COUNTER_2); //Write Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_sg, SLOT1, XAPM_METRIC_SET_2, XAPM_METRIC_COUNTER_3); //Write Bytes

	XAxiPmon_SetMetrics(&apm_accel_group_sg, SLOT2, XAPM_METRIC_SET_17, XAPM_METRIC_COUNTER_4); //Stream Transactions
	XAxiPmon_SetMetrics(&apm_accel_group_sg, SLOT2, XAPM_METRIC_SET_18, XAPM_METRIC_COUNTER_5); //Stream Bytes


	return(XST_SUCCESS);
}

/*
 * setup_shared_apm()
 *
 * Setup Procedure of the Shared APM Peripheral.
 *
 * The Shared APM is Actually a AXI Performance Monitor Unit.
 * It is ONLY Used to Read its Global Clock Counter in Order to Get Time Metrics.
 *
 * It is Called Shared Since it is Commonly Used by the HW Schedulers of the FPGA as well as the Kernel Driver and the Userspace Application.
 * It is Considered as the Reference Clock for the whole System for Time Metrics.
 */
int setup_shared_apm()
{
	int apm_status;

	print("Set-Up Process for AXI Performance Monitor Peripheral\r\n");

	//Setup the Configuration Structure of the AXI Performance Monitor Unit (Shared APM).
	shared_apm_config_ptr = XAxiPmon_LookupConfig(XPAR_SHARED_APM_DEVICE_ID);

	if (shared_apm_config_ptr == NULL)
	{
		print("Setting-up Shared AXI Performance Monitor Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Shared AXI Performance Monitor Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the AXI Performance Monitor Unit (Shared APM).
	apm_status = XAxiPmon_CfgInitialize(&shared_apm, shared_apm_config_ptr,shared_apm_config_ptr->BaseAddress);


	if (apm_status != XST_SUCCESS)
	{
		print("Initializing Shared AXI Performance Monitor Peripheral: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Shared AXI Performance Monitor Peripheral: SUCCESS\r\n");
	}

	return(XST_SUCCESS);
}

/*
 * setup_gpio()
 *
 * Setup Procedure of the GPIO Peripherals.
 *
 * The Current FPGA Design Uses the 4 GPIO Peripherals Below:
 *
 * 1 --> GPIO-MSI
 * 2 --> GPIO-PCIe-Interrupt
 * 3 --> GPIO-MSI-Read
 * 4 --> GPIO-ACK
 *
 * 1) The GPIO-MSI is Connected with two Inputs of the PCIe Bridge and is Used to Trigger MSI Interrupts.
 *    When a AXI Master Peripheral Writes Values to the Data Registers of the GPIO-MSI it Forces the
 *    PCIe Bridge to Send MSI Interrupts that Target the Linux Kernel Driver on Behalf of the FPGA.
 *
 * 2) The GPIO-PCIe-Interrupt is Used as a Technique to Receive Interrupts on Behalf of the Host System.
 *    The Kernel Driver Writes Values to the Data Register of the GPIO-PCIe-Interrupt Peripheral.
 *    When the GPIO-PCIe-Interrupt Has a new Data Entry in its Data Registers it Triggers Interrupts that are Handled by the Microblaze.
 *
 * 3) In Some Cases the Linux Kernel Driver Does not Support Receiving Multiple MSI Interrupts.
 *    In such Cases the Value of the GPIO-MSI Output is, also, the GPIO-MSI-Read Input Value.
 *    When the Driver Receives a MSI Interrupt it Reads the Value of the GPIO-MSI-Read Peripheral.
 *    Depending on that Value the Kernel Driver will Call the Appropriate Handler Function.
 *
 * 4) The GPIO-ACK is Used by the Kernel Driver to Acknowledge that the Last Interrupt Has been Handled.
 *    In other Words, when the Interrupt Manager Sends a MSI Interrupt it waits Until the Driver Sends an Acknowledgment Signal through
 *    Writing a Value to the Data Register of the GPIO-ACK.
 *
 */
int setup_gpio()
{
	int gpio_status;

	print("Set-Up Process for GPIO Modules\r\n");

	//Initialize the GPIO-MSI Peripheral.
	gpio_status = XGpio_Initialize(&gpio_msi, XPAR_GPIO_MSI_DEVICE_ID);

	if (gpio_status != XST_SUCCESS)
	{
		print("Initializing GPIO MSI Module: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing GPIO MSI Module: SUCCESS\r\n");
	}

	//Initialize the GPIO-PCIe-Interrupt Peripheral.
	gpio_status = XGpio_Initialize(&gpio_pcie_interrupt, XPAR_GPIO_PCIE_INTERRUPT_DEVICE_ID);

	if (gpio_status != XST_SUCCESS)
	{
		print("Initializing GPIO PCIe Interrupt Module: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing GPIO PCIe Interrupt Module: SUCCESS\r\n");
	}

	//Set to Zero the Data Registers of the Channel 1 and Channel 2 of the GPIO-MSI Peripheral.
	XGpio_DiscreteWrite(&gpio_msi, 1, 0x00);
	XGpio_DiscreteWrite(&gpio_msi, 2, 0x00);

	//Set to Zero the Data Register of the Channel 1 of the GPIO-PCIe-Interrupt Peripheral.
	XGpio_DiscreteWrite(&gpio_pcie_interrupt, 1, 0x0);

	//Initialize the GPIO-MSI-Read Peripheral.
	gpio_status = XGpio_Initialize(&gpio_msi_read, XPAR_GPIO_MSI_READ_DEVICE_ID);

	if (gpio_status != XST_SUCCESS)
	{
		print("Initializing GPIO MSI Read Module: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing GPIO MSI Read Module: SUCCESS\r\n");
	}

	//Initialize the GPIO-ACK Peripheral.
	gpio_status = XGpio_Initialize(&gpio_ack, XPAR_GPIO_ACK_DEVICE_ID);

	if (gpio_status != XST_SUCCESS)
	{
		print("Initializing GPIO Module: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing GPIO Module: SUCCESS\r\n");
	}

	//Set to Zero the Data Register of the Channel 1 of the GPIO-ACK Peripheral.
	XGpio_DiscreteWrite(&gpio_ack, 1, 0x00);

	return(XST_SUCCESS);
}

/*
 * setup_pcie()
 *
 * Setup Procedure of the PCIe Bridge.
 *
 * The PCIe Bridge is a Translation Controller between the FPGA's AXI Interface and the Host's PCIe Bus.
 */
int setup_pcie()
{
	int pcie_status=0;

	print("Set-Up Process for PCIe Endpoint\r\n");


	//Setup the Configuration Structure of the PCIe Bridge.
	pcie_config_ptr = XAxiPcie_LookupConfig(XPAR_AXIPCIE_0_DEVICE_ID);
	if (pcie_config_ptr == NULL)
	{
		xil_printf("Setting-up PCIe Endpoint Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Setting-up PCIe Endpoint Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the PCIe Bridge.
	pcie_status = XAxiPcie_CfgInitialize(&pcie_ep, pcie_config_ptr,pcie_config_ptr->BaseAddress);

	if (pcie_status != XST_SUCCESS)
	{
		xil_printf("Initializing PCIe Endpoint Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Initializing PCIe Endpoint Instance: SUCCESS\r\n");
	}

	//Check the Status of the PCIe Link.
	pcie_status = XAxiPcie_IsLinkUp(&pcie_ep);

	if (pcie_status != TRUE )
	{
		xil_printf("Checking PCIe Link Status: DOWN\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Checking PCIe Link Status: UP\r\n");
	}

	return(XST_SUCCESS);
}

/*
 * setup_sobel_filters()
 *
 * Setup Procedure of ALL the Sobel Filter Peripherals.
 */
int setup_sobel_filters()
{
	int sobel_status;

	print("Set-Up Process for Sobel Filters\r\n");


	//*************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group 0 Direct
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGD0.
	sobel_filter_accel_group_direct_0_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_0_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_direct_0_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Direct 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Direct 0 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGD0.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_direct_0, sobel_filter_accel_group_direct_0_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Direct 0 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Direct 0 Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group 1 Direct
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGD1.
	sobel_filter_accel_group_direct_1_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_DIRECT_1_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_direct_1_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Direct 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Direct 1 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGD0.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_direct_1, sobel_filter_accel_group_direct_1_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Direct 1 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Direct 1 Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group 0 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGI0.
	sobel_filter_accel_group_indirect_0_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_0_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_indirect_0_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 0 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 0 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGI0.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_indirect_0, sobel_filter_accel_group_indirect_0_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 0 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 0 Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group 1 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGI1.
	sobel_filter_accel_group_indirect_1_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_1_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_indirect_1_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 1 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 1 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGI1.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_indirect_1, sobel_filter_accel_group_indirect_1_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 1 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 1 Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group 2 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGI2.
	sobel_filter_accel_group_indirect_2_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_2_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_indirect_2_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 2 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 2 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGI2.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_indirect_2, sobel_filter_accel_group_indirect_2_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 2 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 2 Instance: SUCCESS\r\n");
	}


	//*************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group 3 Indirect
	//*************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGI3.
	sobel_filter_accel_group_indirect_3_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_INDIRECT_3_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_indirect_3_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 3 Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Indirect 3 Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGI3.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_indirect_3, sobel_filter_accel_group_indirect_3_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 3 Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Indirect 3 Instance: SUCCESS\r\n");
	}


	//**************************************************************************************************//
	// Initialization for the Sobel Filter Core of Acceleration Group for Scatter/Gather Transfers
	//**************************************************************************************************//


	//Setup the Configuration Structure of the Sobel Filter of the AGSG.
	sobel_filter_accel_group_sg_config_ptr = XSobel_filter_LookupConfig(XPAR_ACCEL_GROUP_SG_SOBEL_FILTER_DEVICE_ID);

	if (sobel_filter_accel_group_sg_config_ptr == NULL)
	{
		print("Setting-up Sobel Filter Acceleration Group Scatter/Gather Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Setting-up Sobel Filter Acceleration Group Scatter/Gather Configuration Structure: SUCCESS\r\n");
	}


	//Initialize the Sobel Filter of the AGSG.
	sobel_status = XSobel_filter_CfgInitialize(&sobel_filter_accel_group_sg, sobel_filter_accel_group_sg_config_ptr);

	if (sobel_status != XST_SUCCESS)
	{
		print("Initializing Sobel Filter Acceleration Group Scatter/Gather Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		print("Initializing Sobel Filter Acceleration Group Scatter/Gathers Instance: SUCCESS\r\n");
	}

	XSobel_filter_Set_packet_mode_en(&sobel_filter_accel_group_sg, 1);
	XSobel_filter_Set_packet_size(&sobel_filter_accel_group_sg, PAGE_SIZE);



	return(XST_SUCCESS);
}

/*
 * setup_interrupt_manager()
 *
 * Setup Procedure of the Interrupt Manager Peripheral.
 *
 * The Interrupt Manager Receives by the Acceleration Schedulers Requests to Trigger MSI Interrupts over PCIe.
 * For every new Request it Writes the Vector Number of the MSI to the GPIO MSI Peripheral which Leads to Triggering an Interrupt over the PCIe Bridge.
 * The Interrupt Manager Waits then for an Acknowledgment Signal before Triggering the Next MSI Interrupt.
 *
 * @note For Details Check the HLS Code of the Interrupt Manager.
 */
int setup_interrupt_manager()
{
	int status = 0;

	print("Set-Up Process for Interrupt Manager Block\r\n");

	//Setup the Configuration Structure.
	interrupt_manager_config_ptr = XInterrupt_manager_LookupConfig(XPAR_INTERRUPT_MANAGER_DEVICE_ID);

	if (interrupt_manager_config_ptr == NULL)
	{
		xil_printf("Setting-up Interrupt Manager Configuration Structure: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Setting-up Interrupt Manager Configuration Structure: SUCCESS\r\n");
	}

	//Initialize the Interrupt Manager Peripheral.
	status = XInterrupt_manager_CfgInitialize(&interrupt_manager, interrupt_manager_config_ptr);

	if (status != XST_SUCCESS)
	{
		xil_printf("Initializing Interrupt Manager Instance: FAILURE\r\n");
		return XST_FAILURE;
	}
	else
	{
		xil_printf("Initializing Interrupt Manager Instance: SUCCESS\r\n");
	}

	//Set the Interrupt Manager with the Base Address of the GPIO-MSI Peripheral.
	XInterrupt_manager_Set_gpio_msi_device_address(&interrupt_manager, XPAR_GPIO_MSI_BASEADDR);

	//Set the Interrupt Manager with the Base Address of the GPIO-ACK Peripheral.
	XInterrupt_manager_Set_gpio_ack_device_address(&interrupt_manager, XPAR_GPIO_ACK_BASEADDR);

	//Set the Interrupt Manager with the Address Offset where its own Registers for MSI Requests are Located.
	//@note Check the HLS Code of the Interrupt Manager for Details.
	XInterrupt_manager_Set_self_msi_request_offset(&interrupt_manager, XPAR_INTERRUPT_MANAGER_S_AXI_CFG_BASEADDR + XINTERRUPT_MANAGER_CFG_ADDR_MSI_REQUEST_BASE);

	//Set the Interrupt Manager to Auto Restart after Completing its Execution.
	XInterrupt_manager_EnableAutoRestart(&interrupt_manager);

	//Start the Interrupt Manager Peripheral.
	XInterrupt_manager_Start(&interrupt_manager);

	return XST_SUCCESS;
}

/*
 * setup_interrupts()
 *
 * Setup Procedure of the Interrupt Controller and the Interrupts.
 */
int setup_interrupts()
{

	print("Set-Up Process for Interrupts\r\n");

	//Initialize the Interrupt Controller.
	XIntc_Initialize(&interrupt_controller, XPAR_AXI_INTERRUPT_CONTROLLER_DEVICE_ID);

	//Enable the Interrupt Controller.
	XIntc_Enable(&interrupt_controller, XPAR_AXI_INTERRUPT_CONTROLLER_DEVICE_ID);

	/*
	 * -------------------------------
	 * Register the Interrupt Handlers
	 * -------------------------------
	 */

	//Register the Interrupt Handler for the GPIO PCIe Interrupt Peripheral.
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_GPIO_PCIE_INTERRUPT_IP2INTC_IRPT_INTR, gpio_pcie_interrupt_handler,(void *)XPAR_GPIO_PCIE_INTERRUPT_BASEADDR);

	//Register the Interrupt Handler for the CDMA Fetch Engine.
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_CDMA_FETCH_CDMA_INTROUT_INTR, cdma_fetch_interrupt_handler,(void *)XPAR_CDMA_FETCH_BASEADDR);

	//Register the Interrupt Handler for the CDMA Send Engine.
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_CDMA_SEND_CDMA_INTROUT_INTR, cdma_send_interrupt_handler,(void *)XPAR_CDMA_SEND_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Direct 0 (AGD0).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_DIRECT_0_DMA_S2MM_INTROUT_INTR, dma_accel_group_direct_0_interrupt_handler,(void *)XPAR_ACCEL_GROUP_DIRECT_0_DMA_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Direct 1 (AGD1).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_DIRECT_1_DMA_S2MM_INTROUT_INTR, dma_accel_group_direct_1_interrupt_handler,(void *)XPAR_ACCEL_GROUP_DIRECT_1_DMA_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Indirect 0 (AGI0).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_0_DMA_S2MM_INTROUT_INTR, dma_accel_group_indirect_0_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_0_DMA_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Indirect 1 (AGI1).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_1_DMA_S2MM_INTROUT_INTR, dma_accel_group_indirect_1_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_1_DMA_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Indirect 2 (AGI2).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_2_DMA_S2MM_INTROUT_INTR, dma_accel_group_indirect_2_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_2_DMA_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Indirect 3 (AGI3).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_3_DMA_S2MM_INTROUT_INTR, dma_accel_group_indirect_3_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_3_DMA_BASEADDR);

	//Register the Interrupt Handler for the S2MM Channel of the DMA Engine of the Acceleration Group Scatter/Gather (AGSG).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_SG_DMA_S2MM_INTROUT_INTR, dma_accel_group_sg_interrupt_handler,(void *)XPAR_ACCEL_GROUP_SG_DMA_BASEADDR);

	//Register the Interrupt Handler for the DMA SG PCIe Scheduler.
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_SG_DMA_SG_PCIE_SCHEDULER_INTERRUPT_INTR, dma_sg_pcie_scheduler_interrupt_handler,(void *)XPAR_ACCEL_GROUP_SG_DMA_SG_PCIE_SCHEDULER_S_AXI_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Direct of the Acceleration Group Direct 0 (AGD0).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT_INTERRUPT_INTR, acceleration_scheduler_direct_group_0_interrupt_handler,(void *)XPAR_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT_S_AXI_MM2S_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Direct of the Acceleration Group Direct 1 (AGD1).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT_INTERRUPT_INTR, acceleration_scheduler_direct_group_1_interrupt_handler,(void *)XPAR_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT_S_AXI_MM2S_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Indirect of the Acceleration Group Indirect 0 (AGI0).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_INTR, acceleration_scheduler_indirect_group_0_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT_S_AXI_INT_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Indirect of the Acceleration Group Indirect 1 (AGI1).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_INTR, acceleration_scheduler_indirect_group_1_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT_S_AXI_INT_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Indirect of the Acceleration Group Indirect 2 (AGI2).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_INTR, acceleration_scheduler_indirect_group_2_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT_S_AXI_INT_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Indirect of the Acceleration Group Indirect 3 (AGI3).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_INTR, acceleration_scheduler_indirect_group_3_interrupt_handler,(void *)XPAR_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT_S_AXI_INT_CFG_BASEADDR);

	//Register the Interrupt Handler for the Acceleration Scheduler Scatter/Gather of the Acceleration Group Scatter/Gather (AGSG).
	XIntc_RegisterHandler(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, XPAR_AXI_INTERRUPT_CONTROLLER_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG_XDMA_INTERRUPT_INTR, acceleration_scheduler_sg_interrupt_handler,(void *)XPAR_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG_XDMA_S_AXI_MM2S_CFG_BASEADDR);


	/*
	 * -----------------------------------------------------------------------------
	 * No Need to Enable the CDMA Fetch and CDMA Send Interrupts.
	 * They Get Automatically Enabled by the Fetch and Send Schedulers Respectively.
	 * -----------------------------------------------------------------------------
	 */

//	XAxiCdma_IntrEnable(&cdma_fetch, XAXICDMA_XR_IRQ_ERROR_MASK | XAXICDMA_XR_IRQ_IOC_MASK | XAXICDMA_XR_IRQ_DELAY_MASK);
//	XAxiCdma_IntrEnable(&cdma_send, XAXICDMA_XR_IRQ_ERROR_MASK | XAXICDMA_XR_IRQ_IOC_MASK | XAXICDMA_XR_IRQ_DELAY_MASK);


	/*
	 * ------------------------------------------------------
	 * Enable the DMA Interrupts.
	 * @note This Means that the DMAs Can Trigger Interrupts.
	 * ------------------------------------------------------
	 */

    //Enable the Interrupts of the DMA of the Acceleration Groud Direct 0 (AGD0).
	XAxiDma_IntrEnable(&dma_accel_group_direct_0, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	//Enable the Interrupts of the DMA of the Acceleration Groud Direct 1 (AGD1).
	XAxiDma_IntrEnable(&dma_accel_group_direct_1, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	//Enable the Interrupts of the DMA of the Acceleration Groud Indirect 0 (AGI0).
	XAxiDma_IntrEnable(&dma_accel_group_indirect_0, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	//Enable the Interrupts of the DMA of the Acceleration Groud Indirect 1 (AGI1).
	XAxiDma_IntrEnable(&dma_accel_group_indirect_1, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	//Enable the Interrupts of the DMA of the Acceleration Groud Indirect 2 (AGI2).
	XAxiDma_IntrEnable(&dma_accel_group_indirect_2, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	//Enable the Interrupts of the DMA of the Acceleration Groud Indirect 3 (AGI3).
	XAxiDma_IntrEnable(&dma_accel_group_indirect_3, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	//Enable the Interrupts of the DMA of the Acceleration Groud Scatter/Gather (AGSG).
	XAxiDma_IntrEnable(&dma_accel_group_sg, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);


	/*
	 * ------------------------------------------------------------
	 * Enable the Scheduler Interrupts.
	 * @note This Means that the Schedulers Can Trigger Interrupts.
	 * ------------------------------------------------------------
	 */

	//Enable the Interrupts of the DMA SG PCIe Scheduler.
	XDma_sg_pcie_scheduler_InterruptEnable(&dma_sg_pcie_scheduler, 0xFFFFFFFF);
	XDma_sg_pcie_scheduler_InterruptGlobalEnable(&dma_sg_pcie_scheduler);

	//Enable the Interrupts of the Acceleration Scheduler Direct of the Acceleration Group Direct 0 (AGD0).
	XAcceleration_scheduler_direct_InterruptEnable(&acceleration_scheduler_accel_group_direct_0, 0xFFFFFFFF);
	XAcceleration_scheduler_direct_InterruptGlobalEnable(&acceleration_scheduler_accel_group_direct_0);

	//Enable the Interrupts of the Acceleration Scheduler Direct of the Acceleration Group Direct 1 (AGD1).
	XAcceleration_scheduler_direct_InterruptEnable(&acceleration_scheduler_accel_group_direct_1, 0xFFFFFFFF);
	XAcceleration_scheduler_direct_InterruptGlobalEnable(&acceleration_scheduler_accel_group_direct_1);

	//Enable the Interrupts of the Acceleration Scheduler Indirect of the Acceleration Group Indirect 0 (AGI0).
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_0, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_0);

	//Enable the Interrupts of the Acceleration Scheduler Indirect of the Acceleration Group Indirect 1 (AGI1).
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_1, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_1);

	//Enable the Interrupts of the Acceleration Scheduler Indirect of the Acceleration Group Indirect 2 (AGI2).
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_2, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_2);

	//Enable the Interrupts of the Acceleration Scheduler Indirect of the Acceleration Group Indirect 3 (AGI3).
	XAcceleration_scheduler_indirect_InterruptEnable(&acceleration_scheduler_accel_group_indirect_3, 0xFFFFFFFF);
	XAcceleration_scheduler_indirect_InterruptGlobalEnable(&acceleration_scheduler_accel_group_indirect_3);

	//Enable the Interrupts of the Acceleration Scheduler Indirect of the Acceleration Group Scatter/Gather (AGSG).
	XAcceleration_scheduler_sg_xdma_InterruptEnable(&acceleration_scheduler_sg, 0xFFFFFFFF);
	XAcceleration_scheduler_sg_xdma_InterruptGlobalEnable(&acceleration_scheduler_sg);



	/*
	 * ------------------------------------------------------
	 * Enable the GPIO Interrupts.
	 * @note This Means that the GPIO Can Trigger Interrupts.
	 * ------------------------------------------------------
	 */

	XGpio_WriteReg(XPAR_GPIO_PCIE_INTERRUPT_BASEADDR, XGPIO_IER_OFFSET, XGPIO_IR_CH2_MASK) ;
	XGpio_WriteReg(XPAR_GPIO_PCIE_INTERRUPT_BASEADDR, XGPIO_GIE_OFFSET, XGPIO_GIE_GINTR_ENABLE_MASK);
	XGpio_InterruptEnable(&gpio_pcie_interrupt, XPAR_GPIO_PCIE_INTERRUPT_IP2INTC_IRPT_MASK);
	XGpio_InterruptGlobalEnable(&gpio_pcie_interrupt);

	/*
	 * -------------------------------------------------------------
	 * Set the Interrupt Mask.
	 * Uncomment the Masks of the Interrupts that we Need to Enable.
	 * -------------------------------------------------------------
	 */

	interrupt_mask=
//					XPAR_CDMA_FETCH_CDMA_INTROUT_MASK +
//					XPAR_CDMA_SEND_CDMA_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_DIRECT_0_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_DIRECT_1_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_0_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_1_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_2_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_INDIRECT_3_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_SG_DMA_S2MM_INTROUT_MASK +
//					XPAR_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG_XDMA_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_SG_DMA_SG_S2MM_PCIE_SCHEDULER_INTERRUPT_MASK +
//					XPAR_ACCEL_GROUP_SG_DMA_SG_MM2S_PCIE_SCHEDULER_INTERRUPT_MASK +
					XPAR_GPIO_PCIE_INTERRUPT_IP2INTC_IRPT_MASK;


	/*
	 * Enable the Interrupts Inside the Interrupt Controller.
	 * @note this Means that the Interrupt Controller Can Receive the Interrupts According to the Enabled Interrupt Masks.
	 */
	XIntc_EnableIntr(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR, interrupt_mask);

	//Enable the Interrupt COntroller.
	XIntc_MasterEnable(XPAR_AXI_INTERRUPT_CONTROLLER_BASEADDR);

	/*
	 * Enable Interrupts.
	 * @note this Means that the Microblaze Can Receive Interrupts.
	 */
	microblaze_enable_interrupts();

	//Start the Interrupt Controller.
	XIntc_Start(&interrupt_controller, XIN_REAL_MODE);

	return (XST_SUCCESS);
}
