/*******************************************************************************
* Filename:   dma_sg_pcie_scheduler.cpp
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
#include "dma_sg_pcie_scheduler.h"


/*
 * ------------------------------
 * Registers and Masks of the DMA
 * ------------------------------
 */

/*
 * Tx Channel Registers Base Offset.
 */
#define XAXIDMA_TX_OFFSET 0x00000000

/*
 * Rx Channel Registers Base Offset.
 */
#define XAXIDMA_RX_OFFSET 0x00000030


/*
 * This Set of Registers are Applicable for both Channels of the DMA.
 * Add XAXIDMA_TX_OFFSET to Get to TX channel, and XAXIDMA_RX_OFFSET to Get to RX Channel.
 */
#define XAXIDMA_CR_OFFSET       0x00000000 // Control Register.
#define XAXIDMA_SR_OFFSET       0x00000004 // Status Register.
#define XAXIDMA_SRCADDR_OFFSET  0x00000018 // Source Address Register.
#define XAXIDMA_DESTADDR_OFFSET 0x00000018 // Destination Address Register.
#define XAXIDMA_BUFFLEN_OFFSET  0x00000028 // Transfer Data Size Register.

#define XAXIDMA_CR_RUNSTOP_MASK	0x00000001 // Start/Stop DMA Channel Mask.
#define XAXIDMA_CR_RESET_MASK   0x00000004 // Reset DMA Mask.

#define XAXIDMA_IRQ_IOC_MASK    0x00001000 // Completion Interrupt Mask.
#define XAXIDMA_IRQ_DELAY_MASK  0x00002000 // Delay Interrupt Mask.
#define XAXIDMA_IRQ_ERROR_MASK  0x00004000 // Error Interrupt Mask.
#define XAXIDMA_IRQ_ALL_MASK    0x00007000 // All Interrupts Mask.

/*
 * serve_mm2s_transfer()
 *
 * Invoked from the dma_sg_pcie_scheduler() Top Function.
 *
 * The Usability of this Function is as Follows:
 *
 * a --> Get from the Scatter/Gather List of the Source Memory the Physical Address of the Current Page to Transfer.
 * b --> Set the Address Translation Register of the PCIe Bridge's Source AXI BAR with the Physical Address of the Current Page to Transfer.
 * c --> Setup and Start the DMA.
 *
 * The Function Parameters are:
 *
 * 01 --> The AXI Master Interface of the Core (cfg).
 * 02 --> The Base Address of the DMA.
 * 03 --> The Data Size of the MM2S Transfer.
 * 04 --> The Address of the Scatter/Gather List of the Source Memory.
 * 05 --> The Address of the BCIe Bridge's Source AXI BAR.
 * 06 --> The Offset in the PCIe Bridge of the Address Translation Register that Refers to the Source AXI BAR.
 * 07 --> The Current Value of the Page Counter in order to Know which Physical Address to Extract from the Source Scatter/Gather List.
 * 08 --> The Transfer Size for the Current Page which Might be Less than the Page Size.
 */
int serve_mm2s_transfer(/*01*/volatile ap_uint<32> *cfg,
                        /*02*/unsigned int dma_device_address,
                        /*03*/unsigned int src_data_size,
                        /*04*/unsigned int sgl_address,
                        /*05*/unsigned int axi_bar_src_address,
                        /*06*/unsigned int axi_bar_src_cfg_address,
                        /*07*/int page_counter,
                        /*08*/ap_uint<32>current_transfer_size
						)
{
	ap_uint<32> data_register_array[2]; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.
	ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.


	/*
	 * ---------------------------------------------------------------------------------------------------------------------
	 * Get the Physical Address of the Current Page of the Scatter/Gather List and Set the Source AXI BAR of the PCIe Bridge
	 * ---------------------------------------------------------------------------------------------------------------------
	 */

	//Get the 64 Bit Physical Address of the Current Page from the Source Scatter/Gather List.
	//The data_register_array[0] Holds the 32 LSBs of the Physical Address.
	//The data_register_array[1] Holds the 32 MSBs of the Physical Address.
	memcpy(data_register_array, (const ap_uint<32> *)(cfg + ((sgl_address + (page_counter * sizeof(ap_uint<64>))) / 4)), sizeof(ap_uint<64>));

	data_register = data_register_array[0];
	//Write the 32 LSBs of the Physical Address of the Current Page to the Lower Register of the Source AXI BAR.
	memcpy((ap_uint<32> *)(cfg + (axi_bar_src_cfg_address) / 4), &data_register, sizeof(ap_uint<32>));

	data_register = data_register_array[1];
	//Write the 32 MSBs of the Physical Address of the Current Page to the Upper Register of the Source AXI BAR.
	memcpy((ap_uint<32> *)(cfg + (axi_bar_src_cfg_address - 4) / 4), &data_register, sizeof(ap_uint<32>));



	/*
	 * ---------------------------------------------
	 * Setup and Start DMA to Device Transfer (MM2S)
	 * ---------------------------------------------
	 */

	//Get from the Internal Register (axi_bar_src_address) of the Core the Source Address that the DMA will Use to Read the Initial Image Data.
	//The Source Address of the DMA MM2S Channel will be the Source AXI BAR which Corresponds to the Physical Address of the Current Page.
	data_register = axi_bar_src_address;

	//Write the Source Address to the Source Register of the DMA.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_SRCADDR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	//Read the MM2S Control Register of the DMA.
	memcpy(&data_register, (const ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

	//Set the Recently Read Value with the Mask Required to Enable the MM2S DMA Channel.
	data_register = data_register | XAXIDMA_CR_RUNSTOP_MASK;

	//Write the new Value Back to the Control Register of the DMA in Order to Enable the MM2S Channel.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	//Write the Transfer Size to the MM2S Length Register of the DMA which Starts the MM2S Transfer.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_BUFFLEN_OFFSET) / 4), &current_transfer_size, sizeof(ap_uint<32>));


	return 1;

}

/*
 * serve_s2mm_transfer()
 *
 * Invoked from the dma_sg_pcie_scheduler() Top Function.
 *
 * The Usability of this Function is as Follows:
 *
 * a --> Get from the Scatter/Gather List of the Destination Memory the Physical Address of the Current Page to Transfer.
 * b --> Set the Address Translation Register of the PCIe Bridge's Destination AXI BAR with the Physical Address of the Current Page to Transfer.
 * c --> Setup and Start the DMA.
 *
 * The Function Parameters are:
 *
 * 01 --> The AXI Master Interface of the Core (cfg).
 * 02 --> The Base Address of the DMA.
 * 03 --> The Data Size of the S2MM Transfer.
 * 04 --> The Address of the Scatter/Gather List of the Destination Memory.
 * 05 --> The Address of the BCIe Bridge's Destination AXI BAR.
 * 06 --> The Offset in the PCIe Bridge of the Address Translation Register that Refers to the Source AXI BAR.
 * 07 --> The Current Value of the Page Counter in order to Know which Physical Address to Extract from the Source Scatter/Gather List.
 * 08 --> The Transfer Size for the Current Page which Might be Less than the Page Size.
 */
int serve_s2mm_transfer(/*01*/volatile ap_uint<32> *cfg,
                        /*02*/unsigned int dma_device_address,
                        /*03*/unsigned int src_data_size,
                        /*04*/unsigned int sgl_address,
                        /*05*/unsigned int axi_bar_dst_address,
                        /*06*/unsigned int axi_bar_dst_cfg_address,
                        /*07*/int page_counter,
                        /*08*/ap_uint<32>current_transfer_size)
{
	ap_uint<32> data_register_array[2]; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.
	ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.

	/*
	 * ---------------------------------------------------------------------------------------------------------------------
	 * Get the Physical Address of the Current Page of the Scatter/Gather List and Set the Destination AXI BAR of the PCIe Bridge
	 * ---------------------------------------------------------------------------------------------------------------------
	 */

	//Get the 64 Bit Physical Address of the Current Page from the Destination Scatter/Gather List.
	//The data_register_array[0] Holds the 32 LSBs of the Physical Address.
	//The data_register_array[1] Holds the 32 MSBs of the Physical Address.
	memcpy(data_register_array, (const ap_uint<32> *)(cfg + ((sgl_address + (page_counter * sizeof(ap_uint<64>))) / 4)), sizeof(ap_uint<64>));

	data_register = data_register_array[0];
	//Write the 32 LSBs of the Physical Address of the Current Page to the Lower Register of the Destination AXI BAR.
	memcpy((ap_uint<32> *)(cfg + (axi_bar_dst_cfg_address) / 4), &data_register, sizeof(ap_uint<32>));

	data_register = data_register_array[1];
	//Write the 32 MSBs of the Physical Address of the Current Page to the Upper Register of the Destination AXI BAR.
	memcpy((ap_uint<32> *)(cfg + (axi_bar_dst_cfg_address - 4) / 4), &data_register, sizeof(ap_uint<32>));



	/*
	 * ---------------------------------------------
	 * Setup and Start Device to DMA Transfer (S2MM)
	 * ---------------------------------------------
	 */

	//Get from the Internal Register (axi_bar_dst_address) of the Core the Destination Address that the DMA will Use to Read the Initial Image Data.
	//The Destination Address of the DMA S2MM Channel will be the Destination AXI BAR which Corresponds to the Physical Address of the Current Page.
	data_register = axi_bar_dst_address;

	//Write the Destination Address to the Source Register of the DMA.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_DESTADDR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	//Read the S2MM Control Register of the DMA.
	memcpy(&data_register, (const ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

	//Set the Recently Read Value with the Mask Required to Enable the S2MM DMA Channel.
	data_register = data_register | XAXIDMA_CR_RUNSTOP_MASK;

	//Write the new Value Back to the Control Register of the DMA in Order to Enable the S2MM Channel.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	//Write the Transfer Size to the S2MM Length Register of the DMA which Starts the S2MM Transfer.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_BUFFLEN_OFFSET) / 4), &current_transfer_size, sizeof(ap_uint<32>));

	return 1;

}

/*
 * serve_mm2s_interrupt()
 *
 * Invoked from the dma_sg_pcie_scheduler() Top Function.
 *
 * The Usability of this Function is to Acknowledge Triggered Interrupts on the MM2S Channel of the DMA.
 *
 * The Function Parameters are:
 *
 * 01 --> The AXI Master Interface of the Core (cfg).
 * 02 --> The Base Address of the DMA.
 */
int serve_mm2s_interrupt(volatile ap_uint<32> *cfg, unsigned int dma_device_address)
{
	ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.

	//Read the DMA MM2S Status Register of the DMA to Get the IRQs.
	memcpy(&data_register, (const ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_SR_OFFSET) / 4), sizeof(ap_uint<32>));

	//Filter the Recently Read Value with the XAXIDMA_IRQ_ALL_MASK so as to Keep ONLY the IRQs that were Triggered.
	data_register = data_register & XAXIDMA_IRQ_ALL_MASK;

	//Write the new Value Back to the MM2S Status Register of the DMA which Acknowledges the Triggered Interrupts on the MM2S Channel.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_SR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	return 1;

}

/*
 * serve_s2mm_interrupt()
 *
 * Invoked from the dma_sg_pcie_scheduler() Top Function.
 *
 * The Usability of this Function is to Acknowledge Triggered Interrupts on the S2MM Channel of the DMA.
 *
 * The Function Parameters are:
 *
 * 01 --> The AXI Master Interface of the Core (cfg).
 * 02 --> The Base Address of the DMA.
 */
int serve_s2mm_interrupt(volatile ap_uint<32> *cfg, unsigned int dma_device_address)
{
	ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.

	//Read the DMA S2MM Status Register of the DMA to Get the IRQs.
	memcpy(&data_register, (const ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET) / 4), sizeof(ap_uint<32>));

	//Filter the Recently Read Value with the XAXIDMA_IRQ_ALL_MASK so as to Keep ONLY the IRQs that were Triggered.
	data_register = data_register & XAXIDMA_IRQ_ALL_MASK;

	//Write the new Value Back to the S2MM Status Register of the DMA which Acknowledges the Triggered Interrupts on the S2MM Channel.
	memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	return 1;

}

/*
 * dma_sg_pcie_scheduler() Top Function
 *
 * The Hardware Funtionality of the DMA SG PCIe Scheduler Core.
 *
 * The DMA SG PCIe Scheduler Core is Part of the Acceleration Group Scatter/Gather.
 * It is Used to Manage the the MM2S and S2MM Channels of the DMA when a Scatter/Gather List is Required to Transfer the Image Data.
 * The DMA SG PCIe Scheduler Interacts with the DMA of the Acceleration Group Scatter/Gather and the Configuration AXI Interface of the PCIe Bridge.
 *
 * The Sequential Steps of the Core's Functionality are as Follows:
 *
 * a --> Calculate the Number of Pages to Transfer for the MM2S and S2MM Channels of the DMA.
 * b --> Enable the DMA MM2S Interrupts.
 * c --> Enable the DMA S2MM Interrupts.
 * d --> Start a Page Transfer over the MM2S Channel (See the serve_mm2s_transfer() Function for Details).
 * e --> Start a Page Transfer over the S2MM Channel (See the serve_s2mm_transfer() Function for Details).
 * f --> Loop for as long as Both Channels Require to Complete the Transfer of all the Pages (both_done).
 * g --> In Every Loop Check if Either the MM2S or the S2MM Channels Have Triggered an Interrupt on Completion of the Page Transfer.
 * h --> If any of the Channels Triggers an Interrupt then Clear the Channel's Interrupt
 *       (See the serve_mm2s_interrupt() and serve_s2MM_interrupt Functions for Details)
 *       and Start the Channel's next Page Transfer.
 *
 * The Function Parameters are the Input/Output Ports/Interfaces of the Core:
 *
 * 01 --------> The AXI Master Interface of the Core Used to Access External Devices and Memories.
 * 02 --------> Single Bit Input Used to Receive External Interrupts from the DMA MM2S Channel.
 * 03 --------> Single Bit Input Used to Receive External Interrupts from the DMA S2MM Channel.
 * 04 to 12 --> Registers of the Core that are Accessed through the AXI Slave Lite Interface of the Core.
 */
int dma_sg_pcie_scheduler(/*01*/volatile ap_uint<32> *cfg,
                          /*02*/volatile ap_uint<1> *mm2s_intr_in,
                          /*03*/volatile ap_uint<1> *s2mm_intr_in,
                          /*04*/unsigned int dma_device_address,
                          /*05*/unsigned int requested_data_size,
                          /*06*/unsigned int page_size,
                          /*07*/unsigned int mm2s_sgl_address,
                          /*08*/unsigned int axi_bar_src_address,
                          /*09*/unsigned int axi_bar_src_cfg_address,
                          /*10*/unsigned int s2mm_sgl_address,
                          /*11*/unsigned int axi_bar_dst_address,
                          /*12*/unsigned int axi_bar_dst_cfg_address
					 )
{

/*
 * The cfg is the AXI Master Interface of the Core.
 */
#pragma HLS INTERFACE m_axi port=cfg

/*
 * The mm2s_intr_in is a Single Bit Input which is Used to Receive External Interrupts from the DMA MM2S Channel.
 */
#pragma HLS INTERFACE ap_none port=mm2s_intr_in

/*
 * The s2mm_intr_in is a Single Bit Input which is Used to Receive External Interrupts from the DMA S2MM Channel.
 */
#pragma HLS INTERFACE ap_none port=s2mm_intr_in

/*
 * The dma_device_address is a Register to Store the Base Address of the DMA that this Core
 * will Need to Access through the cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=dma_device_address bundle=cfg

/*
 * The requested_data_size is a Register to Store the Size of the Data that will be Transferred.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=requested_data_size bundle=cfg

/*
 * The page_size is a Register to Store the Size of each Page(Usually 4K in Linux) that will be Transferred.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=page_size bundle=cfg

/*
 * The mm2s_sgl_address is a Register to Store the Address of the Scatter/Gather List of the Source Data.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=mm2s_sgl_address bundle=cfg

/*
 * The axi_bar_src_address is a Register to Store the Address of the AXI BAR that the DMA will Use to Read the Source Data.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=axi_bar_src_address bundle=cfg

/*
 * The axi_bar_src_cfg_address is a Register to Store the Address/Offset of the PCIe Bridge's Address Translation Register that Refers to the Source AXI BAR.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=axi_bar_src_cfg_address bundle=cfg

/*
 * The s2mm_sgl_address is a Register to Store the Address of the Scatter/Gather List of the Destination Data.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=s2mm_sgl_address bundle=cfg

/*
 * The axi_bar_dst_address is a Register to Store the Address of the AXI BAR that the DMA will Use to Write the Destination Data.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=axi_bar_dst_address bundle=cfg

/*
 * The axi_bar_src_cfg_address is a Register to Store the Address/Offset of the PCIe Bridge's Address Translation Register that Refers to the Source AXI BAR.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=axi_bar_dst_cfg_address bundle=cfg

#pragma HLS INTERFACE  s_axilite  port=return bundle=cfg



ap_uint<32> data_register_array[2]; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.
ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.

ap_uint<32> mm2s_data_size; // The Data Size to Transfer for a Page of the MM2S Channel (The Last Page may not be Full).
ap_uint<32> s2mm_data_size; // The Data Size to Transfer for a Page of the S2MM Channel (The Last Page may not be Full).

ap_uint<1> dma_mm2s_intr_in_value; // Used to Read the Last Value of the dma_mm2s_intr_in_value Input Port.
ap_uint<1> dma_s2mm_intr_in_value; // Used to Read the Last Value of the dma_s2mm_intr_in_value Input Port.

int mm2s_pages_counter = 0; // Used to Count the Number of Tranferred Pages for the MM2S Channel.
int s2mm_pages_counter = 0; // Used to Count the Number of Tranferred Pages for the S2MM Channel.

int mm2s_pages_number; // The Number of Pages to Transfer for the MM2S Channel.
int s2mm_pages_number; // The Number of Pages to Transfer for the S2MM Channel.
int mm2s_remaining_bytes; // Used to Count the Remaining Bytes of the MM2S Transfer.
int s2mm_remaining_bytes; // Used to Count the Remaining Bytes of the S2MM Transfer.

ap_uint<32> current_transfer_size;

int both_done = 0; // Flag to Know When Both Channels (MM2S/S2MM) are Done.



//Divide the Size of the Data to Transfer by the Page Size to Get the Number of Pages to Transfer over the MM2S and S2mm Channels.
mm2s_pages_number = requested_data_size / page_size;
s2mm_pages_number = requested_data_size / page_size;


/*
 * If the Value of the MM2S Pages Number Multiplied by the Page Size is Less than the Initial Data Size
 * then there is One More Page with Less Data than the Page Size.
 *
 * So, Increment mm2s_pages_number Variable by 1.
 */
if((mm2s_pages_number * page_size) < requested_data_size)
{
	mm2s_pages_number = mm2s_pages_number + 1;
}


//Initialize the Remaining Bytes for the MM2S Channel to be Equal to the Data Transfer Size.
mm2s_remaining_bytes = requested_data_size;


/*
 * If the Value of the S2MM Pages Number Multiplied by the Page Size is Less than the Initial Data Size
 * then there is One More Page with Less Data than the Page Size.
 *
 * So, Increment s2mm_pages_number Variable by 1.
 */
if((s2mm_pages_number * page_size) < requested_data_size)
{
	s2mm_pages_number = s2mm_pages_number + 1;
}


//Initialize the Remaining Bytes for the S2MM Channel to be Equal to the Data Transfer Size.
s2mm_remaining_bytes = requested_data_size;



/*
 * ----------------------------------------------
 * Enable the DMA MM2S Interrupts (DMA to Device)
 * ----------------------------------------------
 */

//Read the Control Register of the MM2S Channel of the DMA.
memcpy(&data_register, (const ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Masks Required to Enable the IOC, Delay and Error IRQs.
//NOTE that IOC Stands for Interrupt On Complete.
data_register = data_register | (XAXIDMA_IRQ_ERROR_MASK | XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);

//Write the new Value Back to the Control Register of the DMA to Enable the MM2S Interrupts.
memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));



/*
 * ----------------------------------------------
 * Enable the DMA S2MM Interrupts (Device to DMA)
 * ----------------------------------------------
 */

//Read the Control Register of the S2MM Channel of the DMA.
memcpy(&data_register, (const ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Masks Required to Enable the IOC, Delay and Error IRQs.
//NOTE that IOC Stands for Interrupt On Complete.
data_register = data_register | (XAXIDMA_IRQ_ERROR_MASK | XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);

//Write the new Value Back to the Control Register of the DMA to Enable the S2MM Interrupts.
memcpy((ap_uint<32> *)(cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));


/*
 * If the Value of the Remaining Bytes is Larger that a Page Size then we Can Set the DMA to Transfer a whole Page over the MM2S Channel.
 */
if(mm2s_remaining_bytes >= page_size)
{
	mm2s_data_size = page_size;
}

/*
 * If the Value of the Remaining Bytes is Less that a Page Size then we Can Set the DMA to Transfer the Remaining Bytes over the MM2S Channel.
 */
if((mm2s_remaining_bytes > 0) && (mm2s_remaining_bytes < page_size))
{
	mm2s_data_size = mm2s_remaining_bytes;
}

current_transfer_size = mm2s_data_size;

//Start a DMA Page Transfer over the MM2S Channel
//The Transfer Can be of Size Equal to a whole Page Size or Just the Remaining Bytes According to the current_transfer_size Variable.
serve_mm2s_transfer(cfg,
		 	 	 	dma_device_address,
					requested_data_size,
					mm2s_sgl_address,
					axi_bar_src_address,
					axi_bar_src_cfg_address,
					0,
					current_transfer_size);

//Decrement the MM2S Channel's Remaining Bytes According to the mm2s_data_size Variable in order to Know Hom Many Bytes are left to be Transferred.
mm2s_remaining_bytes = mm2s_remaining_bytes - mm2s_data_size;


/*
 * If the Value of the Remaining Bytes is Larger that a Page Size then we Can Set the DMA to Transfer a whole Page over the S2MM Channel.
 */
if(s2mm_remaining_bytes >= page_size)
{
	s2mm_data_size = page_size;
}

/*
 * If the Value of the Remaining Bytes is Less that a Page Size then we Can Set the DMA to Transfer the Remaining Bytes over the S2MM Channel.
 */
if((s2mm_remaining_bytes > 0) && (s2mm_remaining_bytes < page_size))
{
	s2mm_data_size = s2mm_remaining_bytes;
}

current_transfer_size = s2mm_data_size;

//Start a DMA Page Transfer over the S2MM Channel
//The Transfer Can be of Size Equal to a whole Page Size or Just the Remaining Bytes According to the current_transfer_size Variable.
serve_s2mm_transfer(cfg,
		 	 	 	dma_device_address,
					requested_data_size,
					s2mm_sgl_address,
					axi_bar_dst_address,
					axi_bar_dst_cfg_address,
					0,
					current_transfer_size);

//Decrement the S2MM Channel's Remaining Bytes According to the s2mm_data_size Variable in order to Know Hom Many Bytes are left to be Transferred.
s2mm_remaining_bytes = s2mm_remaining_bytes - s2mm_data_size;


//Start Looping for as Long as the Rest of the Pages for the MM2S and S2MM Channels are Being Transferred.
while(both_done < 2)
{
	//Read the Current State of the mm2s_intr_in Input.
	dma_mm2s_intr_in_value = *mm2s_intr_in;

	//Read the Current State of the s2mm_intr_in Input.
	dma_s2mm_intr_in_value = *s2mm_intr_in;

	/*
	 * If we Have an Interrupt from the MM2S Channel than we Should Clear the Interrupt and Start the Next Page Transfer.
	 */
	if(dma_mm2s_intr_in_value == 1)
	{
		//Acknowledge the Triggered Interrupt of the DMA MM2S Channel.
		serve_mm2s_interrupt(cfg, dma_device_address);

		//If the MM2S Pages Counter of the Current Page Has Not Reached the Total Number of Pages then Proceed to Start the Next Page Transfer.
		if(mm2s_pages_counter < (mm2s_pages_number - 1))
		{
			/*
			 * If the Value of the Remaining Bytes is Larger that a Page Size then we Can Set the DMA to Transfer a whole Page over the MM2S Channel.
			 */
			if(mm2s_remaining_bytes >= page_size)
			{
				mm2s_data_size = page_size;
			}

			/*
			 * If the Value of the Remaining Bytes is Less that a Page Size then we Can Set the DMA to Transfer the Remaining Bytes over the MM2S Channel.
			 */
			if((mm2s_remaining_bytes > 0) && (mm2s_remaining_bytes < page_size))
			{
				mm2s_data_size = mm2s_remaining_bytes;
			}

			current_transfer_size = mm2s_data_size;

			//Start a DMA Page Transfer over the MM2S Channel According to the current_transfer_size Variable.
			serve_mm2s_transfer(cfg,
								 dma_device_address,
								 requested_data_size,
								 mm2s_sgl_address,
								 axi_bar_src_address,
								 axi_bar_src_cfg_address,
								 mm2s_pages_counter + 1,
								 current_transfer_size);

			//Decrement the MM2S Channel's Remaining Bytes According to the mm2s_data_size Variable in order to Know Hom Many Bytes are left to be Transferred.
			mm2s_remaining_bytes = mm2s_remaining_bytes - mm2s_data_size;
		}

		//Increment the MM2S Pages Counter to Keep Track of the Remaining MM2S Pages to Transfer.
		mm2s_pages_counter++;

	}

	/*
	 * If we Have an Interrupt from the S2MM Channel than we Should Clear the Interrupt and Start the Next Page Transfer.
	 */
	if(dma_s2mm_intr_in_value == 1)
	{
		//Acknowledge the Triggered Interrupt of the DMA S2MM Channel.
		serve_s2mm_interrupt(cfg, dma_device_address);

		//If the S2MM Pages Counter of the Current Page Has Not Reached the Total Number of Pages then Proceed to Start the Next Page Transfer.
		if(s2mm_pages_counter < (s2mm_pages_number - 1))
		{
			/*
			 * If the Value of the Remaining Bytes is Larger that a Page Size then we Can Set the DMA to Transfer a whole Page over the S2MM Channel.
			 */
			if(s2mm_remaining_bytes >= page_size)
			{
				s2mm_data_size = page_size;
			}

			/*
			 * If the Value of the Remaining Bytes is Less that a Page Size then we Can Set the DMA to Transfer the Remaining Bytes over the S2MM Channel.
			 */
			if((s2mm_remaining_bytes > 0) && (s2mm_remaining_bytes < page_size))
			{
				s2mm_data_size = s2mm_remaining_bytes;
			}

			current_transfer_size = s2mm_data_size;

			//Start a DMA Page Transfer over the S2MM Channel According to the current_transfer_size Variable.
			serve_s2mm_transfer(cfg,
								 dma_device_address,
								 requested_data_size,
								 s2mm_sgl_address,
								 axi_bar_dst_address,
								 axi_bar_dst_cfg_address,
								 s2mm_pages_counter + 1,
								 current_transfer_size);

			//Decrement the S2MM Channel's Remaining Bytes According to the s2mm_data_size Variable in order to Know Hom Many Bytes are left to be Transferred.
			s2mm_remaining_bytes = s2mm_remaining_bytes - s2mm_data_size;

		}

		//Increment the S2MM Pages Counter to Keep Track of the Remaining S2MM Pages to Transfer.
		s2mm_pages_counter++;

	}

	//If the MM2S Pages Counter Has Reached the Total Number of Pages then the MM2S Channel Has Finished the Data Transfer.
	if(mm2s_pages_counter == (mm2s_pages_number))
	{
		//Increment the mm2s_pages_counter Variable so that will not Enter the Current if Condition Again.
		mm2s_pages_counter++;

		//Increment the both_done Variable on Behalf of the MM2S Channel.
		//The both_done Variable will ONLY be Incremented Once on Behalf of the MM2S Channel because we will not Enter this if Condition Again.
		//When the S2MM Channel, also, Increments the both_done Variable the Data Transfer is Completed (both_done =2).
		both_done++;
	}

	//If the S2MM Pages Counter Has Reached the Total Number of Pages then the S2MM Channel Has Finished the Data Transfer.
	if(s2mm_pages_counter == (s2mm_pages_number))
	{
		//Increment the s2mm_pages_counter Variable so that will not Enter the Current if Condition Again.
		s2mm_pages_counter++;

		//Increment the both_done Variable on Behalf of the S2MM Channel.
		//The both_done Variable will ONLY be Incremented Once on Behalf of the S2MM Channel because we will not Enter this if Condition Again.
		//When the MM2S Channel, also, Increments the both_done Variable the Data Transfer is Completed (both_done =2).
		both_done++;
	}

}

//Reset the Variables.
dma_mm2s_intr_in_value = 0;
dma_s2mm_intr_in_value = 0;
both_done = 0;

return 1;


}


