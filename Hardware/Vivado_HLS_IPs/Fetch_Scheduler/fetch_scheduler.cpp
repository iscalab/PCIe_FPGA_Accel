/*******************************************************************************
* Filename:   fetch_scheduler.cpp
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
#include "fetch_scheduler.h"


/*
 * ------------------------------
 * Registers and Masks of the DMA
 * ------------------------------
 */
#define XAXICDMA_CR_OFFSET    	0x00000000 // Control Register.
#define XAXICDMA_SR_OFFSET    	0x00000004 // Status Register.


#define XAXICDMA_SRCADDR_OFFSET 0x00000018 // Source Address Register.
#define XAXICDMA_DSTADDR_OFFSET 0x00000020 // Destination Address Register.
#define XAXICDMA_BTT_OFFSET     0x00000028 // Bytes to Transfer Register.



#define XAXICDMA_CR_RESET_MASK		0x00000004 // Reset CDMA Mask.

#define XAXICDMA_XR_IRQ_IOC_MASK		0x00001000 // Interrupt On Completion (IOC) Mask.
#define XAXICDMA_XR_IRQ_DELAY_MASK		0x00002000 // Delay Interrupt Mask.
#define XAXICDMA_XR_IRQ_ERROR_MASK		0x00004000 // Error Interrupt Mask.
#define XAXICDMA_XR_IRQ_ALL_MASK		0x00007000 // All Interrupt Mask.

/*
 * --------------------------------------------------------------------------
 * Registers and Masks of the AXI Performance Monitor Unit (APM/Shared Timer)
 * --------------------------------------------------------------------------
 */

#define XAPM_GCC_HIGH_OFFSET	0x0000 //Global Clock Counter 32 to 63 Bits (Upper).
#define XAPM_GCC_LOW_OFFSET		0x0004 //Global Clock Counter 0 to 31 Bits (Lower).


/*
 * fetch_scheduler()
 *
 * The Hardware Funtionality of the Fetch Scheduler Core.
 *
 * The Fetch Scheduler Core Does not Belong to Any Particular Acceleration Group but it is Used by ALL(4) the Acceleration Groups Indirect (AGIs).
 * The Responsibility of this Core is to Manage the Procedure of Fetching Data to the DDR3 Memory that will be Processed by the AGIs.
 * It Checks its Scheduler Buffer in Round Robin for new Transfer Requests by any of the AGIs.
 * If it Finds Information for new Transfer it Starts the CDMA Fetch Core to Transfer Image Data from the Host's Memory to the FPGA's DDR3.
 * The Corresponding AGIs will be then Signaled by the Fetch Scheduler to Process the Image Data.
 *
 * When an AGI wants to Request Image Data from the Fetch Scheduler it Has to Write the Source and Destination Addresses as well as the Transfer Size
 * and, if Required, an Address Offset to the Scheduler Buffer that Belongs to the Fetch Scheduler.
 *
 * The Scheduler Buffer Has 4 Sets of Registers with 4 Registers for each Set.
 * The 4 Registers are Used to Store the Source Address, the Destination Address, the Transfer Size and an Address Offset (If Required) Respectively.
 * Each Set Corresponds to One of the 4 AGIs.
 *
 * When an AGI Writes the Above Information to the Scheduler Buffer, the Fetch Scheduler Starts a CDMA Transfer Accordingly
 * to Fetch the Image Data in the FPGA's DDR3.
 *
 * The Sequential Steps of the Acceleration Procedure are as Follows:
 *
 * Start a for Loop of 4 Iterations where in each Iteration we Check for new CDMA Transfer Requests by each of the 4 AGIs Respectively.
 *
 * a --> Read the Data Size Register from the Current Set of Registers of the Scheduler Buffer.
 *       	If there is a Non-Zero Value then we Know that the Corresponding AGI Has Written the Required
 *       	Info (Source/Destination Address, Transfer Size, Addrress Offset) in Order to Request a Transfer by the CDMA Fetch.
 *       	If there is a Zero Value then we Check the Data Size Register of the Next Set for a Transfer Request by the Next AGI.
 * b --> Enable the Interrupts on the CDMA Fetch Core.
 * c --> Setup the CDMA with the Source and Destination Addresses.
 *       	If the Source Data Should be Fetched through the PCIe Bridge then Get the Source Address from the Scheduler Buffer and Set the
 *       	Address Translation Register of the Corresponding AXI BAR of the PCIe Bridge with this Address.
 *       	Then Set the Source Address Register of the CDMA Fetch Core to be the Corresponding AXI BAR.
 *       	If the Source Data Should not be Fetched through the PCIe Bridge then Just Set the Source Address Register of the CDMA Fetch Core
 *       	with the Source Address of the Scheduler Buffer.
 * d --> Read the Current Value of the Shared Timer to Get the Time that the CDMA Fetch Transfer Started.
 * e --> Setup the Bytes to Transfer Register with the Transfer Size which Triggers the CDMA Fetch Transfer.
 * f --> Wait for an Interrupt by the CDMA Fetch on Completion of the Transfer.
 * g --> Read the Current Value of the Shared Timer to Get the Time that the CDMA Fetch Transfer Ended.
 * h --> Acknowledge the CDMA Fetch Interrupt.
 * i --> Reset the CDMA Fetch Core.
 * j --> Re-Enable the Interrupts on the CDMA Fetch Core.
 * k --> Clear the Set of Registers of the Scheduler Buffer that Refer to the Current AGI.
 * l --> Send a Start Signal to the Acceleration Scheduler Indirect of the Corresponding AGI to Initiate the Acceleration Procedure.
 *
 * Repeat the Above Steps (a to l) for the Next Set of Registers of the Scheduler Buffer.
 *
 *
 * The Function Parameters are the Input/Output Ports/Interfaces of the Core:
 *
 * 01 --------> The AXI Master Interface of the Core Used to Access External Devices and Memories.
 * 02 --------> Single Bit Input Used to Receive External Interrupts from the CDMA Fetch Core.
 * 03 --------> Single Bit Output Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI0.
 * 04 --------> Single Bit Output Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI1.
 * 05 --------> Single Bit Output Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI2.
 * 06 --------> Single Bit Output Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI3.
 * 07 to 19 --> Registers of the Core that are Accessed through the AXI Slave Lite Interface of the Core.
 *
 * NOTE datr in pcie_ctl_datr_address Stands for Dynamic Address Translator Register.
 */
int fetch_scheduler(/*01*/volatile ap_uint<32> *ext_cfg,
                    /*02*/volatile ap_uint<1> *cdma_intr_in,
                    /*03*/volatile ap_uint<1> *start_0,
                    /*04*/volatile ap_uint<1> *start_1,
                    /*05*/volatile ap_uint<1> *start_2,
                    /*06*/volatile ap_uint<1> *start_3,
                    /*07*/unsigned int cdma_base_address,
                    /*08*/unsigned int scheduler_buffer_base_address,
                    /*09*/unsigned int src_address_first_reg_offset,
                    /*10*/unsigned int dst_address_first_reg_offset,
                    /*11*/unsigned int data_size_first_reg_offset,
                    /*12*/unsigned int offset_first_reg_offset,
                    /*13*/unsigned int step_offset,
                    /*14*/unsigned int shared_apm_base_address,
                    /*15*/unsigned int shared_metrics_base_address,
                    /*16*/unsigned int axi_bar_base_address,
                    /*17*/unsigned int pcie_ctl_datr_address,
                    /*17*/unsigned int pcie_mode,
                    /*19*/unsigned int accel_group_jump
					)
{

/*
 * The ext_cfg is the AXI Master Interface of the Core.
 */
#pragma HLS INTERFACE m_axi port=ext_cfg

/*
 * The cdma_intr_in is a Single Bit Input which is Used to Receive External Interrupts from the CDMA Fetch Core.
 */
#pragma HLS INTERFACE ap_none port=cdma_intr_in

/*
 * The start_0 is a Single Bit Output which is Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI0.
 */
#pragma HLS INTERFACE ap_ovld port=start_0

/*
 * The start_1 is a Single Bit Output which is Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI1.
 */
#pragma HLS INTERFACE ap_ovld port=start_1

/*
 * The start_2 is a Single Bit Output which is Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI2.
 */
#pragma HLS INTERFACE ap_ovld port=start_2

/*
 * The start_3 is a Single Bit Output which is Used to Send Start Signals to the Acceleration Scheduler Indirect of the AGI3.
 */
#pragma HLS INTERFACE ap_ovld port=start_3

/*
 * The cdma_base_address is a Register to Store the Base Address of the CDMA Fetch that this Core
 * will Need to Access through the ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=cdma_base_address bundle=int_cfg

/*
 * The scheduler_buffer_base_address is a Register to Store the Base Address of the Scheduler Buffer that this Core
 * will Need to Access through the ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=scheduler_buffer_base_address bundle=int_cfg

/*
 * The src_address_first_reg_offset is a Register to Store the Address Offset where the Source Address Register
 * of the First Set of Registers inside the Scheduler Buffer is Located.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=src_address_first_reg_offset bundle=int_cfg

/*
 * The dst_address_first_reg_offset is a Register to Store the Address Offset where the Destination Address Register
 * of the First Set of Registers inside the Scheduler Buffer is Located.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=dst_address_first_reg_offset bundle=int_cfg

/*
 * The data_size_first_reg_offset is a Register to Store the Address Offset where the Transfer Size Register
 * of the First Set of Registers inside the Scheduler Buffer is Located.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=data_size_first_reg_offset bundle=int_cfg

/*
 * The offset_first_reg_offset is a Register to Store the Address Offset where the Offset Register
 * of the First Set of Registers inside the Scheduler Buffer is Located.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=offset_first_reg_offset bundle=int_cfg

/*
 * The step_offset is a Register to Store the Number of Bytes to Jump inside the Scheduler Buffer
 * in order to Locate the Next Set of Registers.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=step_offset bundle=int_cfg

/*
 * The shared_apm_base_address is a Register to Store the Base Address of the Shared Timer (APM) that this Core
 * will Need to Access through the ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=shared_apm_base_address bundle=int_cfg

/*
 * The shared_metrics_base_address is a Register to Store the Base Address of the Memory that this Core
 * will Need to Access through the ext_cfg AXI Master Interface in Order to Write the Metrics Information.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=shared_metrics_base_address bundle=int_cfg

/*
 * The axi_bar_base_address is a Register to Store the Base Address of the Source AXI BAR of the PCIe Bridge that this Core
 * will Need to Access through the ext_cfg AXI Master Interface in Order to Read the Image Data over the PCIe Bus.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=axi_bar_base_address bundle=int_cfg

/*
 * The pcie_ctl_datr_address is a Register to Store the Address/Offset of the PCIe Bridge's Address Translation Register that Refers to the Source AXI BAR.
 * This Register is Accessed through the AXI Slave Lite Interface (s_axilite_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=pcie_ctl_datr_address bundle=int_cfg

/*
 * The pcie_mode is a Register to Store a Value (0 or 1) that Indicates whether we Access the Source Image Data through the PCIe Bridge or not.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=pcie_mode bundle=int_cfg

/*
 * The accel_group_jump is a Register to Store a Value that Helps to Access the Correct Metrics Structure in the Metrics Memory in order
 * to Store the Time Metrics that Refer to the Current AGI.
 * This Register is Accessed through the AXI Slave Lite Interface (int_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=accel_group_jump bundle=int_cfg

#pragma HLS INTERFACE  s_axilite  port=return bundle=int_cfg


int repeat;


ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.
ap_uint<32> irq; // Used to Temporalily Store the IRQ Mask.
ap_uint<32> source_address_register; // Used to Temporalily Store the Value of the Source Address Register of the Scheduler Buffer.
ap_uint<32> destination_address_register; // Used to Temporalily Store the Value of the Destination Address Register of the Scheduler Buffer.
ap_uint<32> data_size_register; // Used to Temporalily Store the Value of the Data Size Register of the Scheduler Buffer.
ap_uint<32> offset_register; // Used to Temporalily Store the Value of the Offset Register of the Scheduler Buffer.

ap_uint<32> address; // Used to Calculate an Address along with an Offset.


ap_uint<1> cdma_intr_in_value; // Used to Read the Last Value of the cdma_intr_in_value Input Port.

ap_uint<32> cdma_fetch_time_start_gcc_l; // Store the CDMA Fetch Transfer Start Time Lower Register from the Shared Timer (Shared APM).
ap_uint<32> cdma_fetch_time_start_gcc_u; // Store the CDMA Fetch Transfer Start Time Upper Register from the Shared Timer (Shared APM).

ap_uint<32> cdma_fetch_time_end_gcc_l; // Store the CDMA Fetch Transfer End Time Lower Register from the Shared Timer (Shared APM).
ap_uint<32> cdma_fetch_time_end_gcc_u; // Store the CDMA Fetch Transfer End Time Upper Register from the Shared Timer (Shared APM).

/*
 * Start an Infinite Loop.
 */
while(1)
{

	/*
	 * Make 4 Iterations and each Time Check the Current Set of Registers of the Scheduler Buffer for a New CDMA Fetch Transfer Request
	 * by the AGI that Refers to the Current Set of Registers.
	 */
	for(repeat = 0; repeat < 4; repeat++)
	{
		//Read the Data Size Register of the Current Set of Registers of the Scheduler Buffer.
		memcpy(&data_size_register, (const ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + data_size_first_reg_offset + (repeat * step_offset)) / 4), sizeof(ap_uint<32>));

		//If the Data Size Register is not Empty then we Have a New CDMA Fetch Transfer Request.
		//Else the Fetch Scheduler will Check the Data Size Register of the Next Set in the Next Iteration.
		if(data_size_register != 0)
		{


			/*
			 * --------------------------------------------
			 * Enable the Interrupts on the CDMA Fetch Core
			 * --------------------------------------------
			 */

			//Read the Control Register of the CDMA Fetch Core.
			memcpy(&data_register, (const ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

			//Set the Recently Read Value with the Masks Required to Enable the IOC, Delay and Error IRQs.
			//NOTE that IOC Stands for Interrupt On Complete.
			data_register = data_register | (XAXICDMA_XR_IRQ_ERROR_MASK | XAXICDMA_XR_IRQ_IOC_MASK | XAXICDMA_XR_IRQ_DELAY_MASK);

			//Write the new Value Back to the Control Register of the CDMA Fetch Core to Enable the Interrupts.
			memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

			/*
			 * -------------------------------------------------------------------------
			 * Setup the Source and Destination Address Registers of the CDMA Fetch Core
			 * -------------------------------------------------------------------------
			 */

			//If the PCIe Mode is Enabled then the Source Data Should be Read through the PCIe Bridge.
			//This Mode Requires to Set the Address Tranlation Register of the Source AXI BAR of the PCI Bridge.
			if(pcie_mode == 1)
			{
				//Read the Source Physical Address from the Source Address Register from the Current Set of the Scheduler Buffer.
				memcpy(&source_address_register, (const ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + src_address_first_reg_offset + (repeat * step_offset)) / 4), sizeof(ap_uint<32>));

				//Set the Address Tranlation Register of the Source AXI BAR of the PCI Bridge with the Source Physical Address.
				memcpy((ap_uint<32> *)(ext_cfg + (pcie_ctl_datr_address) / 4), &source_address_register, sizeof(ap_uint<32>));

				//Read the Address Offset from the Offset Register from the Current Set of the Scheduler Buffer.
				//NOTE it is Possible that this Register Has a Zero Value if there is no Offset Required to Access the Data.
				memcpy(&offset_register, (const ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + offset_first_reg_offset + (repeat * step_offset)) / 4), sizeof(ap_uint<32>));

				//Set the Source Address Register of the CDMA Fetch Core to be the Specified Source AXI BAR along with a Possible Offset.
				address = axi_bar_base_address + offset_register;
				memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_SRCADDR_OFFSET) / 4), &address, sizeof(ap_uint<32>));
			}
			//If the PCIe Mode is Disabled there is no Need to Set the Address Translation Registers of the PCIe Bridge.
			else
			{
				//Read the Source Physical Address from the Source Address Register from the Current Set of the Scheduler Buffer.
				memcpy(&source_address_register, (const ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + src_address_first_reg_offset + (repeat * step_offset)) / 4), sizeof(ap_uint<32>));

				//Read the Address Offset from the Offset Register from the Current Set of the Scheduler Buffer.
				//NOTE it is Possible that this Register Has a Zero Value if there is no Offset Required to Access the Data.
				memcpy(&offset_register, (const ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + offset_first_reg_offset + (repeat * step_offset)) / 4), sizeof(ap_uint<32>));

				//Set the Source Address Register of the CDMA Fetch Core with the Source Address along with the Offset Read from the Scheduler Buffer.
				address = source_address_register + offset_register;
				memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_SRCADDR_OFFSET) / 4), &address, sizeof(ap_uint<32>));

			}

			//Read the Destination Physical Address from the Destination Address Register from the Current Set of the Scheduler Buffer.
			memcpy(&destination_address_register, (const ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + dst_address_first_reg_offset + (repeat * step_offset)) / 4), sizeof(ap_uint<32>));

			//Set the Destination Address Register of the CDMA Fetch Core with the Destination Address.
			memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_DSTADDR_OFFSET) / 4), &destination_address_register, sizeof(ap_uint<32>));


			/*
			 * ----------------------------------------------------------------------------------------------------------------------------
			 * Read the Upper and Lower Registers of the Global Clock Counter of the Shared Timer to Get the CDMA Fetch Transfer Start Time
			 * ----------------------------------------------------------------------------------------------------------------------------
			 */

			//Read the Lower Register of the GCC of the Shared Timer to Get the 32 LSBs of the CDMA Fetch Transfer Start Time.
			memcpy(&cdma_fetch_time_start_gcc_l, (const ap_uint<32> *)(ext_cfg + (shared_apm_base_address + XAPM_GCC_LOW_OFFSET) / 4), sizeof(ap_uint<32>));

			//Store the 32 LSBs of the CDMA Fetch Transfer Start Time to a Specific Offset of the Metrics Memory.
			memcpy((ap_uint<32> *)(ext_cfg + (shared_metrics_base_address + (sizeof(struct metrics) * accel_group_jump) + (sizeof(struct metrics) * repeat) + CDMA_FETCH_TIME_START_L_OFFSET) / 4), &cdma_fetch_time_start_gcc_l, sizeof(ap_uint<32>));

			//Read the Upper Register of the GCC of the Shared Timer to Get the 32 MSBs of the CDMA Fetch Transfer Start Time.
			memcpy(&cdma_fetch_time_start_gcc_u, (const ap_uint<32> *)(ext_cfg + (shared_apm_base_address + XAPM_GCC_HIGH_OFFSET) / 4), sizeof(ap_uint<32>));

			//Store the 32 MSBs of the CDMA Fetch Transfer Start Time to a Specific Offset of the Metrics Memory.
			memcpy((ap_uint<32> *)(ext_cfg + (shared_metrics_base_address + (sizeof(struct metrics) * accel_group_jump) + (sizeof(struct metrics) * repeat) + CDMA_FETCH_TIME_START_U_OFFSET) / 4), &cdma_fetch_time_start_gcc_u, sizeof(ap_uint<32>));


			/*
			 * ---------------------------------------------------------------------------------------------
			 * Setup the Bytes To Transfer (BTT) Register of the CDMA Fetch Core which Triggers the Transfer
			 * ---------------------------------------------------------------------------------------------
			 */

			//Set the Bytes To Tranfer Register of the CDMA Fetch Core with the Transfer Size in Bytes.
			memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_BTT_OFFSET) / 4), &data_size_register, sizeof(ap_uint<32>));


			/*
			 * -------------------------
			 * Wait for a CDMA Interrupt
			 * -------------------------
			 */

			//Make an Initial Read of the Current State of the cdma_intr_in Input.
			cdma_intr_in_value = *cdma_intr_in;

			//Keep Looping for as long as the cdma_intr_in Input Does not Reach a Logic 1 Value.
			while(cdma_intr_in_value != 1)
			{
				//Keep Reading the Last Value of the cdma_intr_in Input.
				cdma_intr_in_value = *cdma_intr_in;
			}

			//Reset the Reader Variable.
			cdma_intr_in_value = 0;


			/*
			 * -------------------------------------------------------------------------------------------------------------------------
			 * Read the Upper and Lower Registers of the Global Clock Counter of the Shared Timer to Get the CDMA Fetch Tranfer End Time
			 * -------------------------------------------------------------------------------------------------------------------------
			 */

			//Read the Lower Register of the GCC of the Shared Timer to Get the 32 LSBs of the CDMA Fetch Tranfer End Time.
			memcpy(&cdma_fetch_time_end_gcc_l, (const ap_uint<32> *)(ext_cfg + (shared_apm_base_address + XAPM_GCC_LOW_OFFSET) / 4), sizeof(ap_uint<32>));

			//Store the 32 LSBs of the CDMA Fetch Tranfer End Time to a Specific Offset of the Metrics Memory.
			memcpy((ap_uint<32> *)(ext_cfg + (shared_metrics_base_address + (sizeof(struct metrics) * accel_group_jump) + (sizeof(struct metrics) * repeat) + CDMA_FETCH_TIME_END_L_OFFSET) / 4), &cdma_fetch_time_end_gcc_l, sizeof(ap_uint<32>));

			//Read the Upper Register of the GCC of the Shared Timer to Get the 32 MSBs of the CDMA Fetch Tranfer End Time.
			memcpy(&cdma_fetch_time_end_gcc_u, (const ap_uint<32> *)(ext_cfg + (shared_apm_base_address + XAPM_GCC_HIGH_OFFSET) / 4), sizeof(ap_uint<32>));

			//Store the 32 MSBs of the CDMA Fetch Tranfer End Time to a Specific Offset of the Metrics Memory.
			memcpy((ap_uint<32> *)(ext_cfg + (shared_metrics_base_address + (sizeof(struct metrics) * accel_group_jump) + (sizeof(struct metrics) * repeat) + CDMA_FETCH_TIME_END_U_OFFSET) / 4), &cdma_fetch_time_end_gcc_u, sizeof(ap_uint<32>));


			/*
			 * ------------------------------------
			 * Acknowledge the CDMA Fetch Interrupt
			 * ------------------------------------
			 */

			//Read the Status Register of the CDMA Fetch Core which among others Includes the Status of the DMA's IRQs.
			memcpy(&data_register, (const ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_SR_OFFSET) / 4), sizeof(ap_uint<32>));

			//Filter the Recently Read Value with the XAXICDMA_IRQ_ALL_MASK so as to Keep ONLY the IRQs that were Triggered.
			irq = data_register & XAXICDMA_XR_IRQ_ALL_MASK;

			//Write the new Value Back to the Status Register of the CDMA Fetch Core which Acknowledges the Triggered Interrupts.
			memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_SR_OFFSET) / 4), &irq, sizeof(ap_uint<32>));

			/*
			 * -------------------------
			 * Reset the CDMA Fetch Core
			 * -------------------------
			 */

			//Write the Reset Mask to the Control Register of the CDMA Fetch Core in order to Reset the Core.
			data_register = XAXICDMA_CR_RESET_MASK;
			memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));


			/*
			 * -----------------------------------------------
			 * Re-Enable the Interrupts on the CDMA Fetch Core
			 * -----------------------------------------------
			 */

			//Read the Control Register of the CDMA Fetch Core.
			memcpy(&data_register, (const ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

			//Set the Recently Read Value with the Masks Required to Enable the IOC, Delay and Error IRQs.
			//NOTE that IOC Stands for Interrupt On Complete.
			data_register = data_register | (XAXICDMA_XR_IRQ_ERROR_MASK | XAXICDMA_XR_IRQ_IOC_MASK | XAXICDMA_XR_IRQ_DELAY_MASK);

			//Write the new Value Back to the Control Register of the CDMA Fetch Core to Enable the Interrupts.
			memcpy((ap_uint<32> *)(ext_cfg + (cdma_base_address + XAXICDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

			/*
			 * Reset to Zero the 4 Registers of the Current Set of Registers of the Scheduler Buffer
			 */
			data_register = 0;
			memcpy((ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + src_address_first_reg_offset + (repeat * step_offset)) / 4), &data_register, sizeof(ap_uint<32>));
			memcpy((ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + dst_address_first_reg_offset + (repeat * step_offset)) / 4), &data_register, sizeof(ap_uint<32>));
			memcpy((ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + data_size_first_reg_offset + (repeat * step_offset)) / 4), &data_register, sizeof(ap_uint<32>));
			memcpy((ap_uint<32> *)(ext_cfg + (scheduler_buffer_base_address + offset_first_reg_offset + (repeat * step_offset)) / 4), &data_register, sizeof(ap_uint<32>));


			/*
			 * Each Iteration Refers to a Specific AGI.
			 * Check the Current Iteration Value and Start the Acceleration Scheduler Indirect of the Correct AGI.
			 */
			if(repeat == 0)
			{
				//Trigger the start_0 Signal for one Clock Cycle.
				*start_0 = 0;
				*start_0 = 1;
			}

			if(repeat == 1)
			{
				//Trigger the start_1 Signal for one Clock Cycle.
				*start_1 = 0;
				*start_1 = 1;
			}

			if(repeat == 2)
			{
				//Trigger the start_2 Signal for one Clock Cycle.
				*start_2 = 0;
				*start_2 = 1;
			}

			if(repeat == 3)
			{
				//Trigger the start_3 Signal for one Clock Cycle.
				*start_3 = 0;
				*start_3 = 1;
			}
		}
	}
}

return 1;

}


