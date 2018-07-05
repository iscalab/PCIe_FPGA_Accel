/*******************************************************************************
* Filename:   acceleration_scheduler_direct.cpp
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
#include "acceleration_scheduler_direct.h"

/*
 * -----------------------------
 * Registers of the Sobel Filter
 * -----------------------------
 */
#define XSOBEL_FILTER_S_AXI4_LITE_ADDR_AP_CTRL   0x00
#define XSOBEL_FILTER_S_AXI4_LITE_ADDR_ROWS_DATA 0x18
#define XSOBEL_FILTER_S_AXI4_LITE_ADDR_COLS_DATA 0x20


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
 * -------------------------------------------------------------
 * Registers and Masks of the AXI Performance Monitor Unit (APM)
 * -------------------------------------------------------------
 */
#define XAPM_CR_GCC_RESET_MASK    0x00020000 // Global Clock Counter (GCC) Reset Mask.
#define XAPM_CR_GCC_ENABLE_MASK   0x00010000 // Global Clock Counter (GCC) Enable Mask.
#define XAPM_CR_MCNTR_RESET_MASK  0x00000002 // Metrics Counter Reset Mask.
#define XAPM_CR_MCNTR_ENABLE_MASK 0x00000001 // Metrics Counter Enable Mask.

#define XAPM_CTL_OFFSET      0x0300 // Control Register Offset.
#define XAPM_GCC_HIGH_OFFSET 0x0000 // Global Clock Counter 32 to 63 bits (Upper) Register Offset.
#define XAPM_GCC_LOW_OFFSET  0x0004 // Global Clock Counter  0 to 31 bits (Lower) Register Offset.

#define XAPM_MC0_OFFSET 0x0100 // Metrics Counter 0 Register Offset.
#define XAPM_MC1_OFFSET	0x0110 // Metrics Counter 1 Register Offset.
#define XAPM_MC2_OFFSET 0x0120 // Metrics Counter 2 Register Offset.
#define XAPM_MC3_OFFSET 0x0130 // Metrics Counter 3 Register Offset.
#define XAPM_MC4_OFFSET 0x0140 // Metrics Counter 4 Register Offset.
#define XAPM_MC5_OFFSET 0x0150 // Metrics Counter 5 Register Offset.

/*
 * acceleration_scheduler_direct()
 *
 * The Hardware Funtionality of the Acceleration Scheduler Direct Core.
 *
 * The Acceleration Scheduler Direct Core is Part of the Acceleration Group Direct and is Used to Manage the whole Acceleration Procedure.
 * It Interacts with the DMA, Sobel Filter and APM of the Acceleration Group Direct as well as the Shared Timer (Shared APM) to Get Time Metrics.
 * It, also, Interacts with the Interrupt Manager to Signalize the Completion of the Acceleration Procedure.
 *
 * The Sequential Steps of the Acceleration Procedure are as Follows:
 *
 * a --> Enable the Counters of the AXI Performance Monitor Unit (APM).
 * b --> Read the Current Value of the Shared Timer to Get the Time that the Acceleration Started.
 * c --> Setup and Start the Sobel Filter.
 * d --> Setup and Start the S2MM and MM2S DMA Transfers.
 * e --> Wait for an Interrupt by the DMA on Completion of the Transfer.
 * f --> Read the Current Value of the Shared Timer to Get the Time that the Acceleration Ended.
 * g --> Disable the Counters of the AXI Performance Monitor Unit (APM).
 * h --> Acknowledge the DMA Interrupt.
 * i --> Collect the Metrics from the Counters of the AXI Performance Monitor Unit (APM).
 * j --> Reset the Counters of the AXI Performance Monitor Unit (APM).
 * k --> Inform the Interrupt Manager About the Completion of the Acceleration Procedure.
 *
 * The Function Parameters are the Input/Output Ports/Interfaces of the Core:
 *
 * 01 --------> The AXI Master Interface of the Core Used to Access External Devices and Memories.
 * 02 --------> Single Bit Input Used to Receive External Interrupts from the DMA.
 * 03 to 13 --> Registers of the Core that are Accessed through the AXI Slave Lite Interface of the Core.
 */
int acceleration_scheduler_direct(/*01*/volatile ap_uint<32> *mm2s_ext_cfg,
                                  /*02*/volatile ap_uint<1> *dma_intr_in,
                                  /*03*/unsigned int dma_device_address,
                                  /*04*/unsigned int sobel_device_address,
                                  /*05*/unsigned int interrupt_manager_register_offset,
                                  /*06*/unsigned int apm_device_address,
                                  /*07*/unsigned int shared_apm_device_address,
                                  /*08*/unsigned int shared_metrics_address,
                                  /*09*/unsigned int image_cols,
                                  /*10*/unsigned int image_rows,
                                  /*11*/unsigned int host_mem_src_data_address,
                                  /*12*/unsigned int host_mem_dst_data_address,
                                  /*13*/unsigned int initiator_group
					              )
{

/*
 * The mm2s_ext_cfg is the AXI Master Interface of the Core.
 */
#pragma HLS INTERFACE m_axi port=mm2s_ext_cfg

/*
 * The dma_intr_in is a Single Bit Input which is Used to Receive External Interrupts from the DMA.
 */
#pragma HLS INTERFACE ap_none port=dma_intr_in

/*
 * The dma_device_address is a Register to Store the Base Address of the DMA that this Core
 * will Need to Access through the mm2s_ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=dma_device_address bundle=mm2s_cfg

/*
 * The sobel_device_address is a Register to Store the Base Address of the Sobel Filter that this Core
 * will Need to Access through the mm2s_ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=sobel_device_address bundle=mm2s_cfg

/*
 * The interrupt_manager_register_offset is a Register to Store the Offset of a Specific Register of the Interrupt Manager that this Core
 * will Need to Access through the mm2s_ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=interrupt_manager_register_offset bundle=mm2s_cfg

/*
 * The apm_device_address is a Register to Store the Base Address of the AXI Performance Monitor Unit (APM) that this Core
 * will Need to Access through the mm2s_ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=apm_device_address bundle=mm2s_cfg

/*
 * The shared_apm_device_address is a Register to Store the Base Address of the Shared Timer (APM) that this Core
 * will Need to Access through the mm2s_ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=shared_apm_device_address bundle=mm2s_cfg

/*
 * The shared_metrics_address is a Register to Store the Base Address of the Memory that this Core
 * will Need to Access through the mm2s_ext_cfg AXI Master Interface in Order to Write the Metrics Information.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=shared_metrics_address bundle=mm2s_cfg

/*
 * The host_mem_src_data_address is a Register to Store the Source Address that the DMA will Use to Read the Initial Image Data.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=host_mem_src_data_address bundle=mm2s_cfg

/*
 * The host_mem_dst_data_address is a Register to Store the Destination Address that the DMA will Use to Write the Processed Image Data.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=host_mem_dst_data_address bundle=mm2s_cfg

/*
 * The image_cols is a Register to Store the Number of Columns of the Image that will be Accelerated.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=image_cols bundle=mm2s_cfg

/*
 * The image_rows is a Register to Store the Number of Rows of the Image that will be Accelerated.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=image_rows bundle=mm2s_cfg

/*
 * The initiator_group is a Register to Store the Acceleration Group Number (0-6) that this Core Belongs to.
 * This Register is Accessed through the AXI Slave Lite Interface (mm2s_cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=initiator_group bundle=mm2s_cfg

#pragma HLS INTERFACE  s_axilite  port=return bundle=mm2s_cfg



ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.
ap_uint<32> initial_data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.

ap_uint<32> read_transactions; // Store the Read Transactions from the APM.
ap_uint<32> read_bytes; // Store the Read Bytes from the APM.

ap_uint<32> write_transactions; // Store the Write Transactions from the APM.
ap_uint<32> write_bytes; // Store the Write Bytes from the APM.

ap_uint<32> stream_packets; // Store the Stream Packets from the APM.
ap_uint<32> stream_bytes; // Store the Stream Bytes from the APM.

ap_uint<32> gcc_lower; // Store the Global Clock Counter Lower Register from the APM.
ap_uint<32> gcc_upper; // Store the Global Clock Counter Upper Register from the APM.

ap_uint<32> dma_accel_time_start_gcc_l; // Store the Acceleration Start Time Lower Register from the Shared Timer (Shared APM).
ap_uint<32> dma_accel_time_start_gcc_u; // Store the Acceleration Start Time Upper Register from the Shared Timer (Shared APM).

ap_uint<32> dma_accel_time_end_gcc_l; // Store the Acceleration End Time Lower Register from the Shared Timer (Shared APM).
ap_uint<32> dma_accel_time_end_gcc_u; // Store the Acceleration End Time Upper Register from the Shared Timer (Shared APM).

ap_uint<1> dma_intr_in_value; // Used to Read the Last Value of the dma_intr_in Input Port.



/*
 * -----------------------
 * Enable the APM Counters
 * -----------------------
 */

//Read the Control Register of the APM.
memcpy(&data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Masks Required to Enable the GCC and Metrics Counters.
data_register = data_register | XAPM_CR_GCC_ENABLE_MASK | XAPM_CR_MCNTR_ENABLE_MASK;

//Write the new Value Back to the Control Register of the APM to Enable the GCC and Metrics Counters.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));


/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Read the Upper and Lower Registers of the Global Clock Counter of the Shared Timer to Get DMA Acceleration Start Time
 * ---------------------------------------------------------------------------------------------------------------------
 */

//Read the Lower Register of the GCC of the Shared Timer to Get the 32 LSBs of the Acceleration Start Time.
memcpy(&dma_accel_time_start_gcc_l, (const ap_uint<32> *)(mm2s_ext_cfg + (shared_apm_device_address + XAPM_GCC_LOW_OFFSET) / 4), sizeof(ap_uint<32>));

//Store the 32 LSBs of the Acceleration Start Time to a Specific Offset of the Metrics Memory.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + DMA_ACCEL_TIME_START_L_OFFSET) / 4), &dma_accel_time_start_gcc_l, sizeof(ap_uint<32>));


//Read the Upper Register of the GCC of the Shared Timer to Get the 32 MSBs of the Acceleration Start Time.
memcpy(&dma_accel_time_start_gcc_u, (const ap_uint<32> *)(mm2s_ext_cfg + (shared_apm_device_address + XAPM_GCC_HIGH_OFFSET) / 4), sizeof(ap_uint<32>));

//Store the 32 MSBs of the Acceleration Start Time to a Specific Offset of the Metrics Memory.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + DMA_ACCEL_TIME_START_U_OFFSET) / 4), &dma_accel_time_start_gcc_u, sizeof(ap_uint<32>));


/*
 * --------------------------------
 * Setup and Start the Sobel Filter
 * --------------------------------
 */

//Get the Sobel Filter Columns from the Internal Register (image_cols) of the Core.
data_register = image_cols;

//Write the Sobel Filter Columns to a Specific Offset of the Sobel Filter Device.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (sobel_device_address + XSOBEL_FILTER_S_AXI4_LITE_ADDR_COLS_DATA) / 4), &data_register, sizeof(ap_uint<32>));

//Get the Sobel Filter Rows from the Internal Register (image_rows) of the Core.
data_register = image_rows;

//Write the Sobel Filter Rows to a Specific Offset of the Sobel Filter Device.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (sobel_device_address + XSOBEL_FILTER_S_AXI4_LITE_ADDR_ROWS_DATA) / 4), &data_register, sizeof(ap_uint<32>));


//Read the Control Register of the Sobel Filter.
memcpy(&data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (sobel_device_address + XSOBEL_FILTER_S_AXI4_LITE_ADDR_AP_CTRL) / 4), sizeof(ap_uint<32>));

//Set the Appropriate Masks According to the Recently Read Value that Will be Needed to Start the Sobel Filter.
data_register = data_register & 0x80;
data_register = data_register | 0x01;

//Write the new Value Back to the Control Register of the Sobel Filter so that the Sobel Filter Gets Started.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (sobel_device_address + XSOBEL_FILTER_S_AXI4_LITE_ADDR_AP_CTRL) / 4), &data_register, sizeof(ap_uint<32>));



/*
 * ---------------------------------------------
 * Setup and Start Device to DMA Transfer (S2MM)
 * ---------------------------------------------
 */

//Get from the Internal Register (host_mem_dst_data_address) of the Core the Destination Address that the DMA will Use to Write the Processed Image Data.
data_register = host_mem_dst_data_address;

//Write the Destination Address to the Destination Register of the DMA.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_DESTADDR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

//Read the S2MM Control Register of the DMA.
memcpy(&data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Mask Required to Enable the S2MM DMA Channel.
data_register = data_register | XAXIDMA_CR_RUNSTOP_MASK;

//Write the new Value Back to the Control Register of the DMA in Order to Enable the S2MM Channel.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

//Calculate the Image/Transfer Size According to the Internal Registers (image_cols, image_rows) of the Core.
data_register = (image_cols * image_rows) * 4;

//Write the Transfer Size to the S2MM Length Register of the DMA which Starts the S2MM Transfer.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_BUFFLEN_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));



/*
 * ---------------------------------------------
 * Setup and Start DMA to Device Transfer (MM2S)
 * ---------------------------------------------
 */

//Get from the Internal Register (host_mem_src_data_address) of the Core the Source Address that the DMA will Use to Read the Initial Image Data.
data_register = host_mem_src_data_address;

//Write the Source Address to the Source Register of the DMA.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_SRCADDR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

//Read the MM2S Control Register of the DMA.
memcpy(&data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Mask Required to Enable the MM2S DMA Channel.
data_register = data_register | XAXIDMA_CR_RUNSTOP_MASK;

//Write the new Value Back to the Control Register of the DMA in Order to Enable the MM2S Channel.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

//Calculate the Image/Transfer Size According to the Internal Registers (image_cols, image_rows) of the Core.
data_register = (image_cols * image_rows) * 4;

//Write the Transfer Size to the MM2S Length Register of the DMA which Starts the MM2S Transfer.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_TX_OFFSET + XAXIDMA_BUFFLEN_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));


/*
 * ------------------------
 * Wait for a DMA Interrupt
 * ------------------------
 */

//Make an Initial Read of the Current State of the dma_intr_in Input.
dma_intr_in_value = *dma_intr_in;

//Keep Looping for as long as the dma_intr_in Input Does not Reach a Logic 1 Value.
while(dma_intr_in_value != 1)
{
	//Keep Reading the Last Value of the dma_intr_in Input.
	dma_intr_in_value = *dma_intr_in;
}

//Reset the Reader Variable.
dma_intr_in_value = 0;


/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Read the Upper and Lower Registers of the Global Clock Counter of the Shared Timer to Get DMA Acceleration End Time
 * ---------------------------------------------------------------------------------------------------------------------
 */

//Read the Lower Register of the GCC of the Shared Timer to Get the 32 LSBs of the Acceleration End Time.
memcpy(&dma_accel_time_end_gcc_l, (const ap_uint<32> *)(mm2s_ext_cfg + (shared_apm_device_address + XAPM_GCC_LOW_OFFSET) / 4), sizeof(ap_uint<32>));

//Store the 32 LSBs of the Acceleration End Time to a Specific Offset of the Metrics Memory.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + DMA_ACCEL_TIME_END_L_OFFSET) / 4), &dma_accel_time_end_gcc_l, sizeof(ap_uint<32>));

//Read the Upper Register of the GCC of the Shared Timer to Get the 32 MSBs of the Acceleration End Time.
memcpy(&dma_accel_time_end_gcc_u, (const ap_uint<32> *)(mm2s_ext_cfg + (shared_apm_device_address + XAPM_GCC_HIGH_OFFSET) / 4), sizeof(ap_uint<32>));

//Store the 32 MSBs of the Acceleration End Time to a Specific Offset of the Metrics Memory.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + DMA_ACCEL_TIME_END_U_OFFSET) / 4), &dma_accel_time_end_gcc_u, sizeof(ap_uint<32>));



/*
 * ------------------------
 * Disable the APM Counters
 * ------------------------
 */

//Read the Control Register of the APM.
memcpy(&data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Masks Accordingly to Disable the GCC and Metrics Counters.
data_register = data_register & ~(XAPM_CR_GCC_ENABLE_MASK) & ~(XAPM_CR_MCNTR_ENABLE_MASK);

//Write the new Value Back to the Control Register of the APM to Disable the GCC and Metrics Counters.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));



/*
 * ---------------------------------------------------------------------
 * Read the DMA S2MM Status Register to Get the IRQs (IOC, Delay, Error)
 * IOC Stands for: Interrupt On Complete
 * ---------------------------------------------------------------------
 */

//Read the S2MM Status Register of the DMA which among others Includes the Status of the DMA's IRQs.
memcpy(&data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET) / 4), sizeof(ap_uint<32>));

//Filter the Recently Read Value with the XAXIDMA_IRQ_ALL_MASK so as to Keep ONLY the IRQs that were Triggered.
data_register = data_register & XAXIDMA_IRQ_ALL_MASK;

/*
 * ------------------------------------
 * Acknowledge the Triggered Interrupts
 * ------------------------------------
 */

//Write the new Value Back to the Status Register of the DMA which Acknowledges the Triggered Interrupts.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (dma_device_address + XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));



/*
 * --------------------------------------------------------------------------
 * Read the APM Metrics Counters and Store their Values to the Metrics Memory
 * --------------------------------------------------------------------------
 */

//Get the Read Transactions from the APM and Write it to the Shared Metrics Memory.
memcpy(&read_transactions, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_MC0_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_READ_TRANSACTIONS_OFFSET) / 4), &read_transactions, sizeof(ap_uint<32>));

//Get the Read Bytes from the APM and Write it to the Shared Metrics Memory.
memcpy(&read_bytes, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_MC1_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_READ_BYTES_OFFSET) / 4), &read_bytes, sizeof(ap_uint<32>));

//Get the Write Transactions from the APM and Write it to the Shared Metrics Memory.
memcpy(&write_transactions, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_MC2_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_WRITE_TRANSACTIONS_OFFSET) / 4), &write_transactions, sizeof(ap_uint<32>));

//Get the Write Bytes from the APM and Write it to the Shared Metrics Memory.
memcpy(&write_bytes, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_MC3_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_WRITE_BYTES_OFFSET) / 4), &write_bytes, sizeof(ap_uint<32>));

//Get the Stream Packets from the APM and Write it to the Shared Metrics Memory.
memcpy(&stream_packets, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_MC4_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_PACKETS_OFFSET) / 4), &stream_packets, sizeof(ap_uint<32>));

//Get the Stream Bytes from the APM and Write it to the Shared Metrics Memory.
memcpy(&stream_bytes, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_MC5_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_BYTES_OFFSET) / 4), &stream_bytes, sizeof(ap_uint<32>));

//Get the GCC Lower Register from the APM and Write it to the Shared Metrics Memory.
memcpy(&gcc_lower, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_GCC_LOW_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_GCC_L_OFFSET) / 4), &gcc_lower, sizeof(ap_uint<32>));

//Get the GCC Upper Register from the APM and Write it to the Shared Metrics Memory.
memcpy(&gcc_upper, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_GCC_HIGH_OFFSET) / 4), sizeof(ap_uint<32>));
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (shared_metrics_address + (sizeof(struct metrics) * initiator_group) + APM_GCC_U_OFFSET) / 4), &gcc_upper, sizeof(ap_uint<32>));


/*
 * ----------------------
 * Reset the APM Counters
 * ----------------------
 */

//Read the Control Register of the APM.
memcpy(&initial_data_register, (const ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), sizeof(ap_uint<32>));

//Set the Recently Read Value with the Masks Accordingly to Reset the GCC and Metrics Counters.
data_register = initial_data_register | XAPM_CR_GCC_RESET_MASK | XAPM_CR_MCNTR_RESET_MASK;

//Write the new Value Back to the Control Register of the APM to Reset the GCC and Metrics Counters.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

//Now Reverse the Value of the Previous Masks in order to Release the Reset.
data_register = initial_data_register & ~(XAPM_CR_GCC_RESET_MASK) & ~(XAPM_CR_MCNTR_RESET_MASK);

//Write the new Value Back to the Control Register of the APM to Release the Reset.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (apm_device_address + XAPM_CTL_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));


/*
 * ------------------------------------------------------------------------------------
 * Inform the Interrupt Manager that this Core Has Completed the Acceleration Procedure
 * ------------------------------------------------------------------------------------
 */

//Get from the Internal Register (initiator_group) of the Core the Current Acceleration Group Number that this Core Belongs to.
data_register = initiator_group;

//Write the Current Acceleration Group Number to a Specific Register of the Interrupt Manager to Let It Know which Acceleration Group Has Completed.
memcpy((ap_uint<32> *)(mm2s_ext_cfg + (interrupt_manager_register_offset) / 4), &data_register, sizeof(ap_uint<32>));


return 1;


}


