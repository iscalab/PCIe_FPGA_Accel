/*******************************************************************************
* Filename:   interrupt_manager.cpp
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
#include "interrupt_manager.h"


/*
 * interrupt_manager()
 *
 * The Hardware Funtionality of the Interrupt Manager Core.
 *
 * The Interrupt Manager Core is Developed to Handle and Forward the Completion Interrupts from the 7 Acceleration Groups.
 *
 * The Goal of each Acceleration Group is to Inform the Linux Kernel Driver for the Completion of the Acceleration Procedure.
 * The Communication of the FPGA with the Host System is Achieved through a PCIe Bus, thus, the Way to Signal the Driver is to Send MSI Interrupts.
 *
 * Sending a MSI is a Responsibility of the FPGA's PCIe Bridge.
 * The PCIe Bridge Carries a 5-Bit Input to Set the Vector Number of the MSI and a 1-Bit Input which is Used to Trigger the MSI According to the Vector Number.
 * In the Current Block Design the 2 Inputs of the PCIe Bridge are Connected with the Two Channels of a GPIO Peripheral.
 * This GPIO from now on will be Recognized as GPIO-MSI.
 * Writing Values in the Data Registers of the 2 Channels of the GPIO-MSI Leads to Triggering a MSI Interrupt.
 *
 * In Older Approaches the Acceleration Scheduler of each Acceleration Group would Simply Access the GPIO-MSI to Send MSI Interrupts on Completion of an Image Process.
 * This Approach was Proved to be Unreliable Since the Concurrent Access to the GPIO-MSI by Multiple Acceleration Groups
 * Could Lead to Possible Loss of Interrupts that were NEVER Transmitted.
 *
 * The new Approach to Ensure Zero Loss of Interrupts was to Develop the Current Interrupt Manager.
 * The Interrupt Manager Includes an Array of 7 Registers where each Register Refers to each of the 7 Acceleration Groups.
 *
 * Register_Array[0] Refers to AGD0
 * Register_Array[1] Refers to AGD1
 * Register_Array[2] Refers to AGI0
 * Register_Array[3] Refers to AGI1
 * Register_Array[4] Refers to AGI2
 * Register_Array[5] Refers to AGI3
 * Register_Array[6] Refers to AGSG
 *
 * When an Acceleration Scheduler of any of the Acceleration Groups Requires to Send an MSI Interrupt for the Completion of its Acceleration Procedure
 * it Simply Writes a Vector Number Value to the Corresponding Field of the Register Array of the Interrupt Manager as a MSI Request.
 * The Kernel Driver Identifies the Acceleration Group that "Sent" the MSI by the Vector Number.
 *
 * Vector Number:0 --> AGD0
 * Vector Number:1 --> AGD1
 * Vector Number:2 --> AGI0
 * Vector Number:3 --> AGI1
 * Vector Number:4 --> AGI2
 * Vector Number:5 --> AGI3
 * Vector Number:6 --> AGSG
 *
 * The Interrupt Manager Checks in a Round Robin Manner the Fields of the Register Array for a Non-Zero Value which Indicates a new MSI Request.
 * This Makes it Obvious that the Acceleration Schedulers Write to the Register Array of the Interrupt Manager the Vector Number Incremented by 1.
 * This is Done to Avoid Zero Values that are not Identified by the Interrupt Manager as MSI Requests.
 *
 * If the Interrupt Manager Finds a Field of the Register Array with Non-Zero Value then it Decreases this Value by 1 in order to Produce
 * the Correct Vector Number and Writes this Value to the GPIO-MSI Peripheral to Trigger the MSI Interrupt.
 *
 * The Interrupt Manager, then, Waits until it Receives an Acknowledgment Signal from the Kernel Driver before Checking for another MSI Request.
 * The Kernel Driver, actually, Writes a Logic 1 Value to another GPIO Peripheral whose 1-Bit Output Signals the Interrupt Manager.
 * This GPIO Peripheral from now on will be Recognized as GPIO-ACK.
 *
 *
 * The Sequential Steps of the Interrupt Management are as Follows:
 *
 * Start a for Loop with 7 Iterations where each Iteration is to Check for a MSI Request by the Corresponding Acceleration Group.
 * NOTE Enabling the Auto Restart Mode of the Current Core will Lead to Starting Over the for Loop.
 *
 * a --> Check if the Current Field of the Register Array Has a Non-Zero Value.
 *       If this is the Case Proceed to Send a MSI Interrupt.
 * b --> Decrease the Value of the Current Field of the Register Array to Get the Correct Vector Number.
 * c --> Write the Vector Number to the GPIO-MSI Peripheral that is Connected with the PCIe Bridge to Trigger an MSI Interrupt.
 * d --> Wait for an Acknowledgment Signal from the Driver through the GPIO-ACK Peripheral.
 * e --> Self-Clear to Zero the Current Field of the Register Array of the Interrupt Manager.
 *       The Next Time we Find a Non-Zero Value in this Field we Know that an Acceleration Group Has Made a Valid MSI Request.
 * f --> Clear the Data Register of the GPIO-ACk Peripheral.
 *
 * The Function Parameters are the Input/Output Ports/Interfaces of the Core:
 *
 * 01 --------> The AXI Master Interface of the Core Used to Access External Devices and Memories.
 * 02 --------> Single Bit Input Used to Receive External Acknowledgements from the Linux Kernel Driver.
 * 03 to 06 --> Registers of the Core that are Accessed through the AXI Slave Lite Interface of the Core.
 */
int interrupt_manager(/*01*/volatile ap_uint<32> *ext_cfg,
                      /*02*/volatile ap_uint<1> *intr_ack,
                      /*03*/unsigned int gpio_msi_device_address,
                      /*04*/unsigned int gpio_ack_device_address,
                      /*05*/unsigned int self_msi_request_offset,
                      /*06*/unsigned int msi_request[7]
                      )
{

/*
 * The ext_cfg is the AXI Master Interface of the Core.
 */
#pragma HLS INTERFACE m_axi port=ext_cfg

/*
 * The gpio_msi_device_address is a Register to Store the Base Address of the GPIO-MSI that this Core
 * will Need to Access through the ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=gpio_msi_device_address bundle=cfg

/*
 * The gpio_ack_device_address is a Register to Store the Base Address of the GPIO-ACK that this Core
 * will Need to Access through the ext_cfg AXI Master Interface.
 * This Register is Accessed through the AXI Slave Lite Interface (cfg) of the Core.
 */
#pragma HLS INTERFACE  s_axilite  port=gpio_ack_device_address bundle=cfg

/*
 * The self_msi_request_offset is a Register to Store the Address Offset where the Register Array (msi_request) is Located.
 * This Address Offset Actually Leads the Interrupt Manager to Access its Own Configuration Registers through its AXI Slave Lite (cfg) Interface.
 */
#pragma HLS INTERFACE  s_axilite  port=self_msi_request_offset bundle=cfg

#pragma HLS INTERFACE s_axilite port=msi_request bundle=cfg

/*
 * The intr_ack is a Single Bit Input which is Used to Receive External Acknowledgements from the Linux Kernel Driver.
 */
#pragma HLS INTERFACE ap_none port=intr_ack

#pragma HLS INTERFACE  s_axilite  port=return bundle=cfg

ap_uint<32> data_register; // Used to Temporalily Store Values when Reading or Writing from/to Registers of External Devices.
ap_uint<1> intr_ack_value; // Used to Read the Last Value of the intr_ack Input Port.


for(int repeat = 0; repeat < 7; repeat++)
{

	//If the Current Field of the Register Array (msi_request) Has a Non-Zero Value then we Have a Valid MSI Request by the Corresponding Acceleration Group.
	if(msi_request[repeat] != 0)
	{
		/*
		 * ---------------------------------------------------------
		 * Send a MSI Interrupt by Writing to the GPIO-MSI Registers
		 * ---------------------------------------------------------
		 */

		//Decrease the Value of the Current Field of the Register Array to Get the Correct Vector Number.
		data_register = msi_request[repeat] - 1;

		//Write the Vector Number to the Data Register of the Second Channel of the GPIO-MSI.
		memcpy((ap_uint<32> *)(ext_cfg + (gpio_msi_device_address + XGPIO_DATA_OFFSET + XGPIO_CHANNEL_2_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

		//Write a Logic 1 Value to the Data Register of the First Channel of the GPIO-MSI to Trigger the MSI Interrupt.
		data_register = 0x1;
		memcpy((ap_uint<32> *)(ext_cfg + (gpio_msi_device_address + XGPIO_DATA_OFFSET + XGPIO_CHANNEL_1_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

		//Set the Data Register of the First Channel of the GPIO-MSI back to Zero.
		data_register = 0x0;
		memcpy((ap_uint<32> *)(ext_cfg + (gpio_msi_device_address + XGPIO_DATA_OFFSET + XGPIO_CHANNEL_1_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));


		/*
		 * ------------------------------------
		 * Wait for a Interrupt Acknowledgement
		 * ------------------------------------
		 */

		//Make an Initial Read of the Current State of the intr_ack Input.
		intr_ack_value = *intr_ack;

		//Keep Looping for as long as the intr_ack Input Does not Reach a Logic 1 Value.
		while(intr_ack_value != 1)
		{
			//Keep Reading the Last Value of the intr_ack Input.
			intr_ack_value = *intr_ack;
		}

		//Reset the Reader Variable.
		intr_ack_value = 0;

		data_register = 0x0;

		/* ---------------------------------------------------------------------------
		 * Self-Clear the Current Field of the Register Array of the Interrupt Manager
		 * ---------------------------------------------------------------------------
		 */

		//Write a Zero Value to the Current Field of the Register Array of the Interrupt Manager to Clear the Field.
		//NOTE the Interrupt Manager Herein Uses its AXI Master Interface to Write to its own AXI Slave Lite Interface.
		memcpy((ap_uint<32> *)(ext_cfg + (self_msi_request_offset + (repeat * 4)) / 4), &data_register, sizeof(ap_uint<32>));

		/*
		 * -----------------------------
		 * Clear the GPIO-ACK Peripheral
		 * -----------------------------
		 */

		//Clear the GPIO-ACK by Writing a Zero Value to its Data Register.
		memcpy((ap_uint<32> *)(ext_cfg + (gpio_ack_device_address + XGPIO_DATA_OFFSET + XGPIO_CHANNEL_1_OFFSET) / 4), &data_register, sizeof(ap_uint<32>));

	}
}

return 1;

}


