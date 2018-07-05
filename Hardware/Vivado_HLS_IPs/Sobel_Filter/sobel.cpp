/*******************************************************************************
* Filename:   sobel.cpp
* Authors:    Othon Tomoutzoglou <otto_sta@hotmail.com>
*             Dimitrios Bakoyiannis <d.bakoyiannis@gmail.com>
* License:
*
* MIT License
*
* Copyright (c) [2018] [Othon Tomoutzoglou, Dimitrios Bakoyiannis]
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

#include "sobel.h"
#include "sobel_operations.h"
#include "packet_mode_operations.h"

/*
 * sobel_filter()
 *
 * The Hardware Funtionality of the Sobel Filter (HW Accelerator) Core.
 *
 * The Sobel Filter is a HW Accelerator that Applies Sobel Edge Detection on Images.
 * It Receives and Processes the Image Data in Rows.
 * In order to Produce one Processed Row it Requires 3 Received Rows.
 * This Precondition is due to the Fact that Edge Detection is Applied to a Pixel according to its Neighbor Pixels.
 *
 * Once the Sobel Filter Receives the First 3 Rows it Produces one Processed Row.
 * Then it Rejects the First Row, Sets the Second Row as First and Sets the Last Row as Second.
 * The Next/Newly Received Row is Set as the Last of the Rows.
 * Now there are, again, 3 Rows in Order to Produce the Next Processed Row.
 * This Procedure Carries on until all the Rows of the Image are Received and Processed.
 *
 * NOTE that the First and Last Rows of the Processed Image are Filled with Dark Pixels.
 * NOTE also that the First and Last Columns of all the Rows of the Processed Image are Filled with Dark Pixels.
 *
 * The Sobel Edge Detection Cannot be Applied to the Perimetric Pixels of the Image Since they Miss the Required Amount of Neighbors
 * this is why they are Filled with Dark Pixels.
 *
 * The Sequential Steps of the Sobel Filter are as Follows:
 *
 * a --> Send the First Row which is Filled with Dark Pixels.
 * b --> Pre-Fetch the 3 First Rows of the Image.
 * c --> Process the 3 Rows.
 * d --> Fill the First and Last Columns of the Produced Row with Dark Pixels.
 * e --> Send the Produced Row.
 * f --> Receive the Next Row.
 * g --> Start Again from Step c Until Receiving and Processing all the Rows.
 * h --> Send the Last Row which is Filled with Dark Pixels.
 *
 * The Function Parameters are the Input/Output Ports/Interfaces of the Core:
 *
 * 01 --------> The AXI Stream Input/Slave Interface of the Core Used to Receive the Image Data.
 * 02 --------> The AXI Stream Output/Master Interface of the Core Used to Forward the Processed Image Data.
 * 03 to 06 --> Registers of the Core that are Accessed through the AXI Slave Lite Interface of the Core.
 *
 *
 * IMPORTANT TECHNIQUES Used to Improve the Overall Performance:
 *
 * A)Each Image Row is not Received in a Single Buffer.
 *   Instead, while it is being Received it is Equally Splitted and Distributed in 16 Sector Buffers.
 *   Each Sector Buffer has no Dependence with the Rest Sector Buffers so the 16 Pieces of the Image Row Can be Processed in Parallel.
 *   The HLS Tool Creates 16 Processing Units to Make Parallel Processing Possible.
 *
 *
 * B)Another Improvement Technique is the Usage of Four Line Sector Buffers which Allows
 *   the Core to Process the Current 3 Rows while Concurrently Receiving the Next Row.
 *   The Four Line Sector Buffer is Designed with 4 Lines where each is Used to Store the Data of a Single Row.
 *   When the Sobel Filter Receives and Fills the 3 First Lines with 3 Rows it Starts the Processing.
 *   The Fourth Line is Free to Start Receiving the Next Row while the Rest 3 Lines are Occupied with the Processing.
 *
 *
 * C)In Older Approaches after 3 Lines of a Three Line Sector Buffer were Processed the Lines would
 *   Have to be Shifted Up so that the Last Line Could be Fed with the Next Received Row.
 *   This Approach Required a Significant Amount of Copies where each Pixel of a Line of the Sector Buffer would Have to be Copied to the Upper Line.
 *   The new Technique Requires Zero Copies as it Uses Indexing to Store the Received Rows in the Four Line Sector Buffer.
 *
 *   Indexing Concerns which Should be Considered as the First, Second and Third Row to Process and where the Next Received Row Should be Stored.
 *
 *   Initially: the First Received Row is Stored in the Line with Index 0 of the Four Line Sector Buffer.
 *            : the Second Received Row is Stored in the Line with Index 3 of the Four Line Sector Buffer.
 *            : the Third Received Row is Stored in the Line with Index 2 of the Four Line Sector Buffer.
 *            : the Line with Index 1 is Used to Store the Next Received Line while the Other 3 are being Processed.
 *
 *   When the Process of the 3 Lines Completes and a new Row is Received then the Indexing Changes so that we Can Start a new Processing and Receive another Row.
 *   Now the First Row is no Longer Needed so the Line with Index 0 will be Used to Receive the Next Row.
 *   The Second Row Becomes the First Row for the New Processing so the Line with Index 3 will be Used as the First Row.
 *   The Third Row Becomes the Second Row for the New Processing so the Line with Index 2 will be Used as the Second Row.
 *   The Last Received Row Becomes the Third Row for the New Processing so the Line with Index 1 will be Used as the Third Row.
 *
 *   Following the Same Pattern as to which Lines to Process and where to Store the Next Row Leads to the Table Below:
 *
 *   Index 0 | First Row  | Next Row   | Third Row  | Second Row |
 *   Index 1 | Next Row   | Third Row  | Second Row | First Row  |
 *   Index 2 | Third Row  | Second Row | First Row  | Next Row   |
 *   Index 3 | Second Row | First Row  | Next Row   | Third Row  |
 *
 *   To Make Indexing Applicable as Part of the Code we Used the first, second, last and temp Integer Variables which Hold the Current Index
 *   in the Four Line Sector Buffer where each Row is Stored.
 *
 *   In order to Calculate the Next Indexing for each Row we Used the Formula Below:
 *   Index = (Index + 3) % 4
 */
int sobel_filter(/*01*/AXI_PIXEL STREAM_IN[MAX_WIDTH],
                 /*02*/AXI_PIXEL STREAM_OUT[MAX_WIDTH],
                 /*03*/int rows,
                 /*04*/int cols,
                 /*05*/int packet_mode_en,
                 /*06*/int packet_size
                 )
{
	/*
	 * Set the Fifo of the STREAM_OUT and STREAM_IN Interfaces to be Implemented with LUT RAM Memory.
	 */
	#pragma HLS RESOURCE variable=STREAM_OUT core=FIFO_LUTRAM
	#pragma HLS RESOURCE variable=STREAM_IN  core=FIFO_LUTRAM

	/*
	 * The rows is a Register to Store the Number of Rows of the Image that will be Accelerated.
	 * This Register is Accessed through the AXI Slave Lite Interface (S_AXI4_LITE) of the Core.
	 */
	#pragma HLS INTERFACE  s_axilite  port=rows            bundle=S_AXI4_LITE

	/*
	 * The cols is a Register to Store the Number of Columns of the Image that will be Accelerated.
	 * This Register is Accessed through the AXI Slave Lite Interface (S_AXI4_LITE) of the Core.
	 */
	#pragma HLS INTERFACE  s_axilite  port=cols            bundle=S_AXI4_LITE

	/*
	 * The packet_mode_en is a Register to Store a Value that Enables/Disables the Packet Mode.
	 * The Packet Mode Should be Enabled when the Data are Transferred with Scatter/Gather Transactions.
	 * When the Packet Mode is Enabled the Core Sends a TLAST=1 Signal in the Output Interface for each Transmitted Packet.
	 * This Register is Accessed through the AXI Slave Lite Interface (S_AXI4_LITE) of the Core.
	 */
	#pragma HLS INTERFACE  s_axilite  port=packet_mode_en  bundle=S_AXI4_LITE

	/*
	 * The packet_size is a Register to Store the Size that each Packet Should Have (e.g 4K) when Using Scatter/Gather Transfers.
	 * This Register is Accessed through the AXI Slave Lite Interface (S_AXI4_LITE) of the Core.
	 */
	#pragma HLS INTERFACE  s_axilite  port=packet_size     bundle=S_AXI4_LITE
	#pragma HLS INTERFACE  s_axilite  port=return          bundle=S_AXI4_LITE

	/*
	 * Set the STREAM_OUT and STREAM_IN Interfaces of the Core to be AXI Stream Interfaces.
	 * The Fifo Depth is Set to 1920 which is the Maximum Image Width that the Core Can Support to Process.
	 */
	#pragma HLS INTERFACE axis depth=1920 port=STREAM_IN
	#pragma HLS INTERFACE axis depth=1920 port=STREAM_OUT

	int bytes_count; //Count the Number of Tranferred Bytes.
	int first; //Used to Know where the First Received Row is Located in the LINE4_SECTOR_BUFFER.
	int second; //Used to Know where the Second Received Row is Located in the LINE4_SECTOR_BUFFER.
	int last; //Used to Know where the Last Received Row is Located in the LINE4_SECTOR_BUFFER.
	int temp; //Used to Know where the Newest Received Row Should be Temporalily Located in the LINE4_SECTOR_BUFFER.

	/*
	 * The Number of Iterations Required to Receive or Send each Sector of a Row.
	 * The sector_iter is an Array with as many Fields as the Number of Sectors.
	 * The sector_iter Array is Configured to be Completely Partitioned according to the #pragma HLS ARRAY_PARTITION.
	 */
	int  sector_iter[SECTORS];
	#pragma HLS ARRAY_PARTITION variable=sector_iter dim=1 complete

	int  sector_size; //The Number of Columns that each Sector Should Store.
	int  remaining_pixels; //If the Number of Columns is not an Integer Multiple of the Number of Sectors then we Have Remaining Pixels that Should be Distributed in all the Sectors.

	const RGB zero_pixel = {0, 0, 0}; //This is a Dark Pixel Used to Set the First and Last Row and all the First and Last Columns of the Image.

	/*
	 * Declare 16 Memory Buffers of Type LINE4_SECTOR_BUFFER.
	 * Each Buffer is Set to be Dual Port BRAM according to the #pragma HLS RESOURCE.
	 *
	 * These Buffers are Used to Receive the Image Rows before being Processed.
	 */
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR0;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR0 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR1;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR1 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR2;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR2 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR3;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR3 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR4;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR4 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR5;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR5 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR6;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR6 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR7;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR7 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR8;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR8 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR9;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR9 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR10;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR10 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR11;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR11 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR12;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR12 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR13;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR13 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR14;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR14 core=RAM_2P_BRAM
	LINE4_SECTOR_BUFFER LINE4_BUFFER_SECTOR15;
	#pragma HLS RESOURCE variable=LINE4_BUFFER_SECTOR15 core=RAM_2P_BRAM

	/*
	 * Declare 16 Memory Buffers of Type LINE1_SECTOR_BUFFER.
	 * Each Buffer is Set to be Dual Port BRAM according to the #pragma HLS RESOURCE.
	 *
	 * These Buffers are Used to Store the Image Rows after being Processed.
	 */
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR0;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR0 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR1;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR1 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR2;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR2 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR3;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR3 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR4;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR4 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR5;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR5 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR6;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR6 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR7;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR7 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR8;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR8 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR9;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR9 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR10;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR10 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR11;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR11 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR12;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR12 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR13;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR13 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR14;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR14 core=RAM_2P_BRAM
	LINE1_SECTOR_BUFFER LINE1_BUFFER_SECTOR15;
	#pragma HLS RESOURCE variable=LINE1_BUFFER_SECTOR15 core=RAM_2P_BRAM

	/*
	 * Set Initial Values.
	 */
	bytes_count = 0;
	first = 0;
	second = 3;
	last = 2;
	temp = 0;


	//Calculate the Number of Columns that Should be Stored to each Sector Buffer.
	//NOTE that this is the Initial Sector Size that is Equal to All the Sector Buffers.
	sector_size = (int)(cols / SECTORS);

	/*
	 * Calculate any Remaining Bytes in Case the Number of Columns is not an Integer Multiple of the Number of Sector Buffers.
	 *
	 * For Example, for an Image of Width 524 Pixels we Have 524 Pixels / 16 Sectors = 32.75 Pixels which is Not an Integer Multiple of the 16 Sector Buffers.
	 * For each Sector Buffer we Have a Sector Size of 32 Pixels so 32 Pixels * 16 Sectors = 512 which Leads to Have 12 Remaining Pixels from the Initial 524.
	 *
	 * As a Result each of the 16 Sector Buffers Initially Has a Sector Size of 32.
	 * The Remaining Pixels Should be Distributed to the Sector Buffers so the First 12 Sector Buffers will Have a Sector Size with one More Pixel which Leads to 33 Pixels Sector Size.
	 *
	 * |Sector0  |Sector1  |Sector2  |Sector3  |Sector4  |Sector5  |Sector6  |Sector7  |Sector8  |Sector9  |Sector10  |Sector11  |Sector12  |Sector13  |Sector14  |Sector15|
	 * |33       |33       |33       |33       |33       |33       |33       |33       |33       |33       |33        |33        |32        |32        |32        |32      |
	 *
	 */
	remaining_pixels = cols - (sector_size * SECTORS);

	//Loop to Distribute the Remaining Bytes to the Sector Buffers.
	for (int i = 0; i < SECTORS; i++)
	{
	#pragma HLS PIPELINE II=1

		//Set the Array Field of the Corresponding Sector Buffer with the Initial Sector Size.
		sector_iter[i] = sector_size;

		//Check if we still Have Remaining Pixels
		if (remaining_pixels > 0)
		{
			//Decrease the Number of Remaining Pixels.
			remaining_pixels--;

			//Increment by 1 the Sector Size of the Corresponding Sector Buffer
			sector_iter[i] ++;
		}
	}

	/*
	 * The First Line/Row of an Image Processed with Sobel Edge Detection is Always Filled with Zero Pixels.
	 * So, Send the First Row of Zero Pixels.
	 */
	send_1st_line:
	for (int col=0; col<cols; col++)
	{
		#pragma HLS PIPELINE II=1

		AXI_PIXEL output_pixel; //Declare a AXI_PIXEL that Represents the AXI Stream Output Interface.

		output_pixel.strb = 0xF; //Set the Strobe of the AXI Stream Interface so that all 4 Transmitted Bytes are Valid.
		output_pixel.user = 0x1;
		output_pixel.tdest = 0x1;

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_en == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packket Size.
			output_pixel.last = is_packet_complete(&bytes_count, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the Data to Transmit to Have Zero Value Since we Transmit Zero Pixels.
		output_pixel.data = 0x0;

		//Forward the Data along with the Rest Signals to the AXI Stream Output Interface.
		STREAM_OUT[col] = output_pixel;
	}

	/*
	 * The Sobel Edge Detection Algorithm Requires Three Rows in Order to Produce one Processed Row.
	 * So, Pre-Fetch the First 3 Rows.
	 */
	prefetch_3lines:
	for (int row=0; row<3; row++)
	{
		//Receive a Row which is Distributed to the 16 Four Line Sector Buffers of Type LINE4_SECTOR_BUFFER.
		receive_post_line(STREAM_IN,
                          &LINE4_BUFFER_SECTOR0,
                          &LINE4_BUFFER_SECTOR1,
                          &LINE4_BUFFER_SECTOR2,
                          &LINE4_BUFFER_SECTOR3,
                          &LINE4_BUFFER_SECTOR4,
                          &LINE4_BUFFER_SECTOR5,
                          &LINE4_BUFFER_SECTOR6,
                          &LINE4_BUFFER_SECTOR7,
                          &LINE4_BUFFER_SECTOR8,
                          &LINE4_BUFFER_SECTOR9,
                          &LINE4_BUFFER_SECTOR10,
                          &LINE4_BUFFER_SECTOR11,
                          &LINE4_BUFFER_SECTOR12,
                          &LINE4_BUFFER_SECTOR13,
                          &LINE4_BUFFER_SECTOR14,
                          &LINE4_BUFFER_SECTOR15,
                          temp,
                          sector_iter
                          );

		//Calculate the Vertical Position where the Next Received Row Should be Stored in the Four Line Sector Buffers.
		temp = (temp+3)%4;
	}

	/*
	 * Produce a Processed Row from the 3 Pre-Fetched Rows.
	 * Send the Processed Row and then Receive a New Row in Order to Produce again a Processed Row.
	 *
	 * Loop Until Receiving all the Rows of the Image.
	 */
	proc_module:
	for (int row=0; row<rows-3; row++)
	{
    #pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR0  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR1  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR2  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR3  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR4  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR5  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR6  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR7  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR8  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR9  array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR10 array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR11 array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR12 array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR13 array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR14 array //false
	#pragma HLS DEPENDENCE variable=LINE4_BUFFER_SECTOR15 array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR0  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR1  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR2  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR3  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR4  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR5  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR6  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR7  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR8  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR9  array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR10 array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR11 array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR12 array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR13 array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR14 array //false
	#pragma HLS DEPENDENCE variable=LINE1_BUFFER_SECTOR15 array //false

	//Process the 3 Rows that are Received in the 16 Four Line Sector Buffers and Produce one Processed Row.
	start_sobel_operations(&LINE4_BUFFER_SECTOR0,
                           &LINE4_BUFFER_SECTOR1,
                           &LINE4_BUFFER_SECTOR2,
                           &LINE4_BUFFER_SECTOR3,
                           &LINE4_BUFFER_SECTOR4,
                           &LINE4_BUFFER_SECTOR5,
                           &LINE4_BUFFER_SECTOR6,
                           &LINE4_BUFFER_SECTOR7,
                           &LINE4_BUFFER_SECTOR8,
                           &LINE4_BUFFER_SECTOR9,
                           &LINE4_BUFFER_SECTOR10,
                           &LINE4_BUFFER_SECTOR11,
                           &LINE4_BUFFER_SECTOR12,
                           &LINE4_BUFFER_SECTOR13,
                           &LINE4_BUFFER_SECTOR14,
                           &LINE4_BUFFER_SECTOR15,
                           &LINE1_BUFFER_SECTOR0,
                           &LINE1_BUFFER_SECTOR1,
                           &LINE1_BUFFER_SECTOR2,
                           &LINE1_BUFFER_SECTOR3,
                           &LINE1_BUFFER_SECTOR4,
                           &LINE1_BUFFER_SECTOR5,
                           &LINE1_BUFFER_SECTOR6,
                           &LINE1_BUFFER_SECTOR7,
                           &LINE1_BUFFER_SECTOR8,
                           &LINE1_BUFFER_SECTOR9,
                           &LINE1_BUFFER_SECTOR10,
                           &LINE1_BUFFER_SECTOR11,
                           &LINE1_BUFFER_SECTOR12,
                           &LINE1_BUFFER_SECTOR13,
                           &LINE1_BUFFER_SECTOR14,
                           &LINE1_BUFFER_SECTOR15,
                           sector_size,
                           first,
                           second,
                           last
						   );

		//Set the First Pixel of the First One Line Sector Buffer to be a Dark/Zero Pixel.
	    //NOTE the First Pixel of the First One Line Sector Buffer is Actually the First Pixel of a Row.
		LINE1_BUFFER_SECTOR0.insert(zero_pixel, 0, 0);

		//Set the Last Pixel of the Last One Line Sector Buffer to be a Dark/Zero Pixel.
	    //NOTE the Last Pixel of the Last One Line Sector Buffer is Actually the Last Pixel of a Row.
		LINE1_BUFFER_SECTOR15.insert(zero_pixel, 0, sector_iter[15]-1);

		//The Produced Row from the three Processed Rows is Stored in the 16 One Line Sector Buffers.
		//Send the Produced Row over the AXI Stream Out Interface.
		send_line(STREAM_OUT,
                  &LINE1_BUFFER_SECTOR0,
                  &LINE1_BUFFER_SECTOR1,
                  &LINE1_BUFFER_SECTOR2,
                  &LINE1_BUFFER_SECTOR3,
                  &LINE1_BUFFER_SECTOR4,
                  &LINE1_BUFFER_SECTOR5,
                  &LINE1_BUFFER_SECTOR6,
                  &LINE1_BUFFER_SECTOR7,
                  &LINE1_BUFFER_SECTOR8,
                  &LINE1_BUFFER_SECTOR9,
                  &LINE1_BUFFER_SECTOR10,
                  &LINE1_BUFFER_SECTOR11,
                  &LINE1_BUFFER_SECTOR12,
                  &LINE1_BUFFER_SECTOR13,
                  &LINE1_BUFFER_SECTOR14,
                  &LINE1_BUFFER_SECTOR15,
                  sector_iter,
                  packet_mode_en,
                  packet_size,
                  &bytes_count
				  );

		//Receive the Next Row which is Distributed to the 16 Four Line Sector Buffers.
		receive_post_line(STREAM_IN,
                          &LINE4_BUFFER_SECTOR0,
                          &LINE4_BUFFER_SECTOR1,
                          &LINE4_BUFFER_SECTOR2,
                          &LINE4_BUFFER_SECTOR3,
                          &LINE4_BUFFER_SECTOR4,
                          &LINE4_BUFFER_SECTOR5,
                          &LINE4_BUFFER_SECTOR6,
                          &LINE4_BUFFER_SECTOR7,
                          &LINE4_BUFFER_SECTOR8,
                          &LINE4_BUFFER_SECTOR9,
                          &LINE4_BUFFER_SECTOR10,
                          &LINE4_BUFFER_SECTOR11,
                          &LINE4_BUFFER_SECTOR12,
                          &LINE4_BUFFER_SECTOR13,
                          &LINE4_BUFFER_SECTOR14,
                          &LINE4_BUFFER_SECTOR15,
                          temp,
                          sector_iter
                          );

		//Calculate which Should be Considered as the First Line for Producing the Next Processed Row.
		first = (first+3)%4;

		//Calculate which Should be Considered as the Second Line for Producing the Next Processed Row.
		second = (second+3)%4;

		//Calculate which Should be Considered as the Last Line for Producing the Next Processed Row.
		last = (last+3)%4;

		//Calculate where the New Received Row Should be Stored in the Four Line Sector Buffers.
		temp = (temp+3)%4;
	}

	//The Previous for Loop Ended before Processing the Last Received Row.
	//So, Process here the Last 3 Rows to Produce the Last Processed Row.
	start_sobel_operations(&LINE4_BUFFER_SECTOR0,
                           &LINE4_BUFFER_SECTOR1,
                           &LINE4_BUFFER_SECTOR2,
                           &LINE4_BUFFER_SECTOR3,
                           &LINE4_BUFFER_SECTOR4,
                           &LINE4_BUFFER_SECTOR5,
                           &LINE4_BUFFER_SECTOR6,
                           &LINE4_BUFFER_SECTOR7,
                           &LINE4_BUFFER_SECTOR8,
                           &LINE4_BUFFER_SECTOR9,
                           &LINE4_BUFFER_SECTOR10,
                           &LINE4_BUFFER_SECTOR11,
                           &LINE4_BUFFER_SECTOR12,
                           &LINE4_BUFFER_SECTOR13,
                           &LINE4_BUFFER_SECTOR14,
                           &LINE4_BUFFER_SECTOR15,
                           &LINE1_BUFFER_SECTOR0,
                           &LINE1_BUFFER_SECTOR1,
                           &LINE1_BUFFER_SECTOR2,
                           &LINE1_BUFFER_SECTOR3,
                           &LINE1_BUFFER_SECTOR4,
                           &LINE1_BUFFER_SECTOR5,
                           &LINE1_BUFFER_SECTOR6,
                           &LINE1_BUFFER_SECTOR7,
                           &LINE1_BUFFER_SECTOR8,
                           &LINE1_BUFFER_SECTOR9,
                           &LINE1_BUFFER_SECTOR10,
                           &LINE1_BUFFER_SECTOR11,
                           &LINE1_BUFFER_SECTOR12,
                           &LINE1_BUFFER_SECTOR13,
                           &LINE1_BUFFER_SECTOR14,
                           &LINE1_BUFFER_SECTOR15,
                           sector_size,
                           first,
                           second,
                           last);

	//Set the First Pixel of the First One Line Sector Buffer to be a Dark/Zero Pixel.
    //NOTE the First Pixel of the First One Line Sector Buffer is Actually the First Pixel of a Row.
	LINE1_BUFFER_SECTOR0.insert(zero_pixel, 0, 0);

	//Set the Last Pixel of the Last One Line Sector Buffer to be a Dark/Zero Pixel.
    //NOTE the Last Pixel of the Last One Line Sector Buffer is Actually the Last Pixel of a Row.
	LINE1_BUFFER_SECTOR15.insert(zero_pixel, 0, sector_iter[15]-1);

	//Send the Last Produced Row over the AXI Stream Out Interface.
	send_line(STREAM_OUT,
              &LINE1_BUFFER_SECTOR0,
              &LINE1_BUFFER_SECTOR1,
              &LINE1_BUFFER_SECTOR2,
              &LINE1_BUFFER_SECTOR3,
              &LINE1_BUFFER_SECTOR4,
              &LINE1_BUFFER_SECTOR5,
              &LINE1_BUFFER_SECTOR6,
              &LINE1_BUFFER_SECTOR7,
              &LINE1_BUFFER_SECTOR8,
              &LINE1_BUFFER_SECTOR9,
              &LINE1_BUFFER_SECTOR10,
              &LINE1_BUFFER_SECTOR11,
              &LINE1_BUFFER_SECTOR12,
              &LINE1_BUFFER_SECTOR13,
              &LINE1_BUFFER_SECTOR14,
              &LINE1_BUFFER_SECTOR15,
              sector_iter,
              packet_mode_en,
              packet_size,
              &bytes_count
			  );

	/*
	 * The Last Line/Row of an Image Processed with Sobel Edge Detection is Always Filled with Zero Pixels.
	 * So, Send the Last Row of Zero Pixels.
	 */
	send_last_line:
	for (int col=0; col<cols; col++)
	{
		#pragma HLS PIPELINE II=1

		AXI_PIXEL output_pixel; //Declare a AXI_PIXEL that Represents the AXI Stream Output Interface.

		output_pixel.strb = 0xF; //Set the Strobe of the AXI Stream Interface so that all 4 Transmitted Bytes are Valid.
		output_pixel.user = 0x1;
		output_pixel.tdest = 0x1;

		//Since this is the Last Row Check if this is the Last Pixel to Send.
		if (col==cols-1 )
		{
			//Set the TLAST Signal to 1 to Indicate that this will be the Last Transmission of the Data.
			output_pixel.last = 0x1;
		}
		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		else if(packet_mode_en == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(&bytes_count, packet_size);
		}
		else
			output_pixel.last = 0x0;

		//Set the Data to Transmit to Have Zero Value Since we Transmit Zero Pixels.
		output_pixel.data = 0x0;

		//Forward the Data along with the Rest Signals to the AXI Stream Output Interface.
		STREAM_OUT[col] = output_pixel;
	}

	bytes_count = 0; // Byte Counter

	return 1;
}

