/*******************************************************************************
* Filename:   sobel_operations.cpp
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
 * rgb2y()
 *
 * Makes RGB to Y Conversion.
 * It Returns a Luminance Value that is Used in Edge Detection.
 */
unsigned char rgb2y(RGB pix)
{
	#pragma HLS INLINE off
	#pragma HLS EXPRESSION_BALANCE off

	unsigned char y;

	//Luminance Calculation According to the R,G,B Values of the Pixel.
	y = ((66 * pix.R.to_int() + 129 * pix.G.to_int() + 25 * pix.B.to_int() + 128) >> 8) + 16;

	return y;
}

/*
 * start_sobel_operations()
 *
 * Produces a Single Processed Row by Applying Sobel Edge Detection to 3 Rows.
 * It Reads the 3 Rows from the 16 Four Line Sector Buffers and Stores the Produced Processed Row in the 16 One Line Sector Buffers.
 */
void start_sobel_operations(LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_0,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_1,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_2,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_3,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_4,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_5,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_6,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_7,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_8,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_9,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_10,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_11,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_12,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_13,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_14,
                            LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_15,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_0,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_1,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_2,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_3,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_4,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_5,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_6,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_7,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_8,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_9,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_10,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_11,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_12,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_13,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_14,
                            LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_15,
                            int sector_size,
                            int first,
                            int second,
                            int last)
{
	#pragma HLS PIPELINE II=1

	/*
	 * Loop for as many Times as the Number of Columns in each Sector Buffer in Order to Apply Sobel Edge Detection to all the Pixels.
	 */
	loop_sobel_operations:
	for (int col = 0; col <= sector_size; col++)
	{

		RGB edge0, edge1, edge2, edge3, edge4, edge5, edge6, edge7,
			   edge8, edge9, edge10, edge11, edge12, edge13, edge14, edge15;

		/*
		 * Create 16 Instances of the sobel_operator Function Template.
		 * Each Instance Applies Sobel Edge Detection to the Current Pixel of each Four Line Sector Buffer.
		 * All Instances are Executed Concurrently Since there is no Dependency between the 16 Four Line Sector Buffers.
		 */
		edge0 = sobel_operator<0>(LINE_BUFFER_SECTOR_0, col, first, second, last);
		edge1 = sobel_operator<1>(LINE_BUFFER_SECTOR_1, col, first, second, last);
		edge2 = sobel_operator<2>(LINE_BUFFER_SECTOR_2, col, first, second, last);
		edge3 = sobel_operator<3>(LINE_BUFFER_SECTOR_3, col, first, second, last);
		edge4 = sobel_operator<4>(LINE_BUFFER_SECTOR_4, col, first, second, last);
		edge5 = sobel_operator<5>(LINE_BUFFER_SECTOR_5, col, first, second, last);
		edge6 = sobel_operator<6>(LINE_BUFFER_SECTOR_6, col, first, second, last);
		edge7 = sobel_operator<7>(LINE_BUFFER_SECTOR_7, col, first, second, last);
		edge8 = sobel_operator<8>(LINE_BUFFER_SECTOR_8, col, first, second, last);
		edge9 = sobel_operator<9>(LINE_BUFFER_SECTOR_9, col, first, second, last);
		edge10 = sobel_operator<10>(LINE_BUFFER_SECTOR_10, col, first, second, last);
		edge11 = sobel_operator<11>(LINE_BUFFER_SECTOR_11, col, first, second, last);
		edge12 = sobel_operator<12>(LINE_BUFFER_SECTOR_12, col, first, second, last);
		edge13 = sobel_operator<13>(LINE_BUFFER_SECTOR_13, col, first, second, last);
		edge14 = sobel_operator<14>(LINE_BUFFER_SECTOR_14, col, first, second, last);
		edge15 = sobel_operator<15>(LINE_BUFFER_SECTOR_15, col, first, second, last);

		/*
		 * Insert the Processed Pixels by the Sobel Operator to the Correct Position in the One Line Sector Buffers.
		 */
		OUTPUT_BUFFER_SECTOR_0->insert(edge0, 0, col);
		OUTPUT_BUFFER_SECTOR_1->insert(edge1, 0, col);
		OUTPUT_BUFFER_SECTOR_2->insert(edge2, 0, col);
		OUTPUT_BUFFER_SECTOR_3->insert(edge3, 0, col);
		OUTPUT_BUFFER_SECTOR_4->insert(edge4, 0, col);
		OUTPUT_BUFFER_SECTOR_5->insert(edge5, 0, col);
		OUTPUT_BUFFER_SECTOR_6->insert(edge6, 0, col);
		OUTPUT_BUFFER_SECTOR_7->insert(edge7, 0, col);
		OUTPUT_BUFFER_SECTOR_8->insert(edge8, 0, col);
		OUTPUT_BUFFER_SECTOR_9->insert(edge9, 0, col);
		OUTPUT_BUFFER_SECTOR_10->insert(edge10, 0, col);
		OUTPUT_BUFFER_SECTOR_11->insert(edge11, 0, col);
		OUTPUT_BUFFER_SECTOR_12->insert(edge12, 0, col);
		OUTPUT_BUFFER_SECTOR_13->insert(edge13, 0, col);
		OUTPUT_BUFFER_SECTOR_14->insert(edge14, 0, col);
		OUTPUT_BUFFER_SECTOR_15->insert(edge15, 0, col);
	}
}

/*
 * receive_post_line()
 *
 * Receives an Image Row through the AXI Stream In Interface.
 * The receive_post_line() Takes Over to Equally Split and Distribute the Row (while being Received) to the 16 Four Line Sector Buffers.
 */
void receive_post_line(AXI_PIXEL *STREAM_IN,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_0,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_1,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_2,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_3,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_4,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_5,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_6,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_7,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_8,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_9,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_10,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_11,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_12,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_13,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_14,
                       LINE4_SECTOR_BUFFER *LINE_BUFFER_SECTOR_15,
                       int row,
                       int *sector_iter_array)
{

		/* Receive the Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_0.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_0 Starting from the Second Field.
		 */
		loop_in_sector_0:
		for (int col = 0; col < sector_iter_array[0]; col++)
		{
		#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the First Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_0.
			LINE_BUFFER_SECTOR_0->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_0 to the First Field of the LINE_BUFFER_SECTOR_1.
		//The Last Pixel of the LINE_BUFFER_SECTOR_0 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_1 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_1->insert(LINE_BUFFER_SECTOR_0->getval(row, sector_iter_array[0]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_1.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_0 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_1 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_0.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_1 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_2.
		 */
		loop_in_sector_1:
		for (int col = 0; col < sector_iter_array[1]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_1.
			LINE_BUFFER_SECTOR_1->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_1 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_0.
		//The First Pixel of the LINE_BUFFER_SECTOR_1 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_0 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_0->insert(LINE_BUFFER_SECTOR_1->getval(row, 1), row, sector_iter_array[0] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_1 to the First Field of the LINE_BUFFER_SECTOR_2.
		//The Last Pixel of the LINE_BUFFER_SECTOR_1 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_2 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_2->insert(LINE_BUFFER_SECTOR_1->getval(row, sector_iter_array[1]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_2.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_2 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_2 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_1.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_2 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_3.
		 */
		loop_in_sector_2:
		for (int col = 0; col < sector_iter_array[2]; col++)
		{
		#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_2.
			LINE_BUFFER_SECTOR_2->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_2 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_1.
		//The First Pixel of the LINE_BUFFER_SECTOR_2 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_1 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_1->insert(LINE_BUFFER_SECTOR_2->getval(row, 1), row, sector_iter_array[1] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_2 to the First Field of the LINE_BUFFER_SECTOR_3.
		//The Last Pixel of the LINE_BUFFER_SECTOR_2 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_3 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_3->insert(LINE_BUFFER_SECTOR_2->getval(row, sector_iter_array[2]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_3.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_3 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_3 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_2.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_3 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_4.
		 */
		loop_in_sector_3:
		for (int col = 0; col < sector_iter_array[3]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_3.
			LINE_BUFFER_SECTOR_3->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_3 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_2.
		//The First Pixel of the LINE_BUFFER_SECTOR_3 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_2 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_2->insert(LINE_BUFFER_SECTOR_3->getval(row, 1), row, sector_iter_array[2] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_3 to the First Field of the LINE_BUFFER_SECTOR_4.
		//The Last Pixel of the LINE_BUFFER_SECTOR_3 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_4 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_4->insert(LINE_BUFFER_SECTOR_3->getval(row, sector_iter_array[3]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_4.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_4 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_4 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_3.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_4 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_5.
		 */
		loop_in_sector_4:
		for (int col = 0; col < sector_iter_array[4]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_4.
			LINE_BUFFER_SECTOR_4->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_4 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_3.
		//The First Pixel of the LINE_BUFFER_SECTOR_4 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_3 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_3->insert(LINE_BUFFER_SECTOR_4->getval(row, 1), row, sector_iter_array[3] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_4 to the First Field of the LINE_BUFFER_SECTOR_5.
		//The Last Pixel of the LINE_BUFFER_SECTOR_4 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_5 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_5->insert(LINE_BUFFER_SECTOR_4->getval(row, sector_iter_array[4]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_5.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_5 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_5 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_4.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_5 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_6.
		 */
		loop_in_sector_5:
		for (int col = 0; col < sector_iter_array[5]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_5.
			LINE_BUFFER_SECTOR_5->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_5 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_4.
		//The First Pixel of the LINE_BUFFER_SECTOR_5 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_4 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_4->insert(LINE_BUFFER_SECTOR_5->getval(row, 1), row, sector_iter_array[4] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_5 to the First Field of the LINE_BUFFER_SECTOR_6.
		//The Last Pixel of the LINE_BUFFER_SECTOR_5 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_6 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_6->insert(LINE_BUFFER_SECTOR_5->getval(row, sector_iter_array[5]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_6.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_6 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_6 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_5.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_6 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_7.
		 */
		loop_in_sector_6:
		for (int col = 0; col < sector_iter_array[6]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_6.
			LINE_BUFFER_SECTOR_6->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_6 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_5.
		//The First Pixel of the LINE_BUFFER_SECTOR_6 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_5 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_5->insert(LINE_BUFFER_SECTOR_6->getval(row, 1), row, sector_iter_array[5] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_6 to the First Field of the LINE_BUFFER_SECTOR_7.
		//The Last Pixel of the LINE_BUFFER_SECTOR_6 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_7 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_7->insert(LINE_BUFFER_SECTOR_6->getval(row, sector_iter_array[6]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_7.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_7 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_7 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_6.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_7 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_8.
		 */
		loop_in_sector_7:
		for (int col = 0; col < sector_iter_array[7]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_7.
			LINE_BUFFER_SECTOR_7->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_7 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_6.
		//The First Pixel of the LINE_BUFFER_SECTOR_7 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_6 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_6->insert(LINE_BUFFER_SECTOR_7->getval(row, 1), row, sector_iter_array[6] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_7 to the First Field of the LINE_BUFFER_SECTOR_8.
		//The Last Pixel of the LINE_BUFFER_SECTOR_7 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_8 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_8->insert(LINE_BUFFER_SECTOR_7->getval(row, sector_iter_array[7]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_8.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_8 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_8 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_7.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_8 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_9.
		 */
		loop_in_sector_8:
		for (int col = 0; col < sector_iter_array[8]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_8.
			LINE_BUFFER_SECTOR_8->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_8 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_7.
		//The First Pixel of the LINE_BUFFER_SECTOR_8 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_7 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_7->insert(LINE_BUFFER_SECTOR_8->getval(row, 1), row, sector_iter_array[7] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_8 to the First Field of the LINE_BUFFER_SECTOR_9.
		//The Last Pixel of the LINE_BUFFER_SECTOR_8 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_9 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_9->insert(LINE_BUFFER_SECTOR_8->getval(row, sector_iter_array[8]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_9.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_9 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_9 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_8.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_9 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_10.
		 */
		loop_in_sector_9:
		for (int col = 0; col < sector_iter_array[9]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_9.
			LINE_BUFFER_SECTOR_9->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_9 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_8.
		//The First Pixel of the LINE_BUFFER_SECTOR_9 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_8 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_8->insert(LINE_BUFFER_SECTOR_9->getval(row, 1), row, sector_iter_array[8] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_9 to the First Field of the LINE_BUFFER_SECTOR_10.
		//The Last Pixel of the LINE_BUFFER_SECTOR_9 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_10 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_10->insert(LINE_BUFFER_SECTOR_9->getval(row, sector_iter_array[9]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_10.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_10 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_10 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_9.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_10 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_11.
		 */
		loop_in_sector_10:
		for (int col = 0; col < sector_iter_array[10]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_10.
			LINE_BUFFER_SECTOR_10->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_10 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_9.
		//The First Pixel of the LINE_BUFFER_SECTOR_10 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_9 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_9->insert(LINE_BUFFER_SECTOR_10->getval(row, 1), row, sector_iter_array[9] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_10 to the First Field of the LINE_BUFFER_SECTOR_11.
		//The Last Pixel of the LINE_BUFFER_SECTOR_10 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_11 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_11->insert(LINE_BUFFER_SECTOR_10->getval(row, sector_iter_array[10]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_11.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_11 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_11 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_10.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_11 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_12.
		 */
		loop_in_sector_11:
		for (int col = 0; col < sector_iter_array[11]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_11.
			LINE_BUFFER_SECTOR_11->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_11 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_10.
		//The First Pixel of the LINE_BUFFER_SECTOR_11 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_10 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_10->insert(LINE_BUFFER_SECTOR_11->getval(row, 1), row, sector_iter_array[10] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_11 to the First Field of the LINE_BUFFER_SECTOR_12.
		//The Last Pixel of the LINE_BUFFER_SECTOR_11 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_12 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_12->insert(LINE_BUFFER_SECTOR_11->getval(row, sector_iter_array[11]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_12.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_12 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_12 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_11.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_12 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_13.
		 */
		loop_in_sector_12:
		for (int col = 0; col < sector_iter_array[12]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_12.
			LINE_BUFFER_SECTOR_12->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_12 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_11.
		//The First Pixel of the LINE_BUFFER_SECTOR_12 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_11 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_11->insert(LINE_BUFFER_SECTOR_12->getval(row, 1), row, sector_iter_array[11] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_12 to the First Field of the LINE_BUFFER_SECTOR_13.
		//The Last Pixel of the LINE_BUFFER_SECTOR_12 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_13 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_13->insert(LINE_BUFFER_SECTOR_12->getval(row, sector_iter_array[12]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_13.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_13 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_13 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_12.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_13 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_14.
		 */
		loop_in_sector_13:
		for (int col = 0; col < sector_iter_array[13]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_13.
			LINE_BUFFER_SECTOR_13->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_13 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_12.
		//The First Pixel of the LINE_BUFFER_SECTOR_13 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_12 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_12->insert(LINE_BUFFER_SECTOR_13->getval(row, 1), row, sector_iter_array[12] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_13 to the First Field of the LINE_BUFFER_SECTOR_14.
		//The Last Pixel of the LINE_BUFFER_SECTOR_13 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_14 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_14->insert(LINE_BUFFER_SECTOR_13->getval(row, sector_iter_array[13]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_14.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_14 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_14 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_13.
		 * The Field Right After the Last Pixel of LINE_BUFFER_SECTOR_14 is Used to Insert the First Pixel of the LINE_BUFFER_SECTOR_15.
		 */
		loop_in_sector_14:
		for (int col = 0; col < sector_iter_array[14]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_14.
			LINE_BUFFER_SECTOR_14->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_14 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_13.
		//The First Pixel of the LINE_BUFFER_SECTOR_14 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_13 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_13->insert(LINE_BUFFER_SECTOR_14->getval(row, 1), row, sector_iter_array[13] + 1);

		//Insert the Y Value of the Last Pixel of the LINE_BUFFER_SECTOR_14 to the First Field of the LINE_BUFFER_SECTOR_15.
		//The Last Pixel of the LINE_BUFFER_SECTOR_14 is the Left Neighbor of the First Pixel of the LINE_BUFFER_SECTOR_15 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_15->insert(LINE_BUFFER_SECTOR_14->getval(row, sector_iter_array[14]), row, 0);


		/* Receive the Next Amount of Pixels of the Current Row that Should be Stored in the LINE_BUFFER_SECTOR_15.
		 *
		 * NOTE that this Loop Fills the Fields of the LINE_BUFFER_SECTOR_15 Starting from the Second Field.
		 * The First Field of the LINE_BUFFER_SECTOR_15 is Used to Insert the Last Pixel of the LINE_BUFFER_SECTOR_14.
		 */
		loop_in_sector_15:
		for (int col = 0; col < sector_iter_array[15]; col++)
		{
			#pragma HLS PIPELINE II=1

			unsigned char tempx;
			AXI_PIXEL input_pixel;
			RGB new_pix;

			//Receive through the AXI Stream In Interface 4 Bytes that Represent the Current Pixel.
			input_pixel = STREAM_IN[col];

			//Get the First Byte from the Received Data that Represents the Blue Value of the Pixel.
			new_pix.B = input_pixel.data.range(7, 0);

			//Get the Second Byte from the Received Data that Represents the Green Value of the Pixel.
			new_pix.G = input_pixel.data.range(15, 8);

			//Get the Third Byte from the Received Data that Represents the Red Value of the Pixel.
			new_pix.R = input_pixel.data.range(23, 16);

			//Convert the Received Pixel from RGB to Y that Represents its Luminance Value.
			tempx = rgb2y(new_pix);

			//Insert the Y Value of the Received Pixel to the Current Field of the LINE_BUFFER_SECTOR_15.
			LINE_BUFFER_SECTOR_15->insert(tempx, row, col+1);
		}

		//Insert the Y Value of the First Pixel of the LINE_BUFFER_SECTOR_15 to the Field Right After the Last Pixel of the LINE_BUFFER_SECTOR_14.
		//The First Pixel of the LINE_BUFFER_SECTOR_15 is the Right Neighbor of the Last Pixel of the LINE_BUFFER_SECTOR_14 which is Required to Apply Sobel Edge Detection.
		LINE_BUFFER_SECTOR_14->insert(LINE_BUFFER_SECTOR_15->getval(row, 1), row, sector_iter_array[14] + 1);

	}

/*
 * send_line()
 *
 * Send a Processed Image Row through the AXI Stream Out Interface.
 * The Processed Image is Distributed in the 16 One Line Sector Buffers so the send_line() will Send the Pixels of each Sector Buffer Sequencially.
 */
void send_line(AXI_PIXEL *STREAM_OUT,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_0,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_1,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_2,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_3,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_4,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_5,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_6,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_7,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_8,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_9,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_10,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_11,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_12,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_13,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_14,
               LINE1_SECTOR_BUFFER *OUTPUT_BUFFER_SECTOR_15,
               int *sector_iter_array,
               int packet_mode_enable,
               int packet_size,
               int *remain_bytes)
{

	int index = 0;
	AXI_PIXEL output_pixel;

	output_pixel.strb = 0xF; //Set the Strobe of the AXI Stream Interface so that all 4 Transmitted Bytes are Valid.
	output_pixel.user = 0x1;
	output_pixel.tdest = 0x1;

	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_0 (One Line Sector Buffer).
	 */
	loop_out_sector_0:
	for (int col = 0; col < sector_iter_array[0]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_0.
		convert = OUTPUT_BUFFER_SECTOR_0->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_1 (One Line Sector Buffer).
	 */
	loop_out_sector_1:
	for (int col = 0; col < sector_iter_array[1]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_1.
		convert = OUTPUT_BUFFER_SECTOR_1->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_2 (One Line Sector Buffer).
	 */
	loop_out_sector_2:
	for (int col = 0; col < sector_iter_array[2]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_2.
		convert = OUTPUT_BUFFER_SECTOR_2->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_3 (One Line Sector Buffer).
	 */
	loop_out_sector_3:
	for (int col = 0; col < sector_iter_array[3]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_3.
		convert = OUTPUT_BUFFER_SECTOR_3->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_4 (One Line Sector Buffer).
	 */
	loop_out_sector_4:
	for (int col = 0; col < sector_iter_array[4]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_4.
		convert = OUTPUT_BUFFER_SECTOR_4->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_5 (One Line Sector Buffer).
	 */
	loop_out_sector_5:
	for (int col = 0; col < sector_iter_array[5]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_5.
		convert = OUTPUT_BUFFER_SECTOR_5->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_6 (One Line Sector Buffer).
	 */
	loop_out_sector_6:
	for (int col = 0; col < sector_iter_array[6]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_6.
		convert = OUTPUT_BUFFER_SECTOR_6->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_7 (One Line Sector Buffer).
	 */
	loop_out_sector_7:
	for (int col = 0; col < sector_iter_array[7]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_7.
		convert = OUTPUT_BUFFER_SECTOR_7->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}
	

	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_8 (One Line Sector Buffer).
	 */
	loop_out_sector_8:
	for (int col = 0; col < sector_iter_array[8]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_8.
		convert = OUTPUT_BUFFER_SECTOR_8->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_9 (One Line Sector Buffer).
	 */
	loop_out_sector_9:
	for (int col = 0; col < sector_iter_array[9]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_9.
		convert = OUTPUT_BUFFER_SECTOR_9->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_10 (One Line Sector Buffer).
	 */
	loop_out_sector_10:
	for (int col = 0; col < sector_iter_array[10]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_10.
		convert = OUTPUT_BUFFER_SECTOR_10->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_11 (One Line Sector Buffer).
	 */
	loop_out_sector_11:
	for (int col = 0; col < sector_iter_array[11]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_11.
		convert = OUTPUT_BUFFER_SECTOR_11->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_12 (One Line Sector Buffer).
	 */
	loop_out_sector_12:
	for (int col = 0; col < sector_iter_array[12]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_12.
		convert = OUTPUT_BUFFER_SECTOR_12->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_13 (One Line Sector Buffer).
	 */
	loop_out_sector_13:
	for (int col = 0; col < sector_iter_array[13]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_13.
		convert = OUTPUT_BUFFER_SECTOR_13->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_14 (One Line Sector Buffer).
	 */
	loop_out_sector_14:
	for (int col = 0; col < sector_iter_array[14]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_14.
		convert = OUTPUT_BUFFER_SECTOR_14->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}


	/*
	 * Send ALL the Pixels of the Processed Row that are Stored in the OUTPUT_BUFFER_SECTOR_15 (One Line Sector Buffer).
	 */
	loop_out_sector_15:
	for (int col = 0; col < sector_iter_array[15]; col++)
	{
	#pragma HLS PIPELINE II=1
		RGB convert;

		//Get the Processed Pixel from the Current Field of the OUTPUT_BUFFER_SECTOR_15.
		convert = OUTPUT_BUFFER_SECTOR_15->getval(0, col);

		//If the Packet Mode is Enabled then we Have to Set the TLAST to 1 if a Full Packet of Size packet_size is Transmitted.
		if (packet_mode_enable == 1)
		{
			//The TLAST(last) Gets the Return Value of the is_packet_complete() which Returns 1 if the Number of Transmitted Bytes is Equal to the Packet Size.
			output_pixel.last = is_packet_complete(remain_bytes, packet_size);
		}
		else
		{
			output_pixel.last = 0x0;
		}

		//Set the First Byte of the Output Data with the Blue Value of the Pixel.
		output_pixel.data.range(7, 0)   = convert.B;

		//Set the Second Byte of the Output Data with the Green Value of the Pixel.
		output_pixel.data.range(15, 8)  = convert.G;

		//Set the Third Byte of the Output Data with the Red Value of the Pixel.
		output_pixel.data.range(23, 16) = convert.R;

		//Send the Current Processed Pixel through the AXI Stream Out Interface.
		STREAM_OUT[index++] = output_pixel;
	}

}
