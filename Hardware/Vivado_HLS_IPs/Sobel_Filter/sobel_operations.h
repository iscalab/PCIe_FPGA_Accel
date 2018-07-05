/*******************************************************************************
* Filename:   sobel_operations.h
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

#ifndef _SOBEL_OPERATIONS_H_
#define _SOBEL_OPERATIONS_H_

unsigned char rgb2y(RGB pix);

/*
 * Template of the sobel_operator()
 *
 * The sobel_operator() Makes Sobel Computation Using a 3x3 Neighborhood
 */
template<int dummy_index>
RGB sobel_operator(LINE4_SECTOR_BUFFER *window,
		           unsigned int x_index,
		           unsigned int y_first,
		           unsigned int y_second,
		           unsigned int y_last)
{
	#pragma HLS INLINE off
	#pragma HLS EXPRESSION_BALANCE off

	short x_weight = 0;
	short y_weight = 0;

	short x_weight_array[9];
	#pragma HLS ARRAY_PARTITION variable=x_weight_array complete dim=1
	short y_weight_array[9];
	#pragma HLS ARRAY_PARTITION variable=y_weight_array complete dim=1

	short edge_weight;
	unsigned char edge_val;
	RGB pixel;

	const char x_op[3][3] = { {-1, 0, 1},
                              {-2, 0, 2},
                              {-1, 0, 1}};
	#pragma HLS ARRAY_PARTITION variable=x_op complete dim=1

	const char y_op[3][3] = { { 1, 2, 1},
                              { 0, 0, 0},
                              {-1,-2,-1}};
	#pragma HLS ARRAY_PARTITION variable=y_op complete dim=1

	sobel_mul:
	{
	#pragma HLS PIPELINE II=1

	//Compute Approximation of the Gradients in the X-Y Direction for the First Row of x_op and y_op.
	for(char j = 0; j < 3; j++)
	{
	#pragma HLS UNROLL
	#pragma HLS PIPELINE II=1

		// X Direction Gradient
		x_weight_array[j] = (window->getval(y_first,x_index + j) * x_op[0][j]);
		// Y Direction Gradient
		y_weight_array[j] = (window->getval(y_first,x_index + j) * y_op[0][j]);
	}

	//Compute Approximation of the Gradients in the X-Y Direction for the Second Row of x_op and y_op.
	for(char j = 0; j < 3; j++)
	{
	#pragma HLS UNROLL
	#pragma HLS PIPELINE II=1

		// X Direction Gradient
		x_weight_array[3+j] = (window->getval(y_second,x_index + j) * x_op[1][j]);
		// Y Direction Gradient
		y_weight_array[3+j] = (window->getval(y_second,x_index + j) * y_op[1][j]);
	}

	//Compute Approximation of the Gradients in the X-Y Direction for the Third Row of x_op and y_op.
	for(char j = 0; j < 3; j++){
	#pragma HLS UNROLL
	#pragma HLS PIPELINE II=1

		// X Direction Gradient
		x_weight_array[6+j] = (window->getval(y_last,x_index + j) * x_op[2][j]);
		// Y Direction Gradient
		y_weight_array[6+j] = (window->getval(y_last,x_index + j) * y_op[2][j]);
	}

	}

	for(char j = 0; j < 9; j++) {
	#pragma HLS UNROLL
	#pragma HLS PIPELINE II=1

		// X Direction Gradient
		x_weight += x_weight_array[j];
		// Y Direction Gradient
		y_weight += y_weight_array[j];
	}

  edge_weight = ABS(x_weight) + ABS(y_weight);

  edge_val = (255-(unsigned char)(edge_weight));

  //Edge Thresholding
  if(edge_val > 200)
  {
    edge_val = 255;
  }
  else if(edge_val < 100)
  {
    edge_val = 0;
  }

  pixel.R = pixel.G = pixel.B = edge_val;

  return pixel;
}

void start_sobel_operations(
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
		int last);

void send_line(
		AXI_PIXEL *STREAM_OUT,
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
		int *remain_bytes);

void receive_post_line(
		AXI_PIXEL *STREAM_IN,
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
		int *sector_iter_array);

#endif
