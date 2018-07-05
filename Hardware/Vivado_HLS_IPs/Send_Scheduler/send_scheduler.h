/*******************************************************************************
* Filename:   send_scheduler.h
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

#define APM_READ_TRANSACTIONS_OFFSET 0
#define APM_READ_BYTES_OFFSET 4

#define APM_WRITE_TRANSACTIONS_OFFSET 8
#define APM_WRITE_BYTES_OFFSET 12

#define APM_PACKETS_OFFSET 16
#define APM_BYTES_OFFSET 20

#define APM_GCC_L_OFFSET 24
#define APM_GCC_U_OFFSET 28

#define CDMA_FETCH_TIME_START_L_OFFSET 32
#define CDMA_FETCH_TIME_START_U_OFFSET 36

#define CDMA_FETCH_TIME_END_L_OFFSET 40
#define CDMA_FETCH_TIME_END_U_OFFSET 44

#define CDMA_SEND_TIME_START_L_OFFSET 48
#define CDMA_SEND_TIME_START_U_OFFSET 52

#define CDMA_SEND_TIME_END_L_OFFSET 56
#define CDMA_SEND_TIME_END_U_OFFSET 60

#define DMA_ACCEL_TIME_START_L_OFFSET 64
#define DMA_ACCEL_TIME_START_U_OFFSET 68

#define DMA_ACCEL_TIME_END_L_OFFSET 72
#define DMA_ACCEL_TIME_END_U_OFFSET 76


struct image_info
{
	ap_uint<32> rows;
	ap_uint<32> columns;
	ap_uint<64> size;
};

struct metrics
{
	/*
	 * AXI Performance Monitor Metrics
	 */
	ap_uint<32> apm_read_transactions; //Offset 0 Bytes
	ap_uint<32> apm_read_bytes; //Offset 4 Bytes

	ap_uint<32> apm_write_transactions; //Offset 8 Bytes
	ap_uint<32> apm_write_bytes; //Offset 12 Bytes

	ap_uint<32> apm_packets; //Offset 16 Bytes
	ap_uint<32> apm_bytes; //Offset 20 Bytes

	ap_uint<32> apm_gcc_l; //Offset 24 Bytes
	ap_uint<32> apm_gcc_u; //Offset 28 Bytes

	ap_uint<32> cdma_fetch_time_start_l; //Offset 32 Bytes
	ap_uint<32> cdma_fetch_time_start_u; //Offset 36 Bytes
	ap_uint<32> cdma_fetch_time_end_l; //Offset 40 Bytes
	ap_uint<32> cdma_fetch_time_end_u; //Offset 44 Bytes

	ap_uint<32> cdma_send_time_start_l; //Offset 48 Bytes
	ap_uint<32> cdma_send_time_start_u; //Offset 52 Bytes
	ap_uint<32> cdma_send_time_end_l; //Offset 56 Bytes
	ap_uint<32> cdma_send_time_end_u; //Offset 60 Bytes

	ap_uint<32> dma_accel_time_start_l; //Offset 64 Bytes
	ap_uint<32> dma_accel_time_start_u; //Offset 68 Bytes
	ap_uint<32> dma_accel_time_end_l; //Offset 72 Bytes
	ap_uint<32> dma_accel_time_end_u; //Offset 76 Bytes

	struct image_info shared_image_info; // Offset 80 Bytes

	/*
	 * Kernel and Userspace Metrics
	 */

	ap_uint<64> total_time_start;
	ap_uint<64> total_time_end;

	ap_uint<64> sleep_time_start;
	ap_uint<64> sleep_time_end;

	ap_uint<64> preparation_time_start;
	ap_uint<64> preparation_time_end;

	ap_uint<64> load_time_start;
	ap_uint<64> load_time_end;

	ap_uint<64> save_time_start;
	ap_uint<64> save_time_end;


};

struct status_flags
{
	ap_uint<32> accel_direct_0_occupied_pid;
	ap_uint<32> accel_direct_1_occupied_pid;

	ap_uint<32> accel_indirect_0_occupied_pid;
	ap_uint<32> accel_indirect_1_occupied_pid;
	ap_uint<32> accel_indirect_2_occupied_pid;
	ap_uint<32> accel_indirect_3_occupied_pid;

	ap_uint<32> accel_sg_0_occupied_pid;


	ap_uint<32> accelerator_busy;
	ap_uint<32> open_modules;
};

struct shared_repository
{
	struct metrics accel_direct_0_shared_metrics;
	struct metrics accel_direct_1_shared_metrics;

	struct metrics accel_indirect_0_shared_metrics;
	struct metrics accel_indirect_1_shared_metrics;
	struct metrics accel_indirect_2_shared_metrics;
	struct metrics accel_indirect_3_shared_metrics;

	struct metrics accel_sg_0_shared_metrics;

	struct status_flags shared_status_flags;

};
