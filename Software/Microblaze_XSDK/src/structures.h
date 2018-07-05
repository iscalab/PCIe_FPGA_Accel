/*******************************************************************************
* Filename:   structures.h
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


struct image_info
{
	u32 rows;
	u32 columns;
	u64 size;
};

struct metrics
{
	/*
	 * AXI Performance Monitor Metrics
	 */
	u32 apm_read_transactions; //Offset 0 Bytes
	u32 apm_read_bytes; //Offset 4 Bytes

	u32 apm_write_transactions; //Offset 8 Bytes
	u32 apm_write_bytes; //Offset 12 Bytes

	u32 apm_packets; //Offset 16 Bytes
	u32 apm_bytes; //Offset 20 Bytes

	u32 apm_gcc_l; //Offset 24 Bytes
	u32 apm_gcc_u; //Offset 28 Bytes

	u32 cdma_fetch_time_start_l; //Offset 32 Bytes
	u32 cdma_fetch_time_start_u; //Offset 36 Bytes
	u32 cdma_fetch_time_end_l; //Offset 40 Bytes
	u32 cdma_fetch_time_end_u; //Offset 44 Bytes

	u32 cdma_send_time_start_l; //Offset 48 Bytes
	u32 cdma_send_time_start_u; //Offset 52 Bytes
	u32 cdma_send_time_end_l; //Offset 56 Bytes
	u32 cdma_send_time_end_u; //Offset 60 Bytes

	u32 dma_accel_time_start_l; //Offset 64 Bytes
	u32 dma_accel_time_start_u; //Offset 68 Bytes
	u32 dma_accel_time_end_l; //Offset 72 Bytes
	u32 dma_accel_time_end_u; //Offset 76 Bytes

	struct image_info shared_image_info; // Offset 80 Bytes

	/*
	 * Kernel and Userspace Metrics
	 */

	u64 total_time_start;
	u64 total_time_end;

	u64 sleep_time_start;
	u64 sleep_time_end;

	u64 preparation_time_start;
	u64 preparation_time_end;

	u64 load_time_start;
	u64 load_time_end;

	u64 save_time_start;
	u64 save_time_end;


};

struct metrics_per_process
{
	/*
	 * AXI Performance Monitor Metrics
	 */
	u32 apm_read_transactions; //Offset 0 Bytes
	u32 apm_read_bytes; //Offset 4 Bytes

	u32 apm_write_transactions; //Offset 8 Bytes
	u32 apm_write_bytes; //Offset 12 Bytes

	u32 apm_packets; //Offset 16 Bytes
	u32 apm_bytes; //Offset 20 Bytes

	u32 apm_gcc_l; //Offset 24 Bytes
	u32 apm_gcc_u; //Offset 28 Bytes

	u32 cdma_fetch_time_start_l; //Offset 32 Bytes
	u32 cdma_fetch_time_start_u; //Offset 36 Bytes
	u32 cdma_fetch_time_end_l; //Offset 40 Bytes
	u32 cdma_fetch_time_end_u; //Offset 44 Bytes

	u32 cdma_send_time_start_l; //Offset 48 Bytes
	u32 cdma_send_time_start_u; //Offset 52 Bytes
	u32 cdma_send_time_end_l; //Offset 56 Bytes
	u32 cdma_send_time_end_u; //Offset 60 Bytes

	u32 dma_accel_time_start_l; //Offset 64 Bytes
	u32 dma_accel_time_start_u; //Offset 68 Bytes
	u32 dma_accel_time_end_l; //Offset 72 Bytes
	u32 dma_accel_time_end_u; //Offset 76 Bytes

	struct image_info shared_image_info; // Offset 80 Bytes

	/*
	 * Kernel and Userspace Metrics
	 */

	u64 total_time_start;
	u64 total_time_end;

	u64 sleep_time_start;
	u64 sleep_time_end;

	u64 preparation_time_start;
	u64 preparation_time_end;

	u64 load_time_start;
	u64 load_time_end;

	u64 save_time_start;
	u64 save_time_end;

	u64 set_pages_overhead_time_start;
	u64 set_pages_overhead_time_end;

	u64 unmap_pages_overhead_time_start;
	u64 unmap_pages_overhead_time_end;


};

struct status_flags
{
	u32 accel_direct_0_occupied_pid;
	u32 accel_direct_1_occupied_pid;

	u32 accel_indirect_0_occupied_pid;
	u32 accel_indirect_1_occupied_pid;
	u32 accel_indirect_2_occupied_pid;
	u32 accel_indirect_3_occupied_pid;

	u32 accel_sg_0_occupied_pid;


	u32 accelerator_busy;
	u32 open_modules;
};

struct shared_repository
{
	struct metrics unused_shared_metrics;

	struct metrics accel_direct_0_shared_metrics;
	struct metrics accel_direct_1_shared_metrics;

	struct metrics accel_indirect_0_shared_metrics;
	struct metrics accel_indirect_1_shared_metrics;
	struct metrics accel_indirect_2_shared_metrics;
	struct metrics accel_indirect_3_shared_metrics;

	struct metrics accel_sg_0_shared_metrics;

	struct status_flags shared_status_flags;

};
