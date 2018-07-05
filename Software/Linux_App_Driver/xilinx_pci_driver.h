/*******************************************************************************
* Filename:   xilinx_pci_driver.h
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


/**
  * 
  * This Header File Contains the Necessary Macros for the Kernel Driver Module and the Userspace Application
  *  
  */
  
//---Valid Macros To Keep---------------------------------------------//
 
#define KC705_PCI_VENDOR_ID 0x10EE
#define KC705_PCI_DEVICE_ID 0x7021

#define VC707_PCI_VENDOR_ID 0x10EE
#define VC707_PCI_DEVICE_ID 0x7022

#define VENDOR_ID VC707_PCI_VENDOR_ID
#define DEVICE_ID VC707_PCI_DEVICE_ID

#define HAVE_REGION  0x01 // I/O Memory region
#define HAVE_IRQ     0x02 // Interupt
#define HAVE_KREG    0x04 // Kernel Registration
#define HAVE_DEBUGFS 0x08 // Debugfs File Creation

#define DEFAULT_SIGNAL_0 	34
#define DEFAULT_SIGNAL_1 	35
#define DEFAULT_SIGNAL_2 	36
#define DEFAULT_SIGNAL_3 	37
#define DEFAULT_SIGNAL_4 	38
#define DEFAULT_SIGNAL_5 	39
#define DEFAULT_SIGNAL_6 	40
#define DEFAULT_SIGNAL_SG 	41

#define OCCUPIED 1
#define NOT_OCCUPIED 0

/**
  * @note
  * 
  * The Following Macro Line Works as a Switch.
  * Remove/Add One of the '*' at the Beggining to Change the State of the Switch.
  * One '*' Enables --> BEST_AVAILABLE.
  * Two '*' Enable  --> GREEDY.
  * This is Used to Safely Disable/Enable Specific Code Parts of the Driver.
  */
 
/*/ #define BEST_AVAILABLE /*/ #define GREEDY /**/

/** @note
  * 
  * The Macros Below are Used to Enable/Disable Debug Messages.
  * 
  * The DEBUG_MESSAGES is Used to Print the Driver's Debug Messages to the /var/log/kern.log File.
  * The DEBUG_MESSAGES_UI is Used to Print the Userspace Application's Debug messages to the Terminal.
  * 
  * Uncomment to Enable the Messages Debugging.
  */
//#define DEBUG_MESSAGES
//#define DEBUG_MESSAGES_UI

#define SUCCESS 0
#define FAILURE 1

#define BYTE  1
#define KBYTE 1024
#define MBYTE 1048576

#define START 0x1
#define ACK 0x1


#define MMAP_ALLOCATION_SIZE 4 * MBYTE
#define POSIX_ALLOCATED_SIZE 32 * MBYTE
#define KERNEL_ALLOCATION_SIZE 4 * MBYTE


#define OPERATION_START_TIMER			0x18000000


#define BAR0_32BIT 0 //For 32 Bit Addressing
#define BAR1_32BIT 1 //For 32 Bit Addressing
#define BAR2_32BIT 2 //For 32 Bit Addressing
#define BAR3_32BIT 3 //For 32 Bit Addressing
#define BAR4_32BIT 4 //For 32 Bit Addressing
#define BAR5_32BIT 5 //For 32 Bit Addressing

#define BAR0_64BIT 0 //For 64 Bit Addressing
#define BAR1_64BIT 2 //For 64 Bit Addressing
#define BAR2_64BIT 4 //For 64 Bit Addressing


#define ACCELERATOR_DIRECT_0_OCCUPIED    0x01
#define ACCELERATOR_DIRECT_1_OCCUPIED    0x02
#define ACCELERATOR_INDIRECT_0_OCCUPIED  0x04
#define ACCELERATOR_INDIRECT_1_OCCUPIED  0x08
#define ACCELERATOR_INDIRECT_2_OCCUPIED  0x10
#define ACCELERATOR_INDIRECT_3_OCCUPIED  0x20
#define ACCELERATOR_SG_OCCUPIED	         0x40
#define ACCELERATOR_ALL_OCCUPIED         0x3F
#define ACCELERATOR_NO_OCCUPIED          0x00


#define ENABLE_GCC_MC 0x00010001 //Enable Global Clock Counter and Metrics Counter Mask
#define RESET_GCC_MC  0x00020002 //Reset Global Clock Counter and Metrics Counter Mask

#define APM_CR_OFFSET        0x300 //AXI Performance Monitor Control Register Offset(0x60 for Long Int Offset 0x300 for Byte Offset)
#define APM_GCC_LOWER_OFFSET 0X0004 //Global Clock Counter Lower 32Bits Register
#define APM_GCC_UPPER_OFFSET 0X0000 //Global Clock Counter Upper 32Bits Register

#define METRIC_SELECTOR_REGISTER_0_OFFSET 0X0044
#define METRIC_SELECTOR_REGISTER_1_OFFSET 0X0048
#define METRIC_SELECTOR_REGISTER_2_OFFSET 0X004C

//////////////////////////////////////////////////////////////////////////////////////
// PCIe BAR0 Address Space -Mapping the FPGA AXI Address Space (HW Peripherals)
//////////////////////////////////////////////////////////////////////////////////////

#define BAR0_OFFSET_INTERRUPT_CONTROLLER									0x00020000
#define BAR0_OFFSET_UARTLITE 												0x00010000
#define BAR0_OFFSET_PCIE_CTL												0x00020000
#define BAR0_OFFSET_GPIO_PCIE_INTERRUPT 									0x00030000
#define BAR0_OFFSET_GPIO_MSI												0x00040000
#define BAR0_OFFSET_TIMER													0x00050000
#define BAR0_OFFSET_FETCH_SCHEDULER											0x00060000
#define BAR0_OFFSET_SEND_SCHEDULER											0x00070000

#define BAR0_OFFSET_SCHEDULER_BUFFER_FETCH									0x00080000
#define BAR0_OFFSET_SCHEDULER_BUFFER_SEND									0x00090000

#define BAR0_OFFSET_CDMA_FETCH												0x000A0000
#define BAR0_OFFSET_CDMA_SEND												0x000B0000

#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT		0x000C0000
#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_APM								0x000D0000
#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_DMA								0x000E0000
#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_SOBEL_FILTER						0x000F0000

#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT		0x00100000
#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_APM								0x00110000
#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_DMA								0x00120000
#define BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_SOBEL_FILTER						0x00130000

#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT	0x00140000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_APM								0x00150000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_DMA								0x00160000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_SOBEL_FILTER						0x00170000

#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT	0x00180000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_APM								0x00190000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_DMA								0x001A0000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_SOBEL_FILTER						0x001B0000

#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT	0x001C0000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_APM								0x001D0000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_DMA								0x001E0000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_SOBEL_FILTER						0x001F0000

#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT	0x00200000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_APM								0x00210000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_DMA								0x00220000
#define BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_SOBEL_FILTER						0x00230000

#define BAR0_OFFSET_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG				0x00240000
#define BAR0_OFFSET_ACCEL_GROUP_SG_APM										0x00250000
#define BAR0_OFFSET_ACCEL_GROUP_SG_DMA_SG_PCIE_SCHEDULER					0x00260000
#define BAR0_OFFSET_ACCEL_GROUP_SG_SOBEL_FILTER_4K							0x00280000
#define BAR0_OFFSET_ACCEL_GROUP_SG_DMA										0x00290000

#define BAR0_OFFSET_GPIO_MSI_READ											0x00300000
#define BAR0_OFFSET_INTERRUPT_MANAGER										0x00310000
#define BAR0_OFFSET_GPIO_ACK												0x00320000

//////////////////////////////////////////////////////////////////////////////////////
// Acceleration Scheduler Direct Register Offsets
//////////////////////////////////////////////////////////////////////////////////////

#define ACCELERATION_SCHEDULER_DIRECT_CONTROL_REGISTER_OFFSET								0X00
#define ACCELERATION_SCHEDULER_DIRECT_GIE_REGISTER_OFFSET									0X04
#define ACCELERATION_SCHEDULER_DIRECT_IER_REGISTER_OFFSET									0X08
#define ACCELERATION_SCHEDULER_DIRECT_ISR_REGISTER_OFFSET									0X0C
#define ACCELERATION_SCHEDULER_DIRECT_DMA_DEVICE_BASE_ADDRESS_REGISTER_OFFSET				0X18
#define ACCELERATION_SCHEDULER_DIRECT_SOBEL_DEVICE_BASE_ADDRESS_REGISTER_OFFSET				0X20
#define ACCELERATION_SCHEDULER_DIRECT_GPIO_DEVICE_BASE_ADDRESS_REGISTER_OFFSET				0X28
#define ACCELERATION_SCHEDULER_DIRECT_APM_DEVICE_BASE_ADDRESS_REGISTER_OFFSET				0X30
#define ACCELERATION_SCHEDULER_DIRECT_SHARED_APM_DEVICE_BASE_ADDRESS_REGISTER_OFFSET		0X38
#define ACCELERATION_SCHEDULER_DIRECT_SHARED_METRICS_DEVICE_BASE_ADDRESS_REGISTER_OFFSET	0X40
#define ACCELERATION_SCHEDULER_DIRECT_IMAGE_COLUMNS_REGISTER_OFFSET							0X48
#define ACCELERATION_SCHEDULER_DIRECT_IMAGE_ROWS_REGISTER_OFFSET							0X50
#define ACCELERATION_SCHEDULER_DIRECT_HOST_SOURCE_ADDRESS_REGISTER_OFFSET					0X58
#define ACCELERATION_SCHEDULER_DIRECT_HOST_DESTINATION_ADDRESS_REGISTER_OFFSET				0X60
#define ACCELERATION_SCHEDULER_DIRECT_INITIATOR_GROUP_REGISTER_OFFSET						0X68


//////////////////////////////////////////////////////////////////////////////////////
// Acceleration Scheduler Indirect Register Offsets
//////////////////////////////////////////////////////////////////////////////////////

#define ACCELERATION_SCHEDULER_INDIRECT_CONTROL_REGISTER_OFFSET									0x00	
#define ACCELERATION_SCHEDULER_INDIRECT_GIE_REGISTER_OFFSET										0x04
#define ACCELERATION_SCHEDULER_INDIRECT_IER_REGISTER_OFFSET										0x08
#define ACCELERATION_SCHEDULER_INDIRECT_ISR_REGISTER_OFFSET										0x0C
#define ACCELERATION_SCHEDULER_INDIRECT_SCHEDULER_BUFFER_BASE_ADDRESS_FETCH_REGISTER_OFFSET		0x18
#define ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_REG_FETCH_REGISTER_OFFSET				0x20
#define ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_REG_FETCH_REGISTER_OFFSET			0x28
#define ACCELERATION_SCHEDULER_INDIRECT_DATA_SIZE_REG_FETCH_REGISTER_OFFSET						0x30
#define ACCELERATION_SCHEDULER_INDIRECT_OFFSET_REG__FETCH__REGISTER_OFFSET			            0x38
#define ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_FETCH_REGISTER_OFFSET					0x40
#define ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_FETCH_REGISTER_OFFSET				0x48
#define ACCELERATION_SCHEDULER_INDIRECT_OFFSET_FETCH_REGISTER_OFFSET                        	0x50
#define ACCELERATION_SCHEDULER_INDIRECT_SCHEDULER_BUFFER_BASE_ADDRESS_SEND_REGISTER_OFFSET		0x58
#define ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_REG_SEND_REGISTER_OFFSET					0x60
#define ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_REG_SEND_REGISTER_OFFSET			0x68
#define ACCELERATION_SCHEDULER_INDIRECT_DATA_SIZE_REG_SEND_REGISTER_OFFSET						0x70
#define ACCELERATION_SCHEDULER_INDIRECT_OFFSET_REG__SEND_REGISTER_OFFSET			            0x78
#define ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_SEND_REGISTER_OFFSET						0x80
#define ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_SEND_REGISTER_OFFSET				0x88
#define ACCELERATION_SCHEDULER_INDIRECT_OFFSET_SEND_REGISTER_OFFSET                        		0x90		
#define ACCELERATION_SCHEDULER_INDIRECT_DMA_BASE_ADDRESS_REGISTER_OFFSET						0x98
#define ACCELERATION_SCHEDULER_INDIRECT_SOBEL_BASE_ADDRESS_REGISTER_OFFSET						0xA0
#define ACCELERATION_SCHEDULER_INDIRECT_IMAGE_COLUMNS_REGISTER_OFFSET							0xA8
#define ACCELERATION_SCHEDULER_INDIRECT_IMAGE_ROWS_REGISTER_OFFSET								0xB0
#define ACCELERATION_SCHEDULER_INDIRECT_ACCEL_GROUP_REGISTER_OFFSET								0xB8
#define ACCELERATION_SCHEDULER_INDIRECT_SHARED_APM_BASE_ADDRESS_REGISTER_OFFSET					0xC0
#define ACCELERATION_SCHEDULER_INDIRECT_SHARED_METRICS_BASE_ADDRESS_REGISTER_OFFSET				0xC8
#define ACCELERATION_SCHEDULER_INDIRECT_APM_BASE_ADDRESS_REGISTER_OFFSET						0xD0             


//////////////////////////////////////////////////////////////////////////////////////
// Acceleration Scheduler SG Register Offsets
//////////////////////////////////////////////////////////////////////////////////////

#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_AP_CTRL                                 0x00
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_GIE                                     0x04
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_IER                                     0x08
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_ISR                                     0x0c
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_AP_RETURN                               0x10
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_DMA_SG_PCIE_SCHEDULER_BASE_ADDRESS_DATA 0x18
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_SOBEL_DEVICE_ADDRESS_DATA               0x20
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_GPIO_DEVICE_ADDRESS_DATA                0x28
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_APM_DEVICE_ADDRESS_DATA                 0x30
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_SHARED_APM_DEVICE_ADDRESS_DATA          0x38
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_SHARED_METRICS_ADDRESS_DATA             0x40
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_IMAGE_COLS_DATA                         0x48
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_IMAGE_ROWS_DATA                         0x50
#define XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_ACCEL_GROUP_DATA                        0x58




//////////////////////////////////////////////////////////////////////////////////////
// AXI BARs Offsets
//////////////////////////////////////////////////////////////////////////////////////

#define AXI_BAR_0_OFFSET 0x20000000
#define AXI_BAR_1_OFFSET 0x30000000
#define AXI_BAR_2_OFFSET 0x40000000
#define AXI_BAR_3_OFFSET 0x50000000
#define AXI_BAR_4_OFFSET 0x60000000
#define AXI_BAR_5_OFFSET 0x70000000


//////////////////////////////////////////////////////////////////////////////////////
// AXI BARs Dynamic Address Translation Registers Offsets
//////////////////////////////////////////////////////////////////////////////////////

#define AXI_BAR0_LOWER_ADDRESS_OFFSET 0x20C
#define AXI_BAR0_UPPER_ADDRESS_OFFSET 0x208

#define AXI_BAR1_LOWER_ADDRESS_OFFSET 0x214
#define AXI_BAR1_UPPER_ADDRESS_OFFSET 0x210

#define AXI_BAR2_LOWER_ADDRESS_OFFSET 0x21C
#define AXI_BAR2_UPPER_ADDRESS_OFFSET 0x218

#define AXI_BAR3_LOWER_ADDRESS_OFFSET 0x224
#define AXI_BAR3_UPPER_ADDRESS_OFFSET 0x220

#define AXI_BAR4_LOWER_ADDRESS_OFFSET 0x22C
#define AXI_BAR4_UPPER_ADDRESS_OFFSET 0x228

#define AXI_BAR5_LOWER_ADDRESS_OFFSET 0x234
#define AXI_BAR5_UPPER_ADDRESS_OFFSET 0x230



//////////////////////////////////////////////////////////////////////////////////////
// IOCtl Commands
//////////////////////////////////////////////////////////////////////////////////////

#define COMMAND_REQUEST_ACCELERATOR_ACCESS		0x0100
#define COMMAND_REQUEST_ACCELERATOR_SG_ACCESS	0x0200
#define COMMAND_SET_PAGES 						0x0300
#define COMMAND_UNMAP_PAGES						0x0400
#define COMMAND_RESET_VARIABLES					0x0500

//////////////////////////////////////////////////////////////////////////////////////
// Scenarios
//////////////////////////////////////////////////////////////////////////////////////

#define SCENARIO_SCATTER_GATHER 	1	
#define SCENARIO_WORST_CASE 		2
#define SCENARIO_WORST_CASE_CDMA	3

struct image_info
{
	uint32_t rows;
	uint32_t columns;
	uint64_t size;
};

struct metrics
{
	/*
	 * AXI Performance Monitor Metrics
	 */
	uint32_t apm_read_transactions; //Offset 0 Bytes
	uint32_t apm_read_bytes; //Offset 4 Bytes

	uint32_t apm_write_transactions; //Offset 8 Bytes
	uint32_t apm_write_bytes; //Offset 12 Bytes

	uint32_t apm_packets; //Offset 16 Bytes
	uint32_t apm_bytes; //Offset 20 Bytes

	uint32_t apm_gcc_l; //Offset 24 Bytes
	uint32_t apm_gcc_u; //Offset 28 Bytes

	uint32_t cdma_fetch_time_start_l; //Offset 32 Bytes
	uint32_t cdma_fetch_time_start_u; //Offset 36 Bytes
	uint32_t cdma_fetch_time_end_l; //Offset 40 Bytes
	uint32_t cdma_fetch_time_end_u; //Offset 44 Bytes

	uint32_t cdma_send_time_start_l; //Offset 48 Bytes
	uint32_t cdma_send_time_start_u; //Offset 52 Bytes
	uint32_t cdma_send_time_end_l; //Offset 56 Bytes
	uint32_t cdma_send_time_end_u; //Offset 60 Bytes

	uint32_t dma_accel_time_start_l; //Offset 64 Bytes
	uint32_t dma_accel_time_start_u; //Offset 68 Bytes
	uint32_t dma_accel_time_end_l; //Offset 72 Bytes
	uint32_t dma_accel_time_end_u; //Offset 76 Bytes

	struct image_info shared_image_info; // Offset 80 Bytes
	
	/*
	 * Kernel and Userspace Metrics
	 */

	uint64_t total_time_start;
	uint64_t total_time_end;

	uint64_t sleep_time_start;
	uint64_t sleep_time_end;

	uint64_t preparation_time_start;
	uint64_t preparation_time_end;

	uint64_t load_time_start;
	uint64_t load_time_end;

	uint64_t save_time_start;
	uint64_t save_time_end;	

};

struct metrics_per_process
{
	struct metrics agd0;
	struct metrics agd1;
	
	struct metrics agi0;
	struct metrics agi1;
	struct metrics agi2;
	struct metrics agi3;
	
	struct metrics agsg;

	/*
	 * Kernel and Userspace Metrics
	 */

	uint64_t total_time_start;
	uint64_t total_time_end;

	uint64_t sleep_time_start;
	uint64_t sleep_time_end;

	uint64_t preparation_time_start;
	uint64_t preparation_time_end;

	uint64_t load_time_start;
	uint64_t load_time_end;

	uint64_t save_time_start;
	uint64_t save_time_end;
	
	uint64_t set_pages_overhead_time_start;
	uint64_t set_pages_overhead_time_end;
	
	uint64_t unmap_pages_overhead_time_start;
	uint64_t unmap_pages_overhead_time_end;	


};

struct status_flags
{
	uint32_t accel_direct_0_occupied_pid;
	
	uint32_t accel_direct_1_occupied_pid;

	uint32_t accel_indirect_0_occupied_pid;
	
	uint32_t accel_indirect_1_occupied_pid;
	
	uint32_t accel_indirect_2_occupied_pid;
	
	uint32_t accel_indirect_3_occupied_pid;

	uint32_t accel_sg_0_occupied_pid;


	uint32_t accelerator_busy;
	uint32_t open_modules;
	
	uint32_t agd0_busy;
	uint32_t agd1_busy;
	uint32_t agi0_busy;
	uint32_t agi1_busy;
	uint32_t agi2_busy;
	uint32_t agi3_busy;
	uint32_t agsg_busy;
	
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

struct shared_repository_process
{
	struct metrics_per_process process_metrics;
	struct image_info shared_image_info;
	int accel_completed;
	int accel_occupied;
	int image_segments;

};

typedef struct {
    uint8_t magic[2];
} bmpfile_magic_t;

typedef struct {
    uint32_t filesz;
    uint16_t creator1;
    uint16_t creator2;
    uint32_t bmp_offset;
} bmpfile_header_t;

typedef struct {
    uint32_t header_sz;
    int32_t  width;
    int32_t  height;
    uint16_t nplanes;
    uint16_t bitspp;
    uint32_t compress_type;
    uint32_t bmp_bytesz;
    int32_t  hres;
    int32_t  vres;
    uint32_t ncolors;
    uint32_t nimpcolors;
} bitmap_info_header_t;


typedef struct {
    uint8_t b;
    uint8_t g;
    uint8_t r;
    uint8_t nothing;
} rgb_t;


typedef unsigned char pixel_t;


struct pid_reserved_memories {
	
	pid_t pid;
	
	struct shared_repository_process *shared_repo_virtual_address;
	uint32_t shared_repo_physical_address;
	
	uint64_t *pre_process_mmap_virtual_address;
	uint32_t pre_process_mmap_physical_address;
	
	uint64_t *post_process_mmap_virtual_address;
	uint32_t post_process_mmap_physical_address;
	
	struct sg_table *dma_sg_table_source;
	struct scatterlist *scatterlist_pointer_source;

	int buffer_dma_buffers_source;
	
	int buffer_mapped_pages_source;
	

	struct sg_table *dma_sg_table_destination;
	struct scatterlist *scatterlist_pointer_destination;

	int buffer_dma_buffers_destination;	
	
	int buffer_mapped_pages_destination;	
	
	uint64_t *u64_sg_list_source;
	uint64_t *u64_sg_list_destination;
	
	struct pid_reserved_memories *next_pid;
	
};

struct sg_list_addresses
{
	pid_t current_pid;
		
	uint64_t *sg_list_source_address;
	uint64_t *sg_list_destination_address;
	
};

struct per_thread_info
{
	struct shared_repository_process *shared_repo_kernel_address;
	uint8_t *u8_pre_process_kernel_address;
	uint8_t *u8_post_process_kernel_address;

	int pre_process_mmap_file;
	int post_process_mmap_file;
	int shared_repo_mmap_file;
};
