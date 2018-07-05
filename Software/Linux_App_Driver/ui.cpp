/*******************************************************************************
* Filename:   ui.cpp
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


/*
 * --------------
 * Public Headers
 * ---------------->
 */
 
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/file.h>


/*
 * -------------
 * Local Headers
 * --------------->
 */
  
#include "xilinx_pci_driver.h"

/*
 * ----------------
 * Global Variables
 * ------------------>
 */ 


/*
 * Flags (access_status_x) Used to Indicate the Completion of an Acceleration Group.
 * Their Values Change by the Corresponding Signal Handler Below Depending on the Received Signal.
 * The Application Remains in Polling Mode Reading One/Some of the Following Flags 
 * until it Changes to Non-Zero Value Indicating the Completion of the Acceleration.
 */
int access_status_0 = 0;
int access_status_1 = 0;
int access_status_2 = 0;
int access_status_3 = 0;
int access_status_4 = 0;
int access_status_5 = 0;
int access_status_6 = 0;
int access_status_7 = 0;

/*
 * total_reserved_size is the Total Number of Bytes that an Image Requires which is (Width * Height * 4).
 * The Images are in .bmp Format which Requires 3 Bytes for each Pixel.
 * In order to Have Aligned Image Trasfers the Acceleration System Requires 4 Bytes per Pixel.
 * As a Result Each Pixel Uses an Extra Byte (Dummy) as Padding to Fill the 4 Bytes Requirement.
 */
int total_reserved_size;

/*
 * The uint_shared_kernel_address is Used to Map the PCIe BAR1 of the PCIe Bridge to the Userspace Virtual Address Space.
 * PCIe BAR1 Represents a BRAM Memory Inside the FPGA which is Used to Store Metrics Information, SG Lists and Synchronization Flags.
 * By Mapping the PCIe BAR1 the Userspace Application ic Able to Have Direct Access to the BRAM Memory of the FPGA.
 * The shared_kernel_address Pointer, also, Points to the BRAM Memory as the uint_shared_kernel_address Pointer but it Makes it Possible
 * to Access the Memory According to the Fields of the struct shared_repository
 */
unsigned int *uint_shared_kernel_address = NULL;
struct shared_repository *shared_kernel_address = NULL;

/*
 * The uint_32_pcie_bar_kernel_address is Used to Map the PCIe BAR0 of the PCIe Bridge to the Userspace Virtual Address Space.
 * PCIe BAR0 Represents the AXI Address Space of the FPGA where All the FPGA Peripherals are Mapped.
 * By Mapping the PCIe BAR0 the Userspace Application is Able to Have Direct Access to the Peripherals of the FPGA.
 * The uint_64_pcie_bar_kernel_address Pointer, also, Points to the FPGA Peripherals as the uint_32_pcie_bar_kernel_address Pointer but
 * it Makes it Possible to Make 64 Bit Data Reads/Writes.
 */
unsigned int *uint_32_pcie_bar_kernel_address = NULL;
uint64_t *uint_64_pcie_bar_kernel_address = NULL;

/*
 * The sigaction Structures are Used to Setup the Signal Handlers for each Signal Received by the Driver
 * There are 8 Signals Specified each for the 7 Acceleration Groups (Two Signals for the Scatter/Gather)
 */
struct sigaction interrupt_signal_action_0;
struct sigaction interrupt_signal_action_1;
struct sigaction interrupt_signal_action_2;
struct sigaction interrupt_signal_action_3;
struct sigaction interrupt_signal_action_4;
struct sigaction interrupt_signal_action_5;
struct sigaction interrupt_signal_action_6;
struct sigaction interrupt_signal_action_sg;

/*
 * pcie_bar_0_mmap_file and pcie_bar_1_mmap_file are Used to Map the PCIe BAR0 and PCIe BAR1 to the Userspace.
 * This Way the Userspace Application Can Have Direct Access to the FPGA Peripherals And Memories.
 */
int pcie_bar_0_mmap_file;
int pcie_bar_1_mmap_file;

/*
 * Structures that are Used to Store Info from the Header of the Image File
 * 
 * bmpfile_magic_t magic_number --> The Structure that Stores the Magic Number of the Bitmap Image
 * bmpfile_header_t bitmap_file_header --> The File Header of the Bitmap Image
 * bitmap_info_header_t bitmap_info_header --> The Info Header of the Bitmap Image
 */
bmpfile_magic_t magic_number;
bmpfile_header_t bitmap_file_header;
bitmap_info_header_t bitmap_info_header;


/*
 * Used to Store the Process ID (PID)
 */
unsigned int pid;

/*
 * common_load (Pointer) Points to an Allocated Memory where the Image is Initially Loaded from the Storage Device.
 * This Allocated Memory is Shareable among the Threads of the Application.
 * Each Thread Copies the Image Data from this Shareable Memory to Its Exclusive Memory Allocation before Requesting Acceleration
 * This Technique is Used to Avoid Loading the Image Multiple Times for each Iteration of each Thread which Has Serious Latency Impact
 */
uint8_t *common_load;

/*
 * global_iterations Indicates the Number of Times that each Thread Should Run to Request Acceleration
 * The Value of this Variable is Given as an Argument when this Application is Called
 */
int global_iterations = 0;

/*
 * save_request Indicates whether the Thread is going to Save the Accelerated Image.
 * Saving an Image Has Serious Latency Impact thus it Should be Optional in Some Cases
 * The Value of this Variable is Given as an Argument when this Application is Called
 * There are 3 Different Values that this Variable Can be Given:
 * 
 * 0 --> Do Not Save the Image
 * 1 --> Save the Image in EACH Iteration
 * 2 --> Save the Image ONLY in the Last Iteration
 */
int save_request = 0;

/*
 * load_path_name is Used to Store the Path and Filename of the Image File that the Application is going to Load
 * The Value of this Array is Given as an Argument when this Application is Called
 */
char load_path_name[100];

/*
 * Structure pthread_barrier_t is Used to Force all the Threads of the Application to Start Simultaneously
 * A Barrier is a Point where the Thread is Going to Wait for other Threads and Will Proceed Further only when
 * the Predefined Number of Threads Reach the Same Barrier. 
 */
pthread_barrier_t threads_barrier;

/*
 * renamer_value is Used when Saving the Multiple .xml Files that Keep Metrics Info.
 * The Application will Run a Number of Tests which is Given as an Argument.
 * Each Test Raises a Number of Threads which is, also, Given as an Argument
 * and each Thread Makes a Number of Acceleration Requests According to the global_iterations Described Above.
 * For Each Test the Application Saves a new .xml File.
 * In order to Dynamically Name each .xml File we use the renamer_value Variable.
 * The Application Reads an Arithmetic Value from a Specific .txt File and Stores it in the renamer_value.
 * The new .xml File is Named According to the renamer_value.
 * The Arithmetic Value of the .txt File is then Incremented for the Next .xml File.
 */
int renamer_value = 0;


/*
 * ---------------------
 * Functions Declaration
 * ----------------------->
 */ 
 
void signal_handler_0(int, siginfo_t *, void *);
void signal_handler_1(int, siginfo_t *, void *);
void signal_handler_2(int, siginfo_t *, void *);
void signal_handler_3(int, siginfo_t *, void *);
void signal_handler_4(int, siginfo_t *, void *);
void signal_handler_5(int, siginfo_t *, void *);
void signal_handler_6(int, siginfo_t *, void *);
void signal_handler_sg(int, siginfo_t *, void *);
 
int setup_signal_handling();
void clear_screen();

int load_bmp(uint8_t *);
int save_bmp(uint8_t *, char *);

uint64_t convert_cycles_2_ns(uint64_t);
int file_size(FILE *);

int print_save_metrics(struct shared_repository_process *, int, unsigned int, int);
int set_save_accelerator(char *, int, int, int);

int pcie_bar_mmap();
struct shared_repository_process * shared_repo_mmap(struct per_thread_info *);

uint8_t * pre_process_mmap(struct per_thread_info *);
uint8_t * post_process_mmap(struct per_thread_info *);

void* start_thread(void *);

int multi_threaded_acceleration(int);

int acceleration_thread();	 


/*
 * ---------------------
 * Functions Description
 * ----------------------->
 */ 


/* OK
 * signal_handler_x()
 * 
 * The Driver Can Send Signals to the Application to Indicate the Completion of an Acceleration.
 * There is One Signal Dedicated for each Acceleration Group.
 * When the Application Requests Acceleration it Remains in Polling Mode Reading a Specific Flag 
 * that Indicates the Status of the Acceleration.
 * The Signal Handler is Called to Change the Corresponding Flag Depending on the Acceleration Group on Completion of the Acceleration.
 * The Signal Method is not Currently Used but is Kept for Future Implementations that Might Require Signal Handling
 */
void signal_handler_0(int sig, siginfo_t *siginfo, void *c)
{
	
	#ifdef DEBUG_MESSAGES_UI
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
	
	access_status_0 = DEFAULT_SIGNAL_0;//ACCELERATOR_DIRECT_0_OCCUPIED;
							
}

void signal_handler_1(int sig, siginfo_t *siginfo, void *c) 
{
	
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_1 = DEFAULT_SIGNAL_1;//ACCELERATOR_DIRECT_1_OCCUPIED;
}

void signal_handler_2(int sig, siginfo_t *siginfo, void *c) 
{	
	
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_2 = DEFAULT_SIGNAL_2;//ACCELERATOR_INDIRECT_0_OCCUPIED;

}

void signal_handler_3(int sig, siginfo_t *siginfo, void *c) 
{	
		
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_3 = DEFAULT_SIGNAL_3;//ACCELERATOR_INDIRECT_1_OCCUPIED;
	
}

void signal_handler_4(int sig, siginfo_t *siginfo, void *c) 
{	
	
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_4 = DEFAULT_SIGNAL_4;//ACCELERATOR_INDIRECT_2_OCCUPIED;

}

void signal_handler_5(int sig, siginfo_t *siginfo, void *c) 
{	
	
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_5 = DEFAULT_SIGNAL_5;//ACCELERATOR_INDIRECT_3_OCCUPIED;
	
}

void signal_handler_6(int sig, siginfo_t *siginfo, void *c) 
{	
	
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_6 = DEFAULT_SIGNAL_6;//ACCELERATOR_SG_OCCUPIED;

}

void signal_handler_sg(int sig, siginfo_t *siginfo, void *c) 
{		
		
	#ifdef DEBUG_MESSAGES_UI	
	printf("Received Signal %d from Kernel Module\n", sig);
	#endif
		
	access_status_7 = DEFAULT_SIGNAL_SG;//ACCELERATOR_SG_TO_OCCUPY;
	
}


/* OK
 * setup_signal_handling()
 * 
 * Sets Up each Signal Handler with a Dedicated Signal
 */
int setup_signal_handling()
{
	unsigned int pid;
 
	/*
	 * Set the struct interrupt_signal_action_0 with the Signal Handler (signal_handler_0) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_0
	 */
	interrupt_signal_action_0.sa_sigaction = &signal_handler_0;
	
	/*
	 * Set the Flags for the struct interrupt_signal_action_0
	 */
	interrupt_signal_action_0.sa_flags = SA_SIGINFO;
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_0 Signal According to the struct interrupt_signal_action_0.
	 */
	if (sigaction(DEFAULT_SIGNAL_0, &interrupt_signal_action_0, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 0 for Kernel Signals\n");
		#endif
		return -1;
   	}
   	
	/*
	 * Set the struct interrupt_signal_action_1 with the Signal Handler (signal_handler_1) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_1
	 */   	
	interrupt_signal_action_1.sa_sigaction = &signal_handler_1;
	
	/*
	 * Set the Flags for the struct interrupt_signal_action_1
	 */	
	interrupt_signal_action_1.sa_flags = SA_SIGINFO;
 
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_1 Signal According to the struct interrupt_signal_action_1.
	 */ 
	if (sigaction(DEFAULT_SIGNAL_1, &interrupt_signal_action_1, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 1 for Kernel Signals\n");
		#endif
		return -1;
   	}  
   	
	/*
	 * Set the struct interrupt_signal_action_2 with the Signal Handler (signal_handler_2) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_2
	 */      	
	interrupt_signal_action_2.sa_sigaction = &signal_handler_2;
	
	/*
	 * Set the Flags for the struct interrupt_signal_action_2
	 */		
	interrupt_signal_action_2.sa_flags = SA_SIGINFO;
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_2 Signal According to the struct interrupt_signal_action_2.
	 */  
	if (sigaction(DEFAULT_SIGNAL_2, &interrupt_signal_action_2, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 2 for Kernel Signals\n");
		#endif
		return -1;
   	}    	
   	
	/*
	 * Set the struct interrupt_signal_action_3 with the Signal Handler (signal_handler_3) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_3
	 */   	
	interrupt_signal_action_3.sa_sigaction = &signal_handler_3;
	
	/*
	 * Set the Flags for the struct interrupt_signal_action_3
	 */		
	interrupt_signal_action_3.sa_flags = SA_SIGINFO;
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_3 Signal According to the struct interrupt_signal_action_3.
	 */  
	if (sigaction(DEFAULT_SIGNAL_3, &interrupt_signal_action_3, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 3 for Kernel Signals\n");
		#endif
		return -1;
   	}      
   	
	/*
	 * Set the struct interrupt_signal_action_4 with the Signal Handler (signal_handler_4) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_4
	 */     	
	interrupt_signal_action_4.sa_sigaction = &signal_handler_4;
	
	/*
	 * Set the Flags for the struct interrupt_signal_action_4
	 */		
	interrupt_signal_action_4.sa_flags = SA_SIGINFO;
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_4 Signal According to the struct interrupt_signal_action_4.
	 */ 
	if (sigaction(DEFAULT_SIGNAL_4, &interrupt_signal_action_4, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 4 for Kernel Signals\n");
		#endif
		return -1;
   	}     		
   	
	/*
	 * Set the struct interrupt_signal_action_5 with the Signal Handler (signal_handler_5) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_5
	 */    	
	interrupt_signal_action_5.sa_sigaction = &signal_handler_5;
	
	/*
	 * Set the Flags for the struct interrupt_signal_action_5
	 */		
	interrupt_signal_action_5.sa_flags = SA_SIGINFO;
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_5 Signal According to the struct interrupt_signal_action_5.
	 */ 
	if (sigaction(DEFAULT_SIGNAL_5, &interrupt_signal_action_5, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 5 for Kernel Signals\n");
		#endif
		return -1;
   	}    
   	
	/*
	 * Set the struct interrupt_signal_action_6 with the Signal Handler (signal_handler_6) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_6
	 */    	
   	interrupt_signal_action_6.sa_sigaction = &signal_handler_6;
   	
	/*
	 * Set the Flags for the struct interrupt_signal_action_6
	 */	   	
	interrupt_signal_action_6.sa_flags = SA_SIGINFO;
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_6 Signal According to the struct interrupt_signal_action_6.
	 */  
	if (sigaction(DEFAULT_SIGNAL_6, &interrupt_signal_action_6, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action 6 for Kernel Signals\n");
		#endif
		return -1;
   	}  
   	
	/*
	 * Set the struct interrupt_signal_action_sg with the Signal Handler (signal_handler_sg) that will be Used when the Driver Triggers the DEFAULT_SIGNAL_SG
	 */    	
   	interrupt_signal_action_sg.sa_sigaction = &signal_handler_sg;
   	
	/*
	 * Set the Flags for the struct interrupt_signal_action_sg
	 */	   	
	interrupt_signal_action_sg.sa_flags = SA_SIGINFO;
 
 
	/*
	 * Call sigaction() Function which Specifies the Action to be Associated with the DEFAULT_SIGNAL_SG Signal According to the struct interrupt_signal_action_sg.
	 */ 
	if (sigaction(DEFAULT_SIGNAL_SG, &interrupt_signal_action_sg, NULL) < 0) 
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Could not Setup Action Scatter/Gather for Kernel Signals\n");
		#endif
		return -1;
   	}    	
   	 	  	

	pid = getpid();
	
	#ifdef DEBUG_MESSAGES_UI
	printf("[DEBUG MESSAGE] Process ID is: %d\n", pid);
	#endif
	
	return 1;
}


/* OK
 * clear_screen()
 * 
 * Prints a Specified String to The Terminal that Forces the Terminal to Clear Its Screen
 */
void clear_screen() 
{
	/*
	 * Clear Screen and Move to Top-Left Corner
	 */
	printf("\033[2J\033[1;1H");
	
}


/* OK
 * load_bmp()
 * 
 * Used to Load the Image from the Storage Device to a Given Memory
 * According to the u8_pre_process_kernel_address Pointer.
 * According to the Current Implementation the u8_pre_process_kernel_address Points Directly to an Allocated Memory in Kernel Space
 */
int load_bmp(uint8_t *u8_pre_process_kernel_address) 
{
	size_t total_read_bytes;
	
	FILE *bmp_file;

	int status;
	int repeat;
	size_t pad;
	uint8_t current_byte;
	
	int count = 0;
	int i,j;
		
	#ifdef DEBUG_MESSAGES_UI	
	printf("The Path is: %s\n", load_path_name);
		
	printf("Loading the Image File\n");
    #endif
    
    /*
     * Open the Image File According to the File Name Given by the the User
     */
    bmp_file = fopen(load_path_name, "r");

    if(bmp_file != NULL)
    {
		#ifdef DEBUG_MESSAGES_UI
    	printf("Image File Opened\n");
    	#endif
    }
    else
    {
        if(bmp_file == NULL)
        {
        	printf("Image Failed to Open [NULL Pointer]\n");
        }
	
   	usleep(2000000);

   	return(FAILURE);
    }
    
    
    #ifdef DEBUG_MESSAGES_UI
    printf("Checking the Magic Number to Validate that this is a Bitmap File\n");
    #endif

	/*
	 * Read the Magic Number from the Header of the Bitmap File.
	 */
    fread(&magic_number, sizeof(bmpfile_magic_t), 1, bmp_file);

	/*
	 * Check the Magic Number to Validate that this is a Bitmap File.
	 * The Magic Number for .bmp Files is: 0x4D42.
	 */
	if (*((uint16_t *)magic_number.magic) == 0x4D42)
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("Bitmap File Valid [MAGIC NUMBER 0x%X]\n", *((uint16_t *)magic_number.magic));
		#endif
	}
	else
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("No Bitmap File Was Found/Aborting\n");
		#endif
		fclose(bmp_file);
		return FAILURE;
	}
    
    

    #ifdef DEBUG_MESSAGES_UI
    printf("Reading the Bitmap File Header\n");
    #endif

    /*
     * Read the Bitmap File Header
     */
    fread(&bitmap_file_header, sizeof(bmpfile_header_t), 1, bmp_file);


    #ifdef DEBUG_MESSAGES_UI
    printf("Reading the Bitmap Info Header\n");
    #endif

    /*
     * Read the Bitmap Info Header
     */
    fread(&bitmap_info_header, sizeof(bitmap_info_header_t), 1, bmp_file);


	 #ifdef DEBUG_MESSAGES_UI
	 printf("Checking Compression\n");
	 #endif
	 
	/*
	 * Check the Info Header Structure to See if Compression is Supported
	 */	 
	if (bitmap_info_header.compress_type == 0)
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("Compression is Supported\n");
		#endif
	}
	else
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("Warning, Compression is not Supported\n");
		#endif
	}	  
	
	/*
	 * Print Information About the Image
	 */
	 #ifdef DEBUG_MESSAGES_UI
	printf("\n* Image Width:       %d Pixels\n", bitmap_info_header.width);
	printf("* Image Height:      %d Pixels\n", bitmap_info_header.height);
	printf("* Image Size:        %d Bytes\n", bitmap_info_header.bmp_bytesz);
	printf("* Image Header Size: %d Bytes\n", bitmap_info_header.header_sz);
	printf("* Bits Per Pixel:    %d \n\n", bitmap_info_header.bitspp);	
	#endif 
	 
	
	/*
	 * Read the Info Header to Make Sure that the Image Resolution is up to 1920x1080 which is the Maximum Supported
	 */
	if((bitmap_info_header.width > 1920) || (bitmap_info_header.height >1080)) 
	{
		printf("The Image Cannot be Processed due to Sobel Accelerator's Restricted Resolution at Maximum of 3840x2160/Aborting\n");
		fclose(bmp_file);
		
		usleep(5000000);
		
		return FAILURE;
	}
	  
	/*
	 * Move the File Pointer at the Beginning of the Bitmap File
	 */
	fseek(bmp_file, bitmap_file_header.bmp_offset, SEEK_SET);

	#ifdef DEBUG_MESSAGES_UI
	printf("Moved File Pointer at the Beginning of Bitmap Data\n");  	  
	#endif
    
    
    /*
     * Get the Total Size Required for the Image Data.
     * See Details at the Global Variables Section at the Comments for the total_reserved_size Variable.s
     */
    total_reserved_size =  (bitmap_info_header.width * bitmap_info_header.height) * 4;
   
    #ifdef DEBUG_MESSAGES_UI
    printf("The Total Reserved Size Should Be: %d\n", total_reserved_size); 
    #endif

	/*
	 * Calculate the Possible Padding that Might be Found at the end of an Image Row.
	 */
	pad = (4 - (((bitmap_info_header.bitspp / 8)*bitmap_info_header.width) % 4)) % 4;     
    
    #ifdef DEBUG_MESSAGES_UI
    printf("The Padding is: %d\n", pad);	
    #endif

	/*
	 * Loop for the Number of Image Rows
	 */
    for(i=0; i<bitmap_info_header.height; i++)
    {
		/*
		 * Loop for the Number of Image Columns (The Pixels of each Row)
		 */
		for(j=0; j<(bitmap_info_header.width); j++)
		{	
			/*
			 * Read and Store the First Byte of the Current Pixel
			 */
			fread(&current_byte, sizeof(uint8_t), 1, bmp_file);
			u8_pre_process_kernel_address[count++] = current_byte;
			
			/*
			 * Read and Store the Second Byte of the Current Pixel
			 */			
			fread(&current_byte, sizeof(uint8_t), 1, bmp_file);
			u8_pre_process_kernel_address[count++] = current_byte;
		
			/*
			 * Read and Store the Third Byte of the Current Pixel
			 */			
			fread(&current_byte, sizeof(uint8_t), 1, bmp_file);
			u8_pre_process_kernel_address[count++] = current_byte;
			
			/*
			 * Store a Padding Byte in Order to Have 4 Bytes Alignment per Pixel.
			 */				
			u8_pre_process_kernel_address[count++] = 0x0;
		}
		
		/*
		 * Move the Position Indicator of the File According to the pad Variable so that we Get to the Beginning of the Next Image Row.
		 * The fseek() Herein is Required because there might be a Chance that in each Row there is a Number of Padding Bytes after the End of the Valid Image Data of each Row.
		 * As a Result Using fseek() is the Way to Move to the Valid Image Data of the Next Row.
		 */
		fseek(bmp_file, pad, SEEK_CUR);
	}
	
	#ifdef DEBUG_MESSAGES_UI
    printf("The Image Data is Loaded\n");	
	#endif
	
    /*
     * Close the Bitmap File.
     */
    fclose(bmp_file);
	
	return SUCCESS;
}


/* OK
 * save_bmp()
 * 
 * Used to Save the Processed Image from a Memory Given by the u8_post_process_kernel_address Pointer
 * to the Storage Device at the Directory and Filename Given by the save_path_name Pointer.
 * According to the Current Implementation the u8_post_process_kernel_address Points Directly to an Allocated Memory in Kernel Space
 */
int save_bmp(uint8_t *u8_post_process_kernel_address, char *save_path_name) 
{
	size_t total_written_bytes;
	
	FILE *bmp_file;
	
	bmpfile_magic_t save_magic_number;
	bmpfile_header_t save_bitmap_file_header;

	size_t i;
	size_t j;
	size_t k;
	size_t pad;	
	
	uint8_t current_byte;
	
	#ifdef DEBUG_MESSAGES_UI
	printf("Saving the Image File\n");
	#endif
    
    /*
     * Open the Image File According to the File Name Given by the the User
     */
    bmp_file = fopen(save_path_name, "wb");

    if(bmp_file != NULL)
    {
		#ifdef DEBUG_MESSAGES_UI
    	printf("New Image File Opened\n");
    	#endif
    }
    else
    {
        if(bmp_file == NULL)
        {
        	printf("New Image File Failed to Open [NULL Pointer]\n");
        }
	
   	usleep(2000000);

   	return(FAILURE);
    }
	

	#ifdef DEBUG_MESSAGES_UI
	printf("[SAVE PROCESS] Writing the Magic Number at the New Image File\n");
	#endif

	/*
	 * Set the Magic Number Structure with the .bmp Image Magic Number.
	 */
	save_magic_number.magic[0] = 0x42;
	save_magic_number.magic[1] = 0x4d;

	/*
	 * Write the Magic Number Inside the File
	 */		
	if (fwrite(&save_magic_number, sizeof(bmpfile_magic_t), 1, bmp_file) != 1)
	{
		printf("[SAVE PROCESS] Failed to Write the Magic Number\n");
		fclose(bmp_file);
		return FAILURE;
	}
	else
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[SAVE PROCESS] The Magic Number is Written\n");		
		#endif
	}
	
	/*
	 * Calculate the Offset where the Clear Image Data (After the File Header) Starts.
	 */
	const uint32_t offset = sizeof(bmpfile_magic_t) + sizeof(bmpfile_header_t) + bitmap_info_header.header_sz;	
	
	/*
	 * Set the File Header with the File Size, Creator 1, Creator 2 and the Offset of the Bitmap Data.
	 */
	save_bitmap_file_header.filesz = offset + bitmap_info_header.bmp_bytesz;
	save_bitmap_file_header.creator1 = 0;
	save_bitmap_file_header.creator2 = 0;
	save_bitmap_file_header.bmp_offset = offset;
	
	#ifdef DEBUG_MESSAGES_UI
	printf("[SAVE PROCESS] Writing the File Header of the Image File\n");
	#endif

	/*
	 * Write the File Header inside the File.
	 */
	if (fwrite(&save_bitmap_file_header, sizeof(bmpfile_header_t), 1, bmp_file) != 1) 
	{
		printf("[SAVE PROCESS] Failed to Write the File Header\n");		
		fclose(bmp_file);
		return FAILURE;
	}
	else
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[SAVE PROCESS] The File Header is Written\n");		
		#endif
	}	
	
	#ifdef DEBUG_MESSAGES_UI
	printf("[SAVE PROCESS] Writing the Info Header of the Image File\n");
	#endif

	/*
	 * Write the Info Header inside the File.
	 */
	if (fwrite(&bitmap_info_header, sizeof(bitmap_info_header_t), 1, bmp_file) != 1) 
	{
		printf("[SAVE PROCESS] Failed to Write the Info Header\n");		
		fclose(bmp_file);
		return FAILURE;
	}
	else
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[SAVE PROCESS] The Info Header is Written\n");		
		#endif
	}		
		
	
	#ifdef DEBUG_MESSAGES_UI
	printf("[SAVE PROCESS] Moving the File Pointer at the Beginning of the Bitmap Data\n");	
	#endif
	
	/*
	 * Move the File Pointer at the Beginning of the Bitmap Data;
	 */	 
	if (fseek(bmp_file, offset, SEEK_SET))
	{
		printf("[SAVE PROCESS] Failed to Move the File Pointer at the Beginning of the Bitmap Data\n");
		fclose(bmp_file);
		return FAILURE;
	}
	
	/*
	 * Calculate the Possible Padding that Might be Found at the end of an Image Row.
	 */	
	pad = (4 - (((bitmap_info_header.bitspp / 8) * bitmap_info_header.width) % 4)) % 4;

	#ifdef DEBUG_MESSAGES_UI
	printf("Writing the Bitmap Data\n");
	#endif
	
	/*
	 * Loop for the Number of Image Rows
	 */	
	for(i=0; i < bitmap_info_header.height; i++) 
	{
		/*
		 * Loop for the Number of Image Columns (The Pixels of each Row)
		 */
		for(j=0; j < bitmap_info_header.width; j++) 
		{
			/*
			 * Loop 3 Times for each Pixel (3 Bytes Per Pixel / bitspp = 24)
			 */
			for(k=0; k<(bitmap_info_header.bitspp / 8); k++)
			{
				/*
				 * Get the Current Byte (1 of 3) of the Current Pixel which is Stored in the (j * 4) + k + (bitmap_info_header.width * i * 4) Offset 
				 * of the Kernel Memory (u8_post_process_kernel_address).
				 */
				current_byte = (uint8_t) u8_post_process_kernel_address[(j * 4) + k + (bitmap_info_header.width * i * 4)];

				/*
				 * Write the Current Byte Inside the Bitmap Data Offset of the File.
				 */
				if (fwrite(&current_byte, sizeof(uint8_t), 1, bmp_file) != 1)
				{
					printf("Failed to Write the Bitmap Data\n");
					fclose(bmp_file);
					return true;
				}
			}
		}

		/*
		 * If Any Padding is Necessary then Add it to the End of the Row.
		 */
		current_byte = 0;
		
		/*
		 * Loop for the Number of Padding Bytes and
		 */
		for(j=0; j < pad; j++)
			if (fwrite(&current_byte, sizeof(uint8_t), 1, bmp_file) != 1) 
			{
				fclose(bmp_file);
				return true;
			}
	}

	/*
	 * Close the Save Bitmap File.
	 */
	fclose(bmp_file);
	
	#ifdef DEBUG_MESSAGES_UI
	printf("Image is Saved\n");	
	#endif	
	
	return SUCCESS;
}


/* OK
 * convert_cycles_2_ns()
 * 
 * Used to Convert Cycles to Nano Seconds.
 * The Global Clock Counter of the AXI Performance Monitors of the FPGA Use two 32 Bit Registers (Lower and Upper) to Store the 
 * Clock Counting in Order to Support Long Cycle Counts.
 * The convert_cycles_2_ns() Combines Lower and Upper Register Values to Convert the Total Cycles to Nano Seconds.
 * 
 * The Userspace Application or the Driver Make One 64 Bit Read from the AXI Performance Monitor Unit to Get the Values of the Upper and Lower 32 Bit Register with a Single Transaction.
 * The First Register Holds the Upper Part of the Cycle Counting while the Second Register Holds the Lower Part of the Cycle Counting.
 * As a Result when we Make a 64 Bit Read we Get the Two Registers at Once which Means that the 32 LSBs of the Read Value Hold the Upper Cycle Counting and the 32 MSBs Hold the Lower Cycle Counting.
 * The convert_cycles_2_ns() Function Shifts the Upper Cycle Counting from the LSBs to Become the MSBs and the Lower Cycle Counting from the MSBs to Become the LSBs.
 * Then the Cycles are Converted to Nano Seconds.
 */
uint64_t convert_cycles_2_ns(uint64_t cycles)
{
	uint64_t gcc_u;
	uint64_t gcc_l;
	uint64_t nano_seconds;
	
	/*
	 * Assign the gcc_u Variable with the 32 LSBs
	 */
	gcc_u = cycles & 0x00000000FFFFFFFF;
	
	/*
	 * Assign the gcc_l Variable with the 32 MSBs
	 */	
	gcc_l = cycles & 0xFFFFFFFF00000000;
	
	/*
	 * Shift Right the gcc_l Variable so that the MSBs Become LSBs
	 */
	gcc_l = gcc_l >> 32;
	
	/*
	 * Shift Left the gcc_l Variable so that the LSBs Become MSBs
	 */	
	gcc_u = gcc_u << 32;

	/*
	 * Add the gcc_l with the gcc_u to Get the Correct Total Number of Cycles.
	 * Then Multiply the Add Value by 8 to Get the Nanoseconds where 8 is the Number of Nanoseconds for each Cycle at 125MHz.
	 */
	nano_seconds = 	(gcc_l + gcc_u) * 8;
	
	return nano_seconds;
	
}


/* OK
 * file_size()
 * 
 * Used to Get the Size of a File.
 * It is Useful when We Need to Know if a File is Empty or Not.
 */
int file_size(FILE *file)
{
    int previous_size;
    int new_size;
    
    /*
     * Use ftell() which Returns the Current Value of the Position Indicator which Should Be at the Beginning of the File.
     * Store the Return Value to the previous_size Variable so that it Can Later be Used to Return the Position Indicator at the Beginning of the File.
     */
    previous_size = ftell(file);
    
    /*
     * Use fseek() to Move the Position Indicator at the End (SEEK_END) of the File.
     */
    fseek(file, 0L, SEEK_END);
    
    /*
     * Use ftell() which Returns the Current Value of the Position Indicator which Should Be Now at the End of the File.
     * Store the Return Value to the new_size Variable so that it Can Later be Used to Know the Size of the File.
     */    
    new_size = ftell(file);
    
    /*
     * Use fseek() to Move the Position Indicator at the Beginning of the File According to the previous_size Variable.
     */    
    fseek(file, previous_size, SEEK_SET);
    
    /*
     * Return the new_size Variable which Indicates the Size of the File.
     */
    return new_size;
}


/* OK
 * print_save_metrics()
 * 
 * Used to Save the Metrics of each Iteration of each Test.
 * The Metrics from the Hardware Schedulers, the Driver and the User Application are Collected in the
 * Shared Kernel Memory which is Accessed by the shared_repo_kernel_address Pointer.
 * The shared_repo_kernel_address Pointer Provides Access to the Collected Metrics which are Stored as Structure Fields  of Type struct shared_repository_process.
 * The Metrics are Organized and Written as Element Nodes of a .xml File.
 * 
 * ----------------------------------------------
 * The Structure of the .xml Nodes is as Follows:
 * 
 * <Process>                                                            
 *      <Iteration> ### </Iteration>                                    
 *      <Image_Segments> ### </Image_Segments>                          
 *      <Preparation_Time_Start> ### </Preparation_Time_Start>          
 *      <Preparation_Time_End> ### </Preparation_Time_End>              
 *      <Load_Time_Start> ### </Load_Time_Start>                        
 *      <Load_Time_End> ### </Load_Time_End>                            
 *      <Total_Time_Start> ### </Total_Time_Start>                      
 *      <Sleep_Time_Start> ### </Sleep_Time_Start>                      
 *      <Sleep_Time_End> ### </Sleep_Time_End>                          
 *      <Save_Time_Start> ### </Save_Time_Start>                        
 *      <Save_Time_End> ### </Save_Time_End>                            
 *      <Total_Time_End> ### </Total_Time_End>                          
 *      <Segment>                                                       
 *           <Segment_Number> ### </Segment_Number>
 *           <Initiator> ### </Initiator>
 *           <Read_Transactions> ### </Read_Transactions>
 *           <Read_Bytes> ### </Read_Bytes>
 *           <Write_Transactions> ### </Write_Transactions>
 *           <Write_Bytes> ### </Write_Bytes>
 *           <Stream_Packets> ### </Stream_Packets>
 *           <Stream_Bytes> ### </Stream_Bytes>
 *           <Process_Cycles> ### </Process_Cycles> 
 *           <Set_Pages_Overhead_Time_Start> ### </Set_Pages_Overhead_Time_Start>
 *           <Set_Pages_Overhead_Time_End> ### </Set_Pages_Overhead_Time_End>\
 *           <Unmap_Pages_Overhead_Time_Start> ### </Unmap_Pages_Overhead_Time_Start>
 *           <Unmap_Pages_Overhead_Time_End> ### </Unmap_Pages_Overhead_Time_End>
 *           <CDMA_Fetch_Time_Start> ### </CDMA_Fetch_Time_Start>
 *           <CDMA_Fetch_Time_End> ### </CDMA_Fetch_Time_End>
 *           <Process_Time_Start> ### </Process_Time_Start>
 *           <Process_Time_End> ### </Process_Time_End>
 *           <CDMA_Send_Time_Start> ### </CDMA_Send_Time_Start>
 *           <CDMA_Send_Time_End> ### </CDMA_Send_Time_End>
 *      </Segment>
 * </Process>
 * 
 * -------------------------
 * Element Node Explanation:
 * 
 * Process --> The Process ID of the Process that those Metrics Refer to
 * Iteration --> The Current Iteration of the Current Thread
 * Image_Segments --> The Number of Segments that the Image was Splitted for Parallel Acceleration
 * Preparation_Time_Start --> The Starting Point for the Preparation Needed before the Acceleration Procedure
 * Preparation_Time_End --> The Ending Point for the Preparation Needed before the Acceleration Procedure
 * Load_Time_Start --> The Starting Point of the Duration for Loading the Image from the Storage Device
 * Load_Time_End --> The Ending Point of the Duration for Loading the Image from the Storage Device
 * Total_Time_Start --> The Starting Point of the Total Time Required for the Acceleration Procedure
 * Sleep_Time_Start --> The Starting Point of the Duration that the Thread Possibly Stayed in Sleep State
 * Sleep_Time_End --> The Ending Point of the Duration that the Thread Possibly Stayed in Sleep State
 * Save_Time_Start --> The Starting Point of the Duration for Saving the Image to the Storage Device
 * Save_Time_End --> The Ending Point of the Duration for Saving the Image to the Storage Device
 * Total_Time_End --> The Ending Point of the Total Time Required for the Acceleration Procedure
 * Segment --> The Segment Element Node Carries Metrics Regarding the Current Segment that Was Accelerated.There Can be as many as 6 Segment Nodes
 * Segment_Number --> The Current Segment Number Among the Rest that the Image Might was Splitted
 * Initiator --> The Acceleration Group that Accelerated the Current Image Segment
 * Read_Transactions --> The Number of Read Transactions that Took Place while the DMA was Reading the Current Image Segment from the Kernel Memory
 * Read_Bytes --> The Number of Read Bytes that were Transferred while the DMA was Reading the Current Image Segment from the Kernel Memory
 * Write_Transactions --> The Number of Write Transactions that Took Place while the DMA was Writing the Current Processed Image Segment to the Kernel Memory
 * Write_Bytes --> The Number of Write Bytes that were Transferred while the DMA was Writing the Current Processed Image Segment to the Kernel Memory
 * Stream_Packets --> The Number of Stream Packets that where Transferred through the Sobel Accelerator's Stream Interface
 * Stream_Bytes --> The Number of Stream Bytes that where Transferred through the Sobel Accelerator's Stream Interface
 * Process_Cycles --> The Number of Clock Cycles that were Required to Complete the Acceleration (DMA Read -> Sobel Acceleration -> DMA Write)
 * Set_Pages_Overhead_Time_Start --> The Starting Point of the Duration that the Driver Required to Create the Scatter/Gather List if the Acceleration Group SG was Requested
 * Set_Pages_Overhead_Time_End --> The Ending Point of the Duration that the Driver Required to Create the Scatter/Gather List if the Acceleration Group SG was Requested
 * Unmap_Pages_Overhead_Time_Start -->  The Starting Point of the Duration that the Driver Required to Unmap the Pages that were Previously Mapped when Creating the Scatter/Gather List
 * Unmap_Pages_Overhead_Time_End -->  The Ending Point of the Duration that the Driver Required to Unmap the Pages that were Previously Mapped when Creating the Scatter/Gather List
 * CDMA_Fetch_Time_Start --> The Starting Point of the Duration that the CDMA Fetch Peripheral Required to Fetch Image Data to the FPGA DDR3 Memory (Applicable for the Initiators: AGI0, AGI1, AGI2, AGI3)
 * CDMA_Fetch_Time_End --> The Ending Point of the Duration that the CDMA Fetch Peripheral Required to Fetch Image Data to the FPGA DDR3 Memory (Applicable for the Initiators: AGI0, AGI1, AGI2, AGI3)
 * Process_Time_Start --> The Starting Point of the Duration that the Image Segment Acceleration Required to Complete
 * Process_Time_End --> The Ending Point of the Duration that the Image Segment Acceleration Required to Complete
 * CDMA_Send_Time_Start --> The Starting Point of the Duration that the CDMA Send Peripheral Required to Send Processed Image Data from the FPGA DDR3 Memory (Applicable for the Initiators: AGI0, AGI1, AGI2, AGI3)
 * CDMA_Send_Time_End --> The Ending Point of the Duration that the CDMA Send Peripheral Required to Send Processed Image Data from the FPGA DDR3 Memory (Applicable for the Initiators: AGI0, AGI1, AGI2, AGI3)
 * 
 */
int print_save_metrics(struct shared_repository_process *shared_repo_kernel_address, int used_accelerator, unsigned int tid, int global_repeat)
{
	
	char file_name[100];
	FILE *metrics_summary_file;
	
	uint64_t gcc_u;
	uint64_t gcc_l;
	uint64_t time_counted;
	
	struct stat file_statistics;

	int segments = 0;
	int segment_count = 0;
	int repeat;
		
	/*
	 * Depending on the Acceleration Policy an Image Can be Processed by Several Acceleration Groups, that is, an Image
	 * Can be Splitted in Several Segments.
	 * The Following if Statements Check which Acceleration Groups Took Part in Accelerating the Current Image which is Equal to 
	 * the Number of Segments that the Image was Splitted.
	 */
	if((used_accelerator & ACCELERATOR_DIRECT_0_OCCUPIED) == ACCELERATOR_DIRECT_0_OCCUPIED)
	{
		segments++;
	}
	if((used_accelerator & ACCELERATOR_DIRECT_1_OCCUPIED) == ACCELERATOR_DIRECT_1_OCCUPIED)
	{
		segments++;
	}
	if((used_accelerator & ACCELERATOR_INDIRECT_0_OCCUPIED) == ACCELERATOR_INDIRECT_0_OCCUPIED)
	{
		segments++;
	}
	if((used_accelerator & ACCELERATOR_INDIRECT_1_OCCUPIED) == ACCELERATOR_INDIRECT_1_OCCUPIED)
	{
		segments++;
	}	
	if((used_accelerator & ACCELERATOR_INDIRECT_2_OCCUPIED) == ACCELERATOR_INDIRECT_2_OCCUPIED)
	{
		segments++;
	}	
	if((used_accelerator & ACCELERATOR_INDIRECT_3_OCCUPIED) == ACCELERATOR_INDIRECT_3_OCCUPIED)
	{
		segments++;
	}
	if((used_accelerator & ACCELERATOR_SG_OCCUPIED) == ACCELERATOR_SG_OCCUPIED)
	{
		segments++;
	}
	
	/*
	 * Use sprintf() to Create a String that Represents the Path and Name of the Metrics .xml File.
	 * The Arithmetic Value of the renamer_value Variable is Included in the File Name to Ensure that each Test Iteration
	 * Creates a New .xml File which is Unique Among the Rest .xml Files.
	 */
	sprintf(file_name,"Results/Metrics_Summary_%d.xml", renamer_value);

	/*
	 * Open Again the .xml File to Write the Collected Metrics.
	 */
	metrics_summary_file = fopen(file_name, "a");
	
	/*
	 * The .xml File is Possibly Accessed by Many Threads.
	 * flock() is Used to Ensure that Only One Thread Write at this .xml File at the Moment.
	 */
	flock(fileno(metrics_summary_file), LOCK_EX);

	/*
	 * If the Metrics .xml File was Found or Created then Start Writing the Metrics Data.
	 */
	if (metrics_summary_file != NULL)
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("File %s Opened\n", file_name);
		#endif
		 
		 
		/*
		 * Write the Open Tag of the Process Element.
		 * The Process Element, also, Includes the "ID" Attribute which is the Process ID of the Current Thread.
		 */
		fprintf(metrics_summary_file,"	<Process ID=\"%d\">\n", tid);
		
		/*
		 * The Following Element Nodes Refer to Metrics of the Acceleration Procedure for the whole Image
		 */		
		
		/*
		 * Write the Iteration Element Node.
		 * The global_repeat is the Function Argument Given as the Current Iteration of the Acceleration that the Thread Requested
		 */
		fprintf(metrics_summary_file,"		<Iteration>%d</Iteration>\n", global_repeat);
		
		/*
		 * Write the Image_Segments Element Node.
		 * The segments was Calculated Previously by Incrementing the Number of Acceleration Groups that Took Part in Accelerating the Current Image
		 */		
		fprintf(metrics_summary_file,"		<Image_Segments>%d</Image_Segments>\n", segments);
		
		/*
		 * Write the Preparation_Time_Start Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the preparation_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */			
		fprintf(metrics_summary_file,"		<Preparation_Time_Start>%lld</Preparation_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.preparation_time_start));
		
		/*
		 * Write the Preparation_Time_End Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the preparation_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */			
		fprintf(metrics_summary_file,"		<Preparation_Time_End>%lld</Preparation_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.preparation_time_end));		
		
		/*
		 * Write the Load_Time_Start Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the load_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */
		fprintf(metrics_summary_file,"		<Load_Time_Start>%lld</Load_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.load_time_start));
		
		/*
		 * Write the Load_Time_End Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the load_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */		
		fprintf(metrics_summary_file,"		<Load_Time_End>%lld</Load_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.load_time_end));	
		
		/*
		 * Write the Total_Time_Start Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the total_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */				
		fprintf(metrics_summary_file,"		<Total_Time_Start>%lld</Total_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.total_time_start));
		
		/*
		 * Write the Sleep_Time_Start Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the sleep_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */			
		fprintf(metrics_summary_file,"		<Sleep_Time_Start>%lld</Sleep_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.sleep_time_start));
		
		/*
		 * Write the Sleep_Time_End Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the sleep_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */				
		fprintf(metrics_summary_file,"		<Sleep_Time_End>%lld</Sleep_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.sleep_time_end));			
		
		/*
		 * Write the Save_Time_Start Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the save_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */			
		fprintf(metrics_summary_file,"		<Save_Time_Start>%lld</Save_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.save_time_start));
		
		/*
		 * Write the Save_Time_End Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the save_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */				
		fprintf(metrics_summary_file,"		<Save_Time_End>%lld</Save_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.save_time_end));
		
		/*
		 * Write the Total_Time_End Element Node Along with its Value (nanoseconds).
		 * The Node Value was Calculated by Converting the Cycles Value Found in the total_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
		 */			
		fprintf(metrics_summary_file,"		<Total_Time_End>%lld</Total_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.total_time_end));		
		
		/*
		 * The Following Element Nodes Refer to Metrics of the Acceleration Procedure for each Image Segment that was Accelerated.
		 * There are 7 if Statements for each of the 7 Acceleration Groups.
		 * For each Acceleration Group that Took Part in the Acceleration Procedure a new Segment Element Node is Added to the Process Parent Node.
		 * 
		 * Comments will be Provided Only for Creating the Segment of the Acceleration Group Direct 0.
		 * The Procedure is Exactly the Same for the Rest of the Acceleration Groups.
		 */		
	 	
		
		/*
		 * Acceleration Group Direct 0 Segment
		 */
		 		
		if((used_accelerator & ACCELERATOR_DIRECT_0_OCCUPIED) == ACCELERATOR_DIRECT_0_OCCUPIED)
		{
			/*
			 * Write the Open Tag of the Segment Element.
			 */
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			/*
			 * Write the Segment_Number Element Node.
			 * The Segment_Number Value is Given by the segment_count Variable which Holds a Value that Increments for each New Segment that is Added to the Process Parent Node
			 */			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			
			/*
			 * Write the Initiator Element Node which is Acceleration Group Direct 0.
			 */	
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group Direct 0</Initiator>\n");
			
			/*
			 * Write the Read_Transactions Element Node Along with its Value.
			 * The Node Value was Found in the apm_read_transactions Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */				
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agd0.apm_read_transactions);
			
			/*
			 * Write the Read_Bytes Element Node Along with its Value.
			 * The Node Value was Found in the apm_read_bytes Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */			
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agd0.apm_read_bytes);
			
			/*
			 * Write the Write_Transactions Element Node Along with its Value.
			 * The Node Value was Found in the apm_write_transactions Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */			
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agd0.apm_write_transactions);
			
			/*
			 * Write the Write_Bytes Element Node Along with its Value.
			 * The Node Value was Found in the apm_write_bytes Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */				
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agd0.apm_write_bytes);
			
			/*
			 * Write the Stream_Packets Element Node Along with its Value.
			 * The Node Value was Found in the apm_packets Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */				
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agd0.apm_packets);
			
			/*
			 * Write the Stream_Bytes Element Node Along with its Value.
			 * The Node Value was Found in the apm_bytes Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */				
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agd0.apm_bytes);
			
			/*
			 * Write the Process_Cycles Element Node Along with its Value.
			 * The Node Value was Found in the apm_gcc_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory
			 */				
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agd0.apm_gcc_l);

			/*
			 * Write the Set_Pages_Overhead_Time_Start Element Node Along with its Value (nanoseconds).
			 * The Node Value was Calculated by Converting the Cycles Value Found in the set_pages_overhead_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
			 */	
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));

			/*
			 * Write the Set_Pages_Overhead_Time_End Element Node Along with its Value (nanoseconds).
			 * The Node Value was Calculated by Converting the Cycles Value Found in the set_pages_overhead_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
			 */	
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	


			/*
			 * Write the Unmap_Pages_Overhead_Time_Start Element Node Along with its Value (nanoseconds).
			 * The Node Value was Calculated by Converting the Cycles Value Found in the unmap_pages_overhead_time_start Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
			 */				
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));

			/*
			 * Write the Unmap_Pages_Overhead_Time_End Element Node Along with its Value (nanoseconds).
			 * The Node Value was Calculated by Converting the Cycles Value Found in the unmap_pages_overhead_time_end Field of the shared_repo_kernel_address->process_metrics Structure of the Shared Kernel Memory
			 */	
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			
			
			/*
			 * Get the cdma_fetch_time_start_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_fetch_time_start_l Field is Partially the CDMA Fetch Time Start Value which was Found at the APM Lower Global Clock Counter Register.
			 */
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_fetch_time_start_l;
			
			/*
			 * Get the cdma_fetch_time_start_u Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_fetch_time_start_u Field is Partially the CDMA Fetch Time Start Value which was Found at the APM Upper Global Clock Counter Register.
			 */			
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_fetch_time_start_u;
			
			/*
			 * Shift Left by 32 the gcc_u Value.
			 */
			gcc_u = gcc_u << 32;
			
			/*
			 * Calculate the Correct CDMA Fetch Time Start Value from Cycles to Nanoseconds and Write it to the CDMA_Fetch_Time_Start Element Node.
			 */
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			
			
			
			/*
			 * Get the cdma_fetch_time_end_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_fetch_time_end_l Field is Partially the CDMA Fetch Time End Value which was Found at the APM Lower Global Clock Counter Register.
			 */			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_fetch_time_end_l;
			
			/*
			 * Get the cdma_fetch_time_end_u Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_fetch_time_end_u Field is Partially the CDMA Fetch Time End Value which was Found at the APM Upper Global Clock Counter Register.
			 */				
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_fetch_time_end_u;
			
			/*
			 * Shift Left by 32 the gcc_u Value.
			 */			
			gcc_u = gcc_u << 32;
					
			/*
			 * Calculate the Correct CDMA Fetch Time End Value from Cycles to Nanoseconds and Write it to the CDMA_Fetch_Time_End Element Node.
			 */					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);



			/*
			 * Get the dma_accel_time_start_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The dma_accel_time_start_l Field is Partially the Acceleration Time Start Value which was Found at the APM Lower Global Clock Counter Register.
			 */	
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.dma_accel_time_start_l;
			
			/*
			 * Get the dma_accel_time_start_u Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The dma_accel_time_start_u Field is Partially the Acceleration Time Start Value which was Found at the APM Upper Global Clock Counter Register.
			 */				
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.dma_accel_time_start_u;
			
			/*
			 * Shift Left by 32 the gcc_u Value.
			 */				
			gcc_u = gcc_u << 32;
			
			/*
			 * Calculate the Correct Acceleration Time Start Value from Cycles to Nanoseconds and Write it to the Process_Time_Start Element Node.
			 */			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			
			
			
			/*
			 * Get the dma_accel_time_end_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The dma_accel_time_end_l Field is Partially the Acceleration Time End Value which was Found at the APM Lower Global Clock Counter Register.
			 */				
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.dma_accel_time_end_l;
			
			/*
			 * Get the dma_accel_time_end_u Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The dma_accel_time_end_u Field is Partially the Acceleration Time End Value which was Found at the APM Upper Global Clock Counter Register.
			 */				
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.dma_accel_time_end_u;
			
			/*
			 * Shift Left by 32 the gcc_u Value.
			 */			
			gcc_u = gcc_u << 32;
	
			/*
			 * Calculate the Correct Acceleration Time End Value from Cycles to Nanoseconds and Write it to the Process_Time_End Element Node.
			 */
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
		
		
		
			
			/*
			 * Get the cdma_send_time_start_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_send_time_start_l Field is Partially the CDMA Send Time Start Value which was Found at the APM Lower Global Clock Counter Register.
			 */
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_send_time_start_l;
			
			/*
			 * Get the cdma_send_time_start_u Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_send_time_start_u Field is Partially the CDMA Send Time Start Value which was Found at the APM Upper Global Clock Counter Register.
			 */				
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_send_time_start_u;
			
			/*
			 * Shift Left by 32 the gcc_u Value.
			 */		
			gcc_u = gcc_u << 32;
			
			/*
			 * Calculate the Correct CDMA Send Time Start Value from Cycles to Nanoseconds and Write it to the CDMA_Send_Time_Start Element Node.
			 */				
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
		
		
		
		
			/*
			 * Get the cdma_send_time_end_l Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_send_time_end_l Field is Partially the CDMA Send Time End Value which was Found at the APM Lower Global Clock Counter Register.
			 */
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_send_time_end_l;
			
			/*
			 * Get the cdma_send_time_end_u Field of the shared_repo_kernel_address->process_metrics.agd0 Structure of the Shared Kernel Memory.
			 * The cdma_send_time_end_u Field is Partially the CDMA Send Time End Value which was Found at the APM Upper Global Clock Counter Register.
			 */				
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd0.cdma_send_time_end_u;
			
			/*
			 * Shift Left by 32 the gcc_u Value.
			 */					
			gcc_u = gcc_u << 32;					
					
			/*
			 * Calculate the Correct CDMA Send Time End Value from Cycles to Nanoseconds and Write it to the CDMA_Send_Time_End Element Node.
			 */						
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			
			
			
			/*
			 * Write the Close Tag of the Segment Element Node.
			 */
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			/*
			 * Increment the segment_count Variable for the Next Segment that Might be Present.
			 */
			segment_count++;
	
		}
	
	
		/*
		 * Acceleration Group Direct 1 Segment
		 */
		
		if((used_accelerator & ACCELERATOR_DIRECT_1_OCCUPIED) == ACCELERATOR_DIRECT_1_OCCUPIED)
		{
		
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group Direct 1</Initiator>\n");
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agd1.apm_read_transactions);
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agd1.apm_read_bytes);
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agd1.apm_write_transactions);
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agd1.apm_write_bytes);
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agd1.apm_packets);
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agd1.apm_bytes);
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agd1.apm_gcc_l);

			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	
			
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_fetch_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_fetch_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_fetch_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_fetch_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.dma_accel_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.dma_accel_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.dma_accel_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.dma_accel_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
			

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_send_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_send_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_send_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agd1.cdma_send_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			segment_count++;
	
		}	
	
	
		/*
		 * Acceleration Group Indirect 0 Segment
		 */
		
		if((used_accelerator & ACCELERATOR_INDIRECT_0_OCCUPIED) == ACCELERATOR_INDIRECT_0_OCCUPIED)
		{
		
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group Indirect 0</Initiator>\n");
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agi0.apm_read_transactions);
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agi0.apm_read_bytes);
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agi0.apm_write_transactions);
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agi0.apm_write_bytes);
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agi0.apm_packets);
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agi0.apm_bytes);
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agi0.apm_gcc_l);

			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	
			
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_fetch_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_fetch_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_fetch_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_fetch_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.dma_accel_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.dma_accel_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.dma_accel_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.dma_accel_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
			

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_send_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_send_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_send_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi0.cdma_send_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			segment_count++;
	
		}
			
			
		/*
		 * Acceleration Group Indirect 1 Segment
		 */
		
		if((used_accelerator & ACCELERATOR_INDIRECT_1_OCCUPIED) == ACCELERATOR_INDIRECT_1_OCCUPIED)
		{
		
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group Indirect 1</Initiator>\n");
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agi1.apm_read_transactions);
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agi1.apm_read_bytes);
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agi1.apm_write_transactions);
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agi1.apm_write_bytes);
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agi1.apm_packets);
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agi1.apm_bytes);
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agi1.apm_gcc_l);

			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	
			
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_fetch_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_fetch_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_fetch_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_fetch_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.dma_accel_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.dma_accel_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.dma_accel_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.dma_accel_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
			

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_send_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_send_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_send_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi1.cdma_send_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			segment_count++;
	
		}
		
		
		/*
		 * Acceleration Group Indirect 2 Segment
		 */
		
		if((used_accelerator & ACCELERATOR_INDIRECT_2_OCCUPIED) == ACCELERATOR_INDIRECT_2_OCCUPIED)
		{
		
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group Indirect 2</Initiator>\n");
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agi2.apm_read_transactions);
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agi2.apm_read_bytes);
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agi2.apm_write_transactions);
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agi2.apm_write_bytes);
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agi2.apm_packets);
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agi2.apm_bytes);
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agi2.apm_gcc_l);

			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	
			
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_fetch_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_fetch_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_fetch_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_fetch_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.dma_accel_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.dma_accel_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.dma_accel_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.dma_accel_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
			

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_send_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_send_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_send_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi2.cdma_send_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			segment_count++;
	
		}
		
		
		
		/*
		 * Acceleration Group Indirect 3 Segment
		 */
		
		if((used_accelerator & ACCELERATOR_INDIRECT_3_OCCUPIED) == ACCELERATOR_INDIRECT_3_OCCUPIED)
		{
		
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group Indirect 3</Initiator>\n");
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agi3.apm_read_transactions);
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agi3.apm_read_bytes);
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agi3.apm_write_transactions);
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agi3.apm_write_bytes);
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agi3.apm_packets);
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agi3.apm_bytes);
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agi3.apm_gcc_l);

			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	
			
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_fetch_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_fetch_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_fetch_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_fetch_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.dma_accel_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.dma_accel_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.dma_accel_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.dma_accel_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
			

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_send_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_send_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_send_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agi3.cdma_send_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			segment_count++;
	
		}
				
		
		/*
		 * Acceleration Group Scatter/Gather Segment
		 */
		
		if((used_accelerator & ACCELERATOR_SG_OCCUPIED) == ACCELERATOR_SG_OCCUPIED)
		{
		
			fprintf(metrics_summary_file,"		<Segment>\n");
			
			fprintf(metrics_summary_file,"			<Segment_Number>%d</Segment_Number>\n", segment_count);
			fprintf(metrics_summary_file,"			<Initiator>Acceleration Group SG</Initiator>\n");
			fprintf(metrics_summary_file,"			<Read_Transactions>%d</Read_Transactions>\n", shared_repo_kernel_address->process_metrics.agsg.apm_read_transactions);
			fprintf(metrics_summary_file,"			<Read_Bytes>%d</Read_Bytes>\n", shared_repo_kernel_address->process_metrics.agsg.apm_read_bytes);
			fprintf(metrics_summary_file,"			<Write_Transactions>%d</Write_Transactions>\n", shared_repo_kernel_address->process_metrics.agsg.apm_write_transactions);
			fprintf(metrics_summary_file,"			<Write_Bytes>%d</Write_Bytes>\n", shared_repo_kernel_address->process_metrics.agsg.apm_write_bytes);
			fprintf(metrics_summary_file,"			<Stream_Packets>%d</Stream_Packets>\n", shared_repo_kernel_address->process_metrics.agsg.apm_packets);
			fprintf(metrics_summary_file,"			<Stream_Bytes>%d</Stream_Bytes>\n", shared_repo_kernel_address->process_metrics.agsg.apm_bytes);
			fprintf(metrics_summary_file,"			<Process_Cycles>%d</Process_Cycles>\n", shared_repo_kernel_address->process_metrics.agsg.apm_gcc_l);

			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_Start>%lld</Set_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Set_Pages_Overhead_Time_End>%lld</Set_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end));	
			
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_Start>%lld</Unmap_Pages_Overhead_Time_Start>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start));
			fprintf(metrics_summary_file,"			<Unmap_Pages_Overhead_Time_End>%lld</Unmap_Pages_Overhead_Time_End>\n", convert_cycles_2_ns(shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end));		
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_fetch_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_fetch_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_Start>%lld</CDMA_Fetch_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_fetch_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_fetch_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Fetch_Time_End>%lld</CDMA_Fetch_Time_End>\n", (gcc_l + gcc_u) * 8);

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.dma_accel_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.dma_accel_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<Process_Time_Start>%lld</Process_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.dma_accel_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.dma_accel_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<Process_Time_End>%lld</Process_Time_End>\n", (gcc_l + gcc_u) * 8);
			

			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_send_time_start_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_send_time_start_u;
			gcc_u = gcc_u << 32;
			
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_Start>%lld</CDMA_Send_Time_Start>\n", (gcc_l + gcc_u) * 8);
			
			gcc_l = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_send_time_end_l;
			gcc_u = (uint64_t)shared_repo_kernel_address->process_metrics.agsg.cdma_send_time_end_u;
			gcc_u = gcc_u << 32;
					
			fprintf(metrics_summary_file,"			<CDMA_Send_Time_End>%lld</CDMA_Send_Time_End>\n", (gcc_l + gcc_u) * 8);		
			
			fprintf(metrics_summary_file,"		</Segment>\n\n\n");		
			
			segment_count++;
	
		}		
	
		/*
		 * Write to the Metrics .xml the Close Tag of the Process Element.
		 */
		fprintf(metrics_summary_file,"	</Process>\n\n\n");

		/*
		 * Close the Metrics .xml File.
		 */
		fclose(metrics_summary_file);
		
		/*
		 * Unlock the Metrics .xml File so that the Rest of the Threads Can Use It.
		 */
		flock(fileno(metrics_summary_file), LOCK_UN);
		
		metrics_summary_file = NULL;	
		
	}
		
	else
	{
		printf("Could not Open the File %s\n", file_name);
	}
	
	
}


/* OK
 * set_save_accelerator()
 * 
 * Used to Create a String that Represents the Path and Name of the .bmp File where the Accelerated Image is Going to be Stored.
 * The Name of the new .bmp File Carries the Following Information Regarding the Processed Image:
 *  -> pid Followed by the Process ID Number.
 *  -> iter Followed by the Current Iteration Number.
 *  -> d0 Followed by Value Zero or One Depending on whether the Accelereration Group Direct 0 Was Used or Not for Accelerating the Current Image
 *  -> d1 Followed by Value Zero or One Depending on whether the Accelereration Group Direct 1 Was Used or Not for Accelerating the Current Image
 *  -> i0 Followed by Value Zero or One Depending on whether the Accelereration Group Indirect 0 Was Used or Not for Accelerating the Current Image
 *  -> i1 Followed by Value Zero or One Depending on whether the Accelereration Group Indirect 1 Was Used or Not for Accelerating the Current Image
 *  -> i2 Followed by Value Zero or One Depending on whether the Accelereration Group Indirect 2 Was Used or Not for Accelerating the Current Image
 *  -> i3 Followed by Value Zero or One Depending on whether the Accelereration Group Indirect 3 Was Used or Not for Accelerating the Current Image
 *  -> sg Followed by Value Zero or One Depending on whether the Accelereration Group Scatter/Gather Was Used or Not for Accelerating the Current Image
 */
int set_save_accelerator(char *save_path_name, int used_accelerator, int tid, int iteration)
{
	int accel_occupied[7];
	int repeat;
	
	/*
	 * Shift Right the used_accelerator Variable to Get the 7 LSB Bits 
	 * that Represent the Status (Used or Not) of each Acceleration Group Respectively
	 */
	for(repeat = 0; repeat < 7; repeat++)
	{
		accel_occupied[repeat] = (used_accelerator >> repeat) & 1;
	}
	
	/*
	 * Create a New String that Carries the Path, Name and Image Info that will be Used to Later Save the Current Processed Image
	 */
	sprintf(save_path_name, "Results/pid_%d_iter_%d_d0%d_d1%d_i0%d_i1%d_i2%d_i3%d_sg%d.bmp", tid, iteration, accel_occupied[0], accel_occupied[1], accel_occupied[2], accel_occupied[3], accel_occupied[4], accel_occupied[5], accel_occupied[6]);
		
		
	return SUCCESS;
}


/* OK
 * pcie_bar_mmap()
 * 
 * Used to Map the PCIe BAR0 and PCIe BAR1 of the PCIe Bridge as Part of the Virtual Address Space of the Userspace Application.
 * PCIe BARs Represent Memories or Address Mappings of a PCIe Endpoint Device.
 * The Host System Can Get Direct Access to the the Peripherals and Memories of the Endpoint Device through the PCIe BARs.
 * 
 * Herein:
 * PCIe BAR0 Represents the AXI Address Space where all the FPGA Peripherals are Mapped.
 * PCIe BAR1 Represents the BRAM Memory of the FPGA which is Used to Store Metrics Information, SG Lists and Synchronization Flags.
 * 
 * At Boot Time the Host System, among others, Enumerates the PCIe BAR0 and PCIe BAR 1 of the PCIe Endpoint Device and Creates
 * the resource0 and resource2 Files at the "/sys/bus/pci/devices/0000:01:00.0/" Path.
 * Those two Files are Used to Map the PCIe BARs Respectively.
 */
int pcie_bar_mmap()
{
	printf("Memory Mapping PCIe BAR Address Space\n");
	
	/*
	 * Open the resource0 File that Represents the PCIe BAR0 of the PCIe Bridge.
	 */
	pcie_bar_0_mmap_file = open("/sys/bus/pci/devices/0000:01:00.0/resource0", O_RDWR);
	
	/*
	 * If the pcie_bar_0_mmap_file Value is Less than Zero then the System Failed to Open the File or the File Does not Exist
	 */
	if ( pcie_bar_0_mmap_file < 0 )  
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Error Opening PCIe BAR 0 MMAP File\n");
		#endif
		return FAILURE;
	}	
	
	/*
	 * Use the mmap() Function to Map the PCIe BAR0 to the Virtual Address Space of the Userspace.
	 * The mmap() Function Returns a 32 Bit Pointer(uint_32_pcie_bar_kernel_address) which Can be Used to Get Direct Access to the AXI Address Space (Peripherals) of the FPGA.
	 */
	uint_32_pcie_bar_kernel_address = (unsigned int *)mmap(0, MMAP_ALLOCATION_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, pcie_bar_0_mmap_file, 0);
	
	/*
	 * Cast the uint_32_pcie_bar_kernel_address Pointer to the 64 Bit uint_64_pcie_bar_kernel_address Pointer.
	 * The uint_64_pcie_bar_kernel_address Can be Used to Make 64 Bit Read/Write Transactions.
	 */
	uint_64_pcie_bar_kernel_address = (uint64_t *)uint_32_pcie_bar_kernel_address;
	
	/*
	 * If the Value of the uint_32_pcie_bar_kernel_address Pointer is Equal with the MAP_FAILED Value it Means that We Failed to Map the PCIe BAR0
	 */
	if (uint_32_pcie_bar_kernel_address == MAP_FAILED)
	{
		printf("Kernel Memory MMAP [FAILURE]\n");
		
		#ifdef DEBUG_MESSAGES_UI
		usleep(2000000);
		#endif
		return FAILURE;
	}	
	else
	{
		printf("PCIe BAR 0 Kernel Memory MMAP [SUCCESS]\n");
		printf("PCIe BAR 0 Kernel Virtual Address is 0x%016lX\n", (unsigned long)uint_64_pcie_bar_kernel_address);	
			
	}
	
	/*
	 * Open the resource2 File that Represents the PCIe BAR1 of the PCIe Bridge.
	 */
	pcie_bar_1_mmap_file = open("/sys/bus/pci/devices/0000:01:00.0/resource2", O_RDWR);
	
	/*
	 * If the pcie_bar_1_mmap_file Value is Less than Zero then the System Failed to Open the File or the File Does not Exist
	 */	
	if ( pcie_bar_1_mmap_file < 0 )  
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Error Opening PCIe BAR 1 MMAP File\n");
		#endif
		return FAILURE;
	}	
	
	/*
	 * Use the mmap() Function to Map the PCIe BAR1 to the Virtual Address Space of the Userspace.
	 * The mmap() Function Returns a n Unsigned Int Pointer(uint_shared_kernel_address) which Can be Used to Get Direct Access to the FPGA BRAM.
	 */
	uint_shared_kernel_address = (unsigned int *)mmap(0, 128 * KBYTE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, pcie_bar_1_mmap_file, 0);
	
	/*
	 * Cast the uint_shared_kernel_address Pointer to the struct shared_repository shared_kernel_address Pointer.
	 * The shared_kernel_address Pointer Can be Used to Make Read/Write Transactions in a Manner of Accessing the Fields of the struct shared_repository.
	 */	
	shared_kernel_address = (struct shared_repository *)uint_shared_kernel_address;
	
	/*
	 * If the Value of the uint_shared_kernel_address Pointer is Equal with the MAP_FAILED Value it Means that We Failed to Map the PCIe BAR1
	 */
	if (uint_shared_kernel_address == MAP_FAILED)
	{
		printf("Kernel Memory MMAP [FAILURE]\n");
		
		#ifdef DEBUG_MESSAGES_UI
		usleep(2000000);
		#endif
		return FAILURE;
	}	
	else
	{
		printf("PCIe BAR 1 Kernel Memory MMAP [SUCCESS]\n");
		printf("PCIe BAR 1 Kernel Virtual Address is 0x%016lX\n", (unsigned long)shared_kernel_address);	
			
	}	
	

	return SUCCESS;
}


/* OK
 * shared_repo_mmap()
 * 
 * This Function is Used to Map a Memory Allocation of the Kernel Space so that it Can Be Directly Accessed by the User Application.
 * This Memory Allocation is Shared Between the Kernel Driver and the User Application and is Used to Store Metrics Gathered During the Whole Acceleration Procedure.
 */
struct shared_repository_process * shared_repo_mmap(struct per_thread_info *per_thread_info)
{
	unsigned int *uint_shared_repo_kernel_address;
	struct shared_repository_process *shared_repo_kernel_address;
	
	//clear_screen();
	printf("Memory Mapping Shared Repo Kernel Allocation Buffer\n");
	
	/*
	 * Open the shared_repo_mmap_value File.
	 * This File is Used to Make File Operations(Open, Read, Write, Mmap, Release, etc) Targetting Specific Code Execution Parts of the Kernel Driver.
	 * The shared_repo_mmap_value File Located inside the "/sys/kernel/debug/" Path is a Debugfs File which is Created by the Kernel Driver.
	 * The shared_repo_mmap_value File is Set with the Open, Mmap and Release File Operations that on Being Called Execute Specific Code Routines inside the Kernel Driver.
	 * The Debugfs File is Integrated to Provide Additional Operations between the User Application and the Kernel Driver.
	 */
	per_thread_info->shared_repo_mmap_file = open("/sys/kernel/debug/shared_repo_mmap_value", O_RDWR);
	
	/*
	 * If the per_thread_info->shared_repo_mmap_file Value is Less than Zero then the System Failed to Open the File or the File Does not Exist
	 */	
	if ( per_thread_info->shared_repo_mmap_file < 0 )  
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Error Opening Shared Repo MMAP File\n");
		#endif
	}	
	
	/*
	 * When Calling the mmap() Function the Driver Makes a MMap File Operation which Allocates Memory in Kernel Space and Maps it to the Userspace Application.
	 * The mmap() Function Returns an unsigned int Pointer(uint_shared_repo_kernel_address) which Can be Used so that the
	 * Userspace Application Can Read/Write Directly to the Kernel Memory Allocation.
	 */
	uint_shared_repo_kernel_address = (unsigned int *)mmap(0, MMAP_ALLOCATION_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, per_thread_info->shared_repo_mmap_file, 0);
	
	/*
	 * Cast the uint_shared_repo_kernel_address Pointer to the struct shared_repository_process shared_repo_kernel_address Pointer.
	 * The shared_repo_kernel_address Pointer Can be Used to Read/Write the Kernel Memory Allocation in a Manner of Accessing the Fields of the struct shared_repository_process.
	 * This Memory Allocation is Used to Store Metrics.
	 * Being Shareable it Means that Both the Userspace Application and the Kernel Driver Can Store Metrics in this Kernel Space Memory Allocation.
	 */	
	shared_repo_kernel_address = (struct shared_repository_process *)uint_shared_repo_kernel_address;
	
	
	/*
	 * If the Value of the uint_shared_repo_kernel_address Pointer is Equal with the MAP_FAILED Value it Means that We Failed to Map the Kernel Space Memory Allocation
	 */	
	if (uint_shared_repo_kernel_address == MAP_FAILED)
	{
		printf("Kernel Memory MMAP [FAILURE]\n");
		
		#ifdef DEBUG_MESSAGES_UI
		usleep(2000000);
		#endif
	}	
	else
	{
		printf("Kernel Memory MMAP [SUCCESS]\n");
		printf("Kernel Virtual Address is 0x%016lX\n", (unsigned long)shared_repo_kernel_address);	
			
	}
	

	return shared_repo_kernel_address;
}


/* OK
 * pre_process_mmap()
 * 
 * This Function is Used to Map a Memory Allocation of the Kernel Space so that it Can Be Directly Accessed by the User Application.
 * This Memory Allocation is Shared Between the Kernel Driver and the User Application and is Used to Load the Image Data Directly from the Storage Device to the Kernel Space Memory.
 * Without Using the MMap Technique Application the Userspace would Have to Load the Image Data in a Userspace Memory Allocation and then Copy the Image Data to a Kernel Memory Allocation.
 * By Using the MMap Technique we Avoid Additional Memory Allocations and Data Copies.
 */
uint8_t * pre_process_mmap(struct per_thread_info *per_thread_info)
{
	unsigned int *pre_process_kernel_address;
	uint8_t *u8_pre_process_kernel_address;
	
	
	//clear_screen();
	printf("Memory Mapping Pre-Process Kernel Allocation Buffer\n");
	
	/*
	 * Open the pre_process_mmap_value File.
	 * This File is Used to Make File Operations(Open, Read, Write, Mmap, Release, etc) Targetting Specific Code Execution Parts of the Kernel Driver.
	 * The pre_process_mmap_value File Located inside the "/sys/kernel/debug/" Path is a Debugfs File which is Created by the Kernel Driver.
	 * The pre_process_mmap_value File is Set with the Open, Mmap and Release File Operations that on Being Called Execute Specific Code Routines inside the Kernel Driver.
	 * The Debugfs File is Integrated to Provide Additional Operations between the User Application and the Kernel Driver.
	 */	
	per_thread_info->pre_process_mmap_file = open("/sys/kernel/debug/pre_process_mmap_value", O_RDWR);
	
	/*
	 * If the per_thread_info->pre_process_mmap_file Value is Less than Zero then the System Failed to Open the File or the File Does not Exist
	 */		
	if ( per_thread_info->pre_process_mmap_file < 0 )  
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Error Opening Pre-Process MMAP File\n");
		#endif
	}	
	
	/*
	 * When Calling the mmap() Function the Driver Makes a MMap File Operation which Allocates Memory in Kernel Space and Maps it to the Userspace Application.
	 * The mmap() Function Returns an unsigned int Pointer(pre_process_kernel_address) which Can be Used so that the
	 * Userspace Application Can Read/Write Directly to the Kernel Memory Allocation.
	 */	
	pre_process_kernel_address = (unsigned int *)mmap(0, MMAP_ALLOCATION_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, per_thread_info->pre_process_mmap_file, 0);
	
	/*
	 * Cast the pre_process_kernel_address Pointer to the 8 Bit u8_pre_process_kernel_address Pointer.
	 * The u8_pre_process_kernel_address Pointer Can be Used to Make Byte Reads/Writes from/to the Kernel Memory Allocation.
	 * This Memory Allocation is Used to Store the Image Data that we Load from the Storage Device.
	 */		
	u8_pre_process_kernel_address = (uint8_t *)pre_process_kernel_address;
	
	/*
	 * If the Value of the pre_process_kernel_address Pointer is Equal with the MAP_FAILED Value it Means that We Failed to Map the Kernel Space Memory Allocation
	 */		
	if (pre_process_kernel_address == MAP_FAILED)
	{
		printf("Kernel Memory MMAP [FAILURE]\n");
		
		#ifdef DEBUG_MESSAGES_UI
		usleep(2000000);
		#endif
	}	
	else
	{
		printf("Kernel Memory MMAP [SUCCESS]\n");
		printf("Kernel Virtual Address is 0x%016lX\n", (unsigned long)pre_process_kernel_address);	
			
	}
	
	
	return u8_pre_process_kernel_address;
}


/* OK
 * post_process_mmap()
 * 
 * This Function is Used to Map a Memory Allocation of the Kernel Space so that it Can Be Directly Accessed by the User Application.
 * This Memory Allocation is Shared Between the Kernel Driver and the User Application and is Used to Save the Processed Image Data Directly from the Kernel Space Memory to the Storage Device.
 * Without Using the MMap Technique the Userspace Application would Have to Copy the Image Data From the Kernel Space Memory Allocation to the Userspace Memory Allocation
 * and then Save the Image Data to the Storage Device.
 * By Using the MMap Technique we Avoid Additional Memory Allocations and Data Copies.
 */	
uint8_t * post_process_mmap(struct per_thread_info *per_thread_info)
{
	unsigned int *post_process_kernel_address;
	uint8_t *u8_post_process_kernel_address;
	
	
	//clear_screen();
	printf("Memory Mapping Post-Process Kernel Allocation Buffer\n");
	
	/*
	 * Open the post_process_mmap_value File.
	 * This File is Used to Make File Operations(Open, Read, Write, Mmap, Release, etc) Targetting Specific Code Execution Parts of the Kernel Driver.
	 * The post_process_mmap_value File Located inside the "/sys/kernel/debug/" Path is a Debugfs File which is Created by the Kernel Driver.
	 * The post_process_mmap_value File is Set with the Open, Mmap and Release File Operations that on Being Called Execute Specific Code Routines inside the Kernel Driver.
	 * The Debugfs File is Integrated to Provide Additional Operations between the User Application and the Kernel Driver.
	 */		
	per_thread_info->post_process_mmap_file = open("/sys/kernel/debug/post_process_mmap_value", O_RDWR);
	
	/*
	 * If the per_thread_info->post_process_mmap_file Value is Less than Zero then the System Failed to Open the File or the File Does not Exist
	 */		
	if ( per_thread_info->post_process_mmap_file < 0 )  
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Error Opening Post-Process MMAP File\n");
		#endif
	}	
	
	/*
	 * When Calling the mmap() Function the Driver Makes a MMap File Operation which Allocates Memory in Kernel Space and Maps it to the Userspace Application.
	 * The mmap() Function Returns an unsigned int Pointer(post_process_kernel_address) which Can be Used so that the
	 * Userspace Application Can Read/Write Directly to the Kernel Memory Allocation.
	 */	
	post_process_kernel_address = (unsigned int *)mmap(0, MMAP_ALLOCATION_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, per_thread_info->post_process_mmap_file, 0);
	
	/*
	 * Cast the post_process_kernel_address Pointer to the 8 Bit u8_post_process_kernel_address Pointer.
	 * The u8_post_process_kernel_address Pointer Can be Used to Make Byte Reads/Writes from/to the Kernel Memory Allocation.
	 * This Memory Allocation is Used to Store the Processed Image Data that are Later Saved to the Storage Device.
	 */		
	u8_post_process_kernel_address = (uint8_t *)post_process_kernel_address;
	
	
	/*
	 * If the Value of the post_process_kernel_address Pointer is Equal with the MAP_FAILED Value it Means that We Failed to Map the Kernel Space Memory Allocation
	 */		
	if (post_process_kernel_address == MAP_FAILED)
	{
		printf("Kernel Memory MMAP [FAILURE]\n");
		
		#ifdef DEBUG_MESSAGES_UI
		usleep(2000000);
		#endif
	}	
	else
	{
		printf("Kernel Memory MMAP [SUCCESS]\n");
		printf("Kernel Virtual Address is 0x%016lX\n", (unsigned long)post_process_kernel_address);	
			
	}
	
	
	return u8_post_process_kernel_address;
}	


/* OK
 * start_thread() 
 * 
 * This is the Function that each new Thread is Actually Going to Execute.
 */
void* start_thread(void *arg)
{	
	/*
	 * Make a System Call to Get the Process ID of the Current Thread (Thread ID).
	 */
	pid_t x = syscall(__NR_gettid);
	
	/*
	 * When a Thread Calls pthread_barrier_wait(), it Blocks Until the Number of Threads Specified Initially in the pthread_barrier_init() Function
	 * Have Called pthread_barrier_wait() (and Blocked Also).
	 * When the Correct Number of Threads Have Called pthread_barrier_wait(), all those Threads Will Unblock at the Same Time.
	 */	
	pthread_barrier_wait(&threads_barrier);
	
	printf("[Thread ID: %d]\n", x);

	/*
	 * At this Point each Thread Has Unblocked from pthread_barrier_wait().
	 * At this Point each Thread calls Its Own acceleration_thread() Function
	 * which Actually Starts and Manages the Acceleration Procedure.
	 */
	acceleration_thread();


    return NULL;
}


/* OK
 * multi_threaded_acceleration()
 * 
 * Called to Generate a Number of Threads According to the threads_number Function Argument.
 */
int multi_threaded_acceleration(int threads_number)
{
	int status;
	int repeat;
	
	/*
	 * Create a pthread_t Type Array According to the threads_number.
	 */
	pthread_t thread_id[threads_number];	

	/*
	 * Initialize the Threads Barrier.
	 * A Barrier is a Point where the Thread is Going to Wait for other Threads and Will Proceed Further only when
	 * the Predefined Number of Threads (threads_number) Reach the Same Barrier.
	 */
	pthread_barrier_init(&threads_barrier, NULL, threads_number + 1);
	
	clear_screen();
	
	printf("Performing the Multi-Threading Test\n");
	
	/*
	 * Loop for as many Times as Defined by the threads_number in order to Create the Required Number of Threads.
	 */
	for(repeat = 0; repeat < threads_number; repeat++)
	{
		/*
		 * Create a New Thread of the start_thread() Function
		 */
		status = pthread_create(&thread_id[repeat], NULL, &start_thread, NULL);
		if (status != 0)
		{
			printf("\nCannot Create a Thread :[%s]", strerror(status));
		}
		else
		{
			printf("\nThread Created Successfully\n");
		}
	}
	
	/*
	 * When a Thread Calls pthread_barrier_wait(), it Blocks Until the Number of Threads Specified Initially in the pthread_barrier_init() Function
	 * Have Called pthread_barrier_wait() (and Blocked Also).
	 * When the Correct Number of Threads Have Called pthread_barrier_wait(), all those Threads Will Unblock at the Same Time.
	 */
	pthread_barrier_wait(&threads_barrier);


	/*
	 * The pthread_join() function Waits for the Thread  to Terminate.
	 * We Loop for as many Times as Defined by the threads_number to Make Sure that All Threads are Terminated.
	 */
	for (repeat = 0;repeat < threads_number; repeat++) 
	{
		/*
		 * The pthread_join() function Waits for the Thread  to Terminate.
		 */
		pthread_join(thread_id[repeat], NULL);
	}
	
	/*
	 * Call the pthread_barrier_destroy() Function which Destroys the Barrier and Releases any Resources Used by the Barrier.
	 */
	pthread_barrier_destroy(&threads_barrier);	
	
	return SUCCESS;	
}

/* OK
 * acceleration_thread()
 * 
 * Called to Start a New Acceleration Request and Manage the Acceleration Procedure.
 * There are as many acceleration_thread() Functions as the Number of Threads that the Application Initiated.
 */
int acceleration_thread()
{
	/*
	 * The pre_process_kernel_address Points to the Kernel Memory Created by the pre_process_mmap() Function.
	 * This Kernel Memory is Used to Load the Initial Image Data
	 */
	unsigned int *pre_process_kernel_address = NULL;
	
	/*
	 * The u8_pre_process_kernel_address, also, Points to the Kernel Memory Created by the pre_process_mmap() Function
	 * It is Used for 8 Bit Reads/Writes.
	 */	
	uint8_t *u8_pre_process_kernel_address = NULL;

	/*
	 * The post_process_kernel_address Points to the Kernel Memory Created by the post_process_mmap() Function.
	 * This Kernel Memory is Used to Keep and Later Save the Processed Image Data
	 */
	unsigned int *post_process_kernel_address = NULL;
	
	/*
	 * The u8_post_process_kernel_address, also, Points to the Kernel Memory Created by the post_process_mmap() Function
	 * It is Used for 8 Bit Reads/Writes.
	 */		
	uint8_t *u8_post_process_kernel_address = NULL;

	/*
	 * The uint_shared_repo_kernel_address Points to the Kernel Memory Created by the shared_repo_mmap() Function.
	 * This Kernel Memory is Used to Collect the Metrics Information.
	 */
	unsigned int *uint_shared_repo_kernel_address = NULL;
	
	/*
	 * The shared_repo_kernel_address, also, Points to the Kernel Memory Created by the shared_repo_mmap() Function
	 * It is Used to Access the Metrics Data as Fields of a struct shared_repository_process Structure Type.
	 */		
	struct shared_repository_process *shared_repo_kernel_address = NULL;

	
	/*
	 * The Name of the PCIe Device Driver.
	 */
	char device_driver_name[] = "/dev/xilinx_pci_driver";
	int device_file = -1;

	/*
	 * This Variable Increments for Each New Completed Acceleration.
	 */
	int completed = 0;

	/*
	 * The save_path_name Char Array is Used to Store the String with the Path and Name of the Save Image File.
	 */
	char save_path_name[100];
		
	pid_t tid = syscall(__NR_gettid);
	
	int repeat;
	int global_repeat = 0;
	
	int status = 0;
	
	int page_size;
	
	/*
	 * This Structure Pointer is Used to Store the Pre Process, Post Process and Metrics Kernel Memory Pointers as well as the File Descriptors 
	 * which are used by the pre_process_mmap(), post_process_mmap() and shared_repo_mmap() Functions for Mapping the Kernel's Allocated Memories for the Current Thread.
	 */	
	struct per_thread_info *mm_per_thread_info;
	
	/*
	 * Used to Keep the Last Time Value Captured by the FPGA's Shared Timer.
	 */			
	uint64_t time_stamp;
	 	
	/*
	 * Used to Point to Pre Process Userspace Memory for the Source Data of the Acceleration Group SG.
	 */
	uint8_t *u8_sg_pre_process_kernel_address = NULL;
	
	/*
	 * Used to Point to Post Process Userspace Memory for the Destination Data of the Acceleration Group SG.
	 */	
	uint8_t *u8_sg_post_process_kernel_address = NULL;
	
	/*
	 * This Structure Pointer is Used to Store the Pre Process and Post Process Userspace Memory Pointers.
	 */
	struct sg_list_addresses *sg_list_src_dst_addresses = NULL;
	
	
	char* device_file_name = device_driver_name;
	
	/*
	 * Open the PCIe Device Driver.
	 */
	device_file = open(device_file_name, O_RDWR);

	if ( device_file < 0 )  
	{
		#ifdef DEBUG_MESSAGES_UI
		printf("[DEBUG MESSAGE] Error Opening Device File\n");
		#endif
		return 0;
	}
	
	/*
	 * Call the clear_screen() Function to Clear the Terminal Screen
	 */ 
	clear_screen();
		
	
	#ifdef DEBUG_MESSAGES_UI
	printf("Performing .bmp Image Acceleration\n");
	#endif
	

	/*
	 * Allocate a "per_thread_info" Structure 
	 */	
	mm_per_thread_info = (struct per_thread_info *)malloc(sizeof(struct per_thread_info));
	
	pid = getpid();
	
	/*
	 * Read the Time Spot where the Required Preparation before Acceleration Started.
	 * 
	 * The uint_64_pcie_bar_kernel_address Pointer Points to the PCIe BAR0 which Gives Access to the Peripherlas of the FPGA.
	 * The BAR0_OFFSET_TIMER is the Offset where the Shared Timer Peripheral is Mapped in the FPGA AXI Address Space.
	 * Reading from that Offset of the uint_64_pcie_bar_kernel_address Pointer is Actually Reading the Lower and Upper Registers of the Global Clock Counter of the Shared Timer (Shared APM).
	 */ 
	time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];

	/*
	 * MMap Kernel Memory Allocation (4M) that is Common Between the Kernel Space and the Userspace
	 * This Memory is Used to Store Time and Transfer Metrics
	 */
	shared_repo_kernel_address = shared_repo_mmap(mm_per_thread_info);
	
	/*
	 * Store the Height, Width and Size of the Image that will be Accelerated.
	 * This Information will be Given by the Driver to the Appropriate Acceleration Group.
	 */
	shared_repo_kernel_address->shared_image_info.rows = bitmap_info_header.height;
    shared_repo_kernel_address->shared_image_info.columns = bitmap_info_header.width;
    shared_repo_kernel_address->shared_image_info.size = total_reserved_size;
	

	/*
	 * Store the Time Spot where the Required Preparation before Acceleration Started
	 */ 
	shared_repo_kernel_address->process_metrics.preparation_time_start = time_stamp;
		
	/*
	 * MMap a Kernel Memory Allocation (4M Size) so that it Can be Common Between the Kernel Space and the Userspace.
	 * This Memory is Used by the Userspace Application to Load the Image Directly to the Kernel Space (Pre-Process Data)
	 * This Memory is where the Accelerator Reads the Data from.
	 */ 	
	u8_pre_process_kernel_address = pre_process_mmap(mm_per_thread_info);
	

	/*
	 * MMap a Kernel Memory Allocation (4M Size) so that it can be Common Between the Kernel Space and the Userspace.
	 * This Memory is where the Accelerator Writes the Processed Data to (Post-Process Data).
	 * This Memory is Directly Accessed by the Userspace Application to Save the Processed Data to an Image File.
	 */ 
	u8_post_process_kernel_address = post_process_mmap(mm_per_thread_info);
	
	
	/*
	 * The Post Process Kernel Memory Allocated and Mapped in the Previous Step (post_process_mmap()) is not Used in this Implementation.
	 * Due to Limitation of Available AXI BARs we Use one Kernel Memory Allocation for the Image Data which is the Pre Process Kernel Memory.
	 * The DMA Gets Access to that Memory through one AXI BAR, It Reads the Initial Image Data which are Processed and then Returned to the Same Memory and Same Offset.
	 * As a Result, the Post Process Kernel Memory is Not Required but it is Created in Case the Developer Decides to Make a Different Implementation
	 * Regarding where the DMA Reads from or Writes to and how Many BARs are Used for a Single Acceleration.
	 * 
	 * Taking the Above into Consideration, the u8_post_process_kernel_address Pointer is Set to Point at the Pre Process Kernel Memory as the u8_pre_process_kernel_address Pointer.
	 * The Application will Use the u8_pre_process_kernel_address Pointer to Load the Image and the u8_post_process_kernel_address Pointer to Save the Processed Image.
	 */
	u8_post_process_kernel_address = (uint8_t * )u8_pre_process_kernel_address;
		
	
	/*
	 * Read the Time Spot where the Required Preparation before Acceleration Ended
	 */
	time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
	
	/*
	 * Store the Time Spot where the Required Preparation before Acceleration Ended
	 */	
	shared_repo_kernel_address->process_metrics.preparation_time_end = time_stamp;

	/*
	 * This Loop Contains the Main Steps of the Acceleration Procedure from Requesting Acceleration to Completing the Acceleration.
	 * Each New Iteration of the for Loop is A New Acceleration Request.
	 */	
	for(global_repeat = 0; global_repeat < global_iterations; global_repeat++)
	{
	
		/*
		 * Read and Store the Time Spot where we Start to Capture the Total Time of a Single Iteration of the Acceleration Procedure
		 */	
		time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
		shared_repo_kernel_address->process_metrics.total_time_start = time_stamp;
	
	
		/*
		 * Read and Store the Time Spot where Loading the Image to the Kernel Memory Started
		 */		
		time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
		shared_repo_kernel_address->process_metrics.load_time_start = time_stamp;
		
		/*
		 * Copy the Image Data from the Common Memory where they were Initially Loaded  to the Pre Process Kernel Memory (u8_pre_process_kernel_address).
		 * An Old but Slower Approach was to Load the Image Data to the Pre Process Kernel Memory instead of Using the Copy Method.
		 */
		memcpy((void *)u8_pre_process_kernel_address, (void *)common_load, shared_repo_kernel_address->shared_image_info.size);

		
		/*
		 * Read and Store the Time Spot where Loading the Image to the Kernel Memory Ended.
		 */			
		time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
		shared_repo_kernel_address->process_metrics.load_time_end = time_stamp;
	

        #ifdef DEBUG_MESSAGES_UI
		printf("Sending Access Request to the Driver\n");
		#endif

		/*
		 * Read and Store the Time Spot Right Before the Thread is Possibly Set to Sleep State (If no Acceleration Groups were Found Available).
		 * This is where the Sleep State Possibly Started.
		 */							
		time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
		shared_repo_kernel_address->process_metrics.sleep_time_start = time_stamp;	

		/*
		 * IOCtl Request Access to Hardware Accelerator From Driver.
		 * This System Call Makes the Driver to Execute a Specific Code Routine that will Try to Occupy Acceleration Group(s)
		 */ 		 
		status = ioctl(device_file, COMMAND_REQUEST_ACCELERATOR_ACCESS, (unsigned long)0);
		
		if(status == FAILURE)
		{
			printf("IOCtl Failed\n");
			usleep(1500000);
			
			return FAILURE;
		}

		/*
		 * This if Statement Checks if the Acceleration Group SG is Occupied which Requires Additional Handling for Creating Scatter/Gather Lists.
		 * The Reason for Scatter/Gather Lists is that the AGSG Uses Userspace Memory for Loading the Image Data which is Chunked in Pages.
		 */
		if(shared_repo_kernel_address->accel_occupied == ACCELERATOR_SG_OCCUPIED)
		{
			#ifdef DEBUG_MESSAGES_UI	
			printf("The Only Available Accelerator is SG\nGoing to Allocate Userspace Memory in order to Occupy the Acceleration Group SG\n");
			#endif		
			
			if(u8_sg_pre_process_kernel_address == NULL && u8_sg_post_process_kernel_address == NULL)
			{
				/*
				 * Get the Page Size which is Set by the Linux System.
				 */
				page_size = getpagesize();	
				
				/*
				 * Allocate a "sg_list_addresses" Structure.
				 * This Structure Holds the Pointers for the Pre Process (Source) and the Post Process (Destination) Userspace Memories.
				 */	
				sg_list_src_dst_addresses = (struct sg_list_addresses *)malloc(sizeof(struct sg_list_addresses));	
				
				/*
				 * Allocate 4M of Memory Aligned in Pages of PAGE_SIZE (4K) for the Pre Process Userspace Memory
				 */ 
				status = posix_memalign((void **)&sg_list_src_dst_addresses->sg_list_source_address, page_size, POSIX_ALLOCATED_SIZE);
				
				/*
				 * Set the u8_sg_pre_process_kernel_address Pointer to Point at the Pre Process Userspace Memory as the sg_list_src_dst_addresses->sg_list_source_address Pointer.
				 */
				u8_sg_pre_process_kernel_address = (uint8_t *)sg_list_src_dst_addresses->sg_list_source_address;
				
				if(status == 0)
				{
					printf("Succesfully Allocated Memory for Source Buffer\nThe Virtual Address for Source Buffer is: 0x%016X\n", (unsigned long)sg_list_src_dst_addresses->sg_list_source_address);
				}
				else
				{
					printf("Failed to Allocate Memory for Source Buffer [ERROR %d]", status);
				}
				
				/*
				 * Pin the Allocated Memory to Avoid Swapping.
				 */
				mlock(sg_list_src_dst_addresses->sg_list_source_address, POSIX_ALLOCATED_SIZE);
				
				/*
				 * Allocate 4M of Memory Aligned in Pages of PAGE_SIZE (4K) for the Post Process Userspace Memory
				 */ 				
				status = posix_memalign((void **)&sg_list_src_dst_addresses->sg_list_destination_address, page_size, POSIX_ALLOCATED_SIZE);
				
				/*
				 * Set the u8_sg_post_process_kernel_address Pointer to Point at the Post Process Userspace Memory as the sg_list_src_dst_addresses->sg_list_destination_address Pointer.
				 */				
				u8_sg_post_process_kernel_address = (uint8_t *)sg_list_src_dst_addresses->sg_list_destination_address;
				
				
				if(status == 0)
				{
					printf("Succesfully Allocated Memory for Destination Buffer\nThe Virtual Address for Destination Buffer is: 0x%016X\n", (unsigned long)sg_list_src_dst_addresses->sg_list_destination_address);
				}
				else
				{
					printf("Failed to Allocate Memory for Destination Buffer [ERROR %d]", status);
				}	

				/*
				 * Pin the Allocated Memory to Avoid Swapping.
				 */
				mlock(sg_list_src_dst_addresses->sg_list_destination_address, POSIX_ALLOCATED_SIZE);	
			}
			
			/*
			 * The Thread Originally Copied the Image Data to the Pre Process Kernel Memory (u8_pre_process_kernel_address).
			 * Since there was no Available Acceleration Group (Except for the AGSG) that Uses the Kernel Memory the Data Must be Copied to the
			 * Pre Process Userspace Memory so that they Can be Processed by the Acceleration Group SG (AGSG).
			 */
			memcpy(u8_sg_pre_process_kernel_address, u8_pre_process_kernel_address, total_reserved_size);
			
			
			sg_list_src_dst_addresses->current_pid = tid;
			
			/*
			 * Read and Store the Time Spot where Setting the Scatter/Gather Lists Started.
			 */			
			time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
			shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start = time_stamp;		
			
			/*
			 * IOCtl Request to Create the Scatter/Gather List.
			 * This System Call Provides the Driver with the Pre Process and Post Process Memory Pointers so that the Driver Can Create 
			 * two Scatter/Gather Lists for the Source and Destination of the Image Data.
			 */ 
			ioctl(device_file, COMMAND_SET_PAGES, (unsigned long)sg_list_src_dst_addresses);

			/*
			 * Read and Store the Time Spot where Setting the Scatter/Gather Lists Ended.
			 */				
			time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
			shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end = time_stamp;						

			/*
			 * Read and Store the Time Spot Right Before the Thread is Possibly Set to Sleep State (If no Acceleration Groups were Found Available).
			 * This is where the Sleep State Possibly Started.
			 */	
			time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
			shared_repo_kernel_address->process_metrics.sleep_time_start = time_stamp;
			
			/*
			 * IOCtl Request Access to Hardware Accelerator From Driver.
			 * This System Call Makes the Driver to Execute a Specific Code Routine that will Try to Occupy Acceleration Group(s).
			 * This Time Since there were no other Acceleration Groups Available (Except for the AGSG) the Application Requests to Occupy the Acceleration Group SG.
			 */ 			
			ioctl(device_file, COMMAND_REQUEST_ACCELERATOR_SG_ACCESS, (unsigned long)0);
			
		}
		
		/*
		 * The shared_repo_kernel_address->accel_occupied is a Flag whose 7 LSBs Indicate which Acceleration Groups where Occupied for the Current Thread Depending on the Acceleration Policy.
		 * The shared_repo_kernel_address->accel_completed is a Flag whose 7 LSBs Indicate which Acceleration Groups Have Completed their Procedure.
		 * When the Driver Occupies a Number of Acceleration Groups for the Thread we Expect that the Total Acceleration is Completed when all the Occupied Acceleration Groups Have Completed.
		 * As a Result the while Loop is Active until all the Acceleration Groups that where Occupied are Completed.
		 */
		 
		while(shared_repo_kernel_address->accel_completed != shared_repo_kernel_address->accel_occupied)
		{

		}					
		
		printf("Occupied: %d Completed: %d [PID: %d]\n", shared_repo_kernel_address->accel_occupied, shared_repo_kernel_address->accel_completed, tid);
		

		#ifdef DEBUG_MESSAGES_UI	
		printf("Accereration Completed\n");
		#endif
				
		/*
		 * Call the set_save_accelerator() Function to Create the Path and Name for the Image File.
		 */		
		set_save_accelerator(save_path_name, shared_repo_kernel_address->accel_occupied, tid, global_repeat);
		
		/*
		 * The if-else Statement Below Nests the Procedure for Saving the Processed Image.
		 * Check the accel_occupied Flag to Know if the Acceleration Group SG was Used Because Saving the Image in such Case Requires Special Handling.
		 */
		if(shared_repo_kernel_address->accel_occupied == ACCELERATOR_SG_OCCUPIED)
		{
			/*
			 * Read and Store the Time Spot where Unmapping the Pages Started.
			 * Those Pages were Previously Mapped when Creating the Scatter/Gather Lists.
			 */	
			time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
			shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start = time_stamp;			
			
			/*
			 * IOCtl Request to Unmap the Pages.
			 * This System Call Makes the Driver to Execute a Specific Code Routine that will Try to Unmap the Pages
			 * that were Previoulsy Mapped when Creating the Scatter/Gather Lists.
			 * The Scatter/Gather Mapped Pages Must be Released before the Application Tries to Read and Store
			 * the Processed Image Data from the Pre Process Userspace Memory.
			 * 
			 * The Unmap Procedure is Only Required if the Acceleration Group SG was Occupied.
			 */ 
			ioctl(device_file, COMMAND_UNMAP_PAGES, (unsigned long)0);

			/*
			 * Read and Store the Time Spot where Unmapping the Pages Ended.
			 */				
			time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
			shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end = time_stamp;	
			
			/*
			 * If the save_request Value is Set to 1 then Save the Image in EACH Iteration.
			 */
			if(save_request == 1)
			{
				/*
				 * Read and Store the Time Spot where Saving the Processed Image Started.
				 */	
				time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
				shared_repo_kernel_address->process_metrics.save_time_start = time_stamp;
	
				/*
				 * Call the save_bmp() Function to Save the Image from the Post Process Usespace Memory (u8_sg_post_process_kernel_address) to the Storage Device (save_path_name).
				 */					
				status = save_bmp(u8_sg_post_process_kernel_address, save_path_name);				
	
				/*
				 * Read and Store the Time Spot where Saving the Processed Image Ended.
				 */					
				time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
				shared_repo_kernel_address->process_metrics.save_time_end = time_stamp;	
			}
			/*
			 * If the save_request Value is Set to 2 then Save the Image Only in the Last Iteration.
			 */			
			if(save_request == 2)
			{
				/*
				 * If the Current Iteration is Equal with the Number of global_iterations then it is the Last Iteration so Save the Processed Image.
				 */
				if(global_repeat == (global_iterations - 1))
				{
					/*
					 * Read and Store the Time Spot where Saving the Processed Image Started.
					 */						
					time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
					shared_repo_kernel_address->process_metrics.save_time_start = time_stamp;
				
					/*
					 * Call the save_bmp() Function to Save the Image from the Post Process Usespace Memory (u8_sg_post_process_kernel_address) to the Storage Device (save_path_name).
					 */					
					status = save_bmp(u8_sg_post_process_kernel_address, save_path_name);
					
					/*
					 * Read and Store the Time Spot where Saving the Processed Image Ended.
					 */						
					time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
					shared_repo_kernel_address->process_metrics.save_time_end = time_stamp;						
				}
			}
		}
		else
		{		
			if(save_request == 1)
			{
				/*
				 * Read and Store the Time Spot where Saving the Processed Image Started.
				 */						
				time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
				shared_repo_kernel_address->process_metrics.save_time_start = time_stamp;
					
				/*
				 * Call the save_bmp() Function to Save the Image from the Pre Process Kernel Memory (u8_post_process_kernel_address Points to u8_pre_process_kernel_address)
				 * to the Storage Device (save_path_name).
				 */									
				status = save_bmp(u8_post_process_kernel_address, save_path_name);	
				
				/*
				 * Read and Store the Time Spot where Saving the Processed Image Ended.
				 */					
				time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
				shared_repo_kernel_address->process_metrics.save_time_end = time_stamp;									
			}
			if(save_request == 2)
			{
				/*
				 * If the Current Iteration is Equal with the Number of global_iterations then it is the Last Iteration so Save the Processed Image.
				 */				
				if(global_repeat == (global_iterations - 1))
				{
					/*
					 * Read and Store the Time Spot where Saving the Processed Image Started.
					 */							
					time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
					shared_repo_kernel_address->process_metrics.save_time_start = time_stamp;
												
					/*
					 * Call the save_bmp() Function to Save the Image from the Pre Process Kernel Memory (u8_post_process_kernel_address Points to u8_pre_process_kernel_address)
					 * to the Storage Device (save_path_name).
					 */										
					status = save_bmp(u8_post_process_kernel_address, save_path_name);
					
					/*
					 * Read and Store the Time Spot where Saving the Processed Image Ended.
					 */	
					time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
					shared_repo_kernel_address->process_metrics.save_time_end = time_stamp;						
				}
			}
		}
					
		/*
		 * Read and Store the Time Spot where we End to Capture the Total Time of a Single Iteration of the Acceleration Procedure
		 */			
		time_stamp = uint_64_pcie_bar_kernel_address[BAR0_OFFSET_TIMER / 8];
		shared_repo_kernel_address->process_metrics.total_time_end = time_stamp;
						
		/*
		 * Call the print_save_metrics() Function to Collect and Save the Metrics of the Current Iteration in the Metrics .xml File.
		 */
		print_save_metrics(shared_repo_kernel_address, shared_repo_kernel_address->accel_occupied, tid, global_repeat);
	
		/*
		 * Reset to Zero the Following 6 Fields of the Metrics and Flags Kernel Memory.
		 */
		shared_repo_kernel_address->process_metrics.set_pages_overhead_time_start = 0;
		shared_repo_kernel_address->process_metrics.set_pages_overhead_time_end = 0;
		
		shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_start = 0;
		shared_repo_kernel_address->process_metrics.unmap_pages_overhead_time_end = 0;
		
		shared_repo_kernel_address->accel_completed = 0;
		shared_repo_kernel_address->accel_occupied = 0;			
			
		if(status == SUCCESS)
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("Saving Bitmap [SUCCESS]\n");	
			#endif
			
			
		}
		else
		{
			printf("Multi-Application Access Test Failed / Save Image Error\n");
			usleep(1500000);
		
			return FAILURE;		
		}
		
		completed++;
	
		
		printf("Completed Jobs: %d [PID: %d]\n", completed, tid);
		
	}

		
		
	/*
	 * If the sg_list_src_dst_addresses Pointer is not Null then Release All the Memories Related with the Acceleration Group SG.
	 */
	if(sg_list_src_dst_addresses != NULL)
	{	
		printf("Freed SG Lists [PID: %d]\n", tid);
		
		/*
		 * If the Pre Process (Source) Userspace Memory (sg_list_src_dst_addresses->sg_list_source_address) was Used (not Null) then Release it.
		 */
		if(sg_list_src_dst_addresses->sg_list_source_address != NULL)
		{
			free(sg_list_src_dst_addresses->sg_list_source_address);
			#ifdef DEBUG_MESSAGES_UI
			printf("SG LIST SOURCE FREED %d\n", tid);
			#endif
		}
		
		/*
		 * If the Post Process (Destination) Userspace Memory (sg_list_src_dst_addresses->sg_list_destination_address) was Used (not Null) then Release it.
		 */		
		if(sg_list_src_dst_addresses->sg_list_destination_address != NULL)
		{
			free(sg_list_src_dst_addresses->sg_list_destination_address);		
			#ifdef DEBUG_MESSAGES_UI
			printf("SG LIST DESTINATION FREED %d\n", tid);
			#endif
		}
	
		/*
		 * Free the Memory Allocation of the sg_list_src_dst_addresses Pointer.
		 */
		free(sg_list_src_dst_addresses);		
	}
	
	/*
	 * Call munmap() to Release the Pre Process Kernel Memory that was Mapped when Calling the pre_process_mmap() Function.
	 */
	munmap(mm_per_thread_info->u8_pre_process_kernel_address, MMAP_ALLOCATION_SIZE);
	
	/*
	 * Call munmap() to Release the Post Process Kernel Memory that was Mapped when Calling the post_process_mmap() Function.
	 */	
	munmap(mm_per_thread_info->u8_post_process_kernel_address, MMAP_ALLOCATION_SIZE);
	
	/*
	 * Call munmap() to Release the Metrics Kernel Memory that was Mapped when Calling the shared_repo_mmap() Function.
	 */	
	munmap(mm_per_thread_info->shared_repo_kernel_address, MMAP_ALLOCATION_SIZE);	
	
	/*
	 * Close the pre_process_mmap_file File that was Opened when Calling the pre_process_mmap() Function.
	 */	
	close(mm_per_thread_info->pre_process_mmap_file);
	
	/*
	 * Close the post_process_mmap_file File that was Opened when Calling the post_process_mmap() Function.
	 */		
	close(mm_per_thread_info->post_process_mmap_file);

	/*
	 * Close the shared_repo_mmap_file File that was Opened when Calling the shared_repo_mmap() Function.
	 */		
	close(mm_per_thread_info->shared_repo_mmap_file);
	
	
	/*
	 * Close the PCIe Device Driver.
	 */
	close(device_file);
	
	/*
	 * Free the Memory Allocation of the mm_per_thread_info Pointer.
	 */
	free(mm_per_thread_info);
		
	
	completed = 0;
			
	return SUCCESS;	
}	


/* OK
 * The Starting Point for the Application
 */
int main(int argc, char *argv[]) 
{
	/*
	 * The Device Driver to Open
	 */
	char device_driver_name[] = "/dev/xilinx_pci_driver";
	int device_file = -1;
	
	/*
	 * Used to Store the Arithmetic Value Read from the renamer.txt File
	 */
	char value[4];
	
	/*
	 * Used for File Operations on the Image File
	 */
	FILE *bmp_file;

	/*
	 * Used for File Operations on the Metrics .xml File
	 */		
	FILE *metrics_summary_file;
	
	/*
	 * Used for File Operations on the renamer.txt File
	 */	
	FILE *renamer_file;
	
	/*
	 * Used to Store the Path and Name of the Metrics .xml File
	 */	
	char file_name[100];
	
	int repeat;
	int global_repeat = 0;
	int test_iterations = 0;
	int test_repeat = 0;
	
	int status;
	
	/*
	 * Used to Store the Number of Threads that the Application is Going to Start.
	 */		
	int threads_number = 0;

	/*
	 * Get the First Argument of the Application Call.
	 * The First Argument Represents the Path and Name of the Image File to Accelerate.
	 */	
	strcpy(load_path_name, argv[1]);

	/*
	 * Get the Second Argument of the Application Call.
	 * The Second Argument Represents the Number of Iterations (Acceleration Requests) that each Thread Should Perform.
	 */		
	global_iterations = atoi(argv[2]);
	
	/*
	 * Get the Third Argument of the Application Call.
	 * The Third Argument Represents the Number of Threads that the Application is Going to Start.
	 */	
	threads_number = atoi(argv[3]);
	
	/*
	 * Get the Fourth Argument of the Application Call.
	 * The Fourth Argument Represents the Save Flag which Refers to Saving or Not the Accelerated Image.
	 * See the Comments of the save_request at the Global Variables Section for more Details.
	 */		
	save_request = atoi(argv[4]);
	
	/*
	 * Get the Fifth Argument of the Application Call.
	 * The Fifth Argument Represents the Number of Tests to Run.
	 * In every Test the Application Starts a Number of Threads according to the threads_number Variable
	 * and each Thread Runs for a Number of Iterations (Acceleration Requests) According to the global_iterations Variable
	 */		
	test_iterations = atoi(argv[5]);
	
	clear_screen();
	
	/*
	 * The for Loop Below Represents the Tests Execution
	 * It Loops for as Many Times as Defined by the test_iterations Variable 
	 */
	for(test_repeat = 0; test_repeat < test_iterations; test_repeat++)
	{
		/*
		 * Call pcie_bar_mmap() to Map the PCIe BAR0 and PCIe BAR1 of the PCIe Bridge to the Virtual Address Space of the Userspace
		 * See Details Inside the pcie_bar_mmap() Function Description
		 */
		status = pcie_bar_mmap();
		
		if(status == SUCCESS)
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("Memory Mapping PCIe BAR Address Space [SUCCESS]\n");
			#endif
		}
		else
		{
			printf("Memory Mapping PCIe BAR Address Space [FAILURE]\n");
			#ifdef DEBUG_MESSAGES_UI
			usleep(1500000);
			#endif
			
			return FAILURE;
		}	
				
		/*
		 * Call setup_signal_handling() Function to Setup the Handler for Signals Triggered by the Kernel Module
		 */
		setup_signal_handling();
		
		
		/*
		 * Call getpid() to Get the Parent Process ID
		 */
		pid = getpid();
		
		printf("Process ID is: %d\n", pid);
		
		
		/*
		 * Open the Image File According to the File Name Given by the the User.
		 * In this Point We Open the Image File to Extract Information from Its Header.
		 * This Information (Image Width/Heigth etc) Will be Shared by All the Threads
		 */	 
		 
		bmp_file = fopen(load_path_name, "r");

		if(bmp_file != NULL)
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("Image File Opened\n");
			#endif
		}
		else
		{
			if(bmp_file == NULL)
			{
				printf("Image Failed to Open [NULL Pointer]\n");
			}
		
		usleep(2000000);

		return(FAILURE);
		}
		
		

		#ifdef DEBUG_MESSAGES_UI
		printf("Checking the Magic Number to Validate that this is a Bitmap File\n");
		#endif

		/*
		 * Read the Magic Number from the Header of the Bitmap File.
		 */
		fread(&magic_number, sizeof(bmpfile_magic_t), 1, bmp_file);

		/*
		 * Check the Magic Number to Validate that this is a Bitmap File.
		 * The Magic Number for .bmp Files is: 0x4D42.
		 */
		if (*((uint16_t *)magic_number.magic) == 0x4D42)
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("Bitmap File Valid [MAGIC NUMBER 0x%X]\n", *((uint16_t *)magic_number.magic));
			#endif
		}
		else
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("No Bitmap File Was Found/Aborting\n");
			#endif
			fclose(bmp_file);
			return FAILURE;
		}
		
		
		#ifdef DEBUG_MESSAGES_UI
		printf("Reading the Bitmap File Header\n");
		#endif
		
		/*
		 * Read the Bitmap File Header
		 */
		fread(&bitmap_file_header, sizeof(bmpfile_header_t), 1, bmp_file);


		#ifdef DEBUG_MESSAGES_UI
		printf("Reading the Bitmap Info Header\n");
		#endif

		/*
		 * Read the Bitmap Info Header
		 */
		fread(&bitmap_info_header, sizeof(bitmap_info_header_t), 1, bmp_file);


		 #ifdef DEBUG_MESSAGES_UI
		 printf("Checking Compression\n");
		 #endif
		 
		/*
		 * Read the Info Header Structure to Check if Compression is Supported
		 */		 
		if (bitmap_info_header.compress_type == 0)
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("Compression is Supported\n");
			#endif
		}
		else
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("Warning, Compression is not Supported\n");
			#endif
		}	  
		
		/*
		 * Print Information About the Image
		 */
		 #ifdef DEBUG_MESSAGES_UI
		printf("\n* Image Width:       %d Pixels\n", bitmap_info_header.width);
		printf("* Image Height:      %d Pixels\n", bitmap_info_header.height);
		printf("* Image Size:        %d Bytes\n", bitmap_info_header.bmp_bytesz);
		printf("* Image Header Size: %d Bytes\n", bitmap_info_header.header_sz);
		printf("* Bits Per Pixel:    %d \n\n", bitmap_info_header.bitspp);	
		#endif 
		
		/*
		 * Close the Image File Since We Extracted the Necessary Information from the Headers
		 */		
		fclose(bmp_file);
		
		/*
		 * Allocate a Common Memory Area Equal to the Size of the Clear Image Data (No Headers) Along with the Required Padding.
		 * common_load is the Pointer where All Threads will Copy the Image from.
		 */
		common_load = (uint8_t *)malloc(bitmap_info_header.width * bitmap_info_header.height * 4);

		
		/*
		 * Call the load_bmp() Function to Load the Image to a Common Memory		 
		 */			
		status = load_bmp(common_load);


		/*
		 * Open the renamer.txt File
		 */
		renamer_file = fopen("Results/renamer.txt", "r");	
		
		/*
		 * Read the Arithmetic Value Stored as a String in the renamer.txt File
		 */		
		fscanf(renamer_file, "%s", value);	
		
		/*
		 * Close the renamer.txt File
		 */
		fclose(renamer_file);
		
		/*
		 * Convert the Previous Arithmetic Value from String to Integer and Write it to the renamer_value
		 * This Integer Value Will Be Used to Name and Save the Metrics .xml File when the Current Test Completes
		 */
		renamer_value = atoi(value);

		/*
		 * Use sprintf() to Create a String that Represents the Path and Name of the Metrics .xml File.
		 * The Arithmetic Value of the renamer_value Variable is Included in the File Name to Ensure that each Test Iteration
		 * Creates a New .xml File which is Unique Among the Rest .xml Files.
		 */
		sprintf(file_name,"Results/Metrics_Summary_%d.xml", renamer_value);

        /*
         * Open the Current Metrics .xml File
         */
		metrics_summary_file = fopen(file_name, "a");
		
        /*
         * Since this is the First Time we Open the File it is Required to Write the XML Header.
         */		
		fprintf(metrics_summary_file,"<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n\n\n");
		
		/*
         * It is, also, Required to Write the Open Tag of the Root Node
         */		
		fprintf(metrics_summary_file,"<Process_Plot>\n\n");
		
        /*
         * Close for now the Current Metrics .xml File
         */		
		fclose(metrics_summary_file);

		/*
		 * Write a Value to the Data Register of the GPIO PCIe Interrupt Peripheral of the FPGA through the PCIe BAR0.
		 * The Written Value is a Command to Start the FPGA Shared Timer (Shared APM).
		 * On Receiving the new Value the GPIO PCIe Interrupt Peripheral Triggers an Interrupt.
		 * This Interrupt is Handled by the Microblaze that Reads the Command (Written Value) from the Data Register of the GPIO PCIe Interrupt Peripheral.
		 * See Details for this Pointer at the Global Variables Section
		 */
		uint_64_pcie_bar_kernel_address[BAR0_OFFSET_GPIO_PCIE_INTERRUPT / 8] = (uint32_t)OPERATION_START_TIMER;


		usleep(150000); //Do Not Remove. Microblaze Requires Some Time to Restart the Shared Timer Before we Use it to Get Correct Time Stamps


		/*
		 * Call multi_threaded_acceleration() Function to Start new Threads According to the Value of the threads_number Variable.
		 * When this Function Returns All Threads Have Completed and we are Ready to Move to the Next Test Iteration.
		 */
		multi_threaded_acceleration(threads_number);

		/*
		 * At this Point All Threads Have Completed and any Metrics Info is Already Written to the Current Metrics .xml File.
		 * We Have to Re-open the Current Metrics .xml File to Write the Close Tag of the Root Element.
		 */
		metrics_summary_file = fopen(file_name, "a");
		
		/*
		 * Write the Close Tag of the Root Element.
		 */
		fprintf(metrics_summary_file,"</Process_Plot>\n\n");
		
		/*
		 * Close the Current Metrics .xml File.
		 */
		fclose(metrics_summary_file);

		/*
		 * Increment the Arithmetic Value of the renamer_value Variable.
		 * This Value will be Used in the Next Test Iteration to Save a New Metrics .xml File.
		 */
		renamer_value++;	
		
		/*
		 * Open the renamer.txt File and Update it with the Incremented Arithmetic Value of the renamer_value Variable.
		 */
		renamer_file = fopen("Results/renamer.txt", "w");	
		fprintf(renamer_file,"%d", renamer_value);	
		fclose(renamer_file);
		
		/*
		 * Unmap the PCIe BAR0 from the Virtual Address Space.
		 * It is Important that the Unmap Operation Should Happen before Closing the Corresponding pcie_bar_0_mmap_file.
		 */
		munmap(uint_64_pcie_bar_kernel_address, MMAP_ALLOCATION_SIZE);
		
		/*
		 * Unmap the PCIe BAR1 from the Virtual Address Space
		 * It is Important that the Unmap Operation Should Happen before Closing the Corresponding pcie_bar_1_mmap_file.
		 */		
		munmap(shared_kernel_address, 128 * KBYTE);

		/*
		 * Free the Allocated Common Memory
		 */
		free(common_load);

		/*
		 * Close the pcie_bar_0_mmap_file and pcie_bar_1_mmap_file Files that where Opened when we Previously Called pcie_bar_mmap().
		 * These to Files are Actually Debugfs Files that are Associated with the Xilinx PCIe Device Driver.
		 * See Details in the pcie_bar_mmap() Function Description.
		 */
		close(pcie_bar_0_mmap_file);
		close(pcie_bar_1_mmap_file);

		/*
		 * Open the Xilinx PCIe Device Driver.
		 */
		char* device_file_name = device_driver_name;
		device_file = open(device_file_name, O_RDWR);

		if ( device_file < 0 )  
		{
			#ifdef DEBUG_MESSAGES_UI
			printf("[DEBUG MESSAGE] Error Opening Device File\n");
			#endif
			return 0;
		}
		
		/*
		 * Make a IOCtl System Call to Request Reseting the Driver's Variables.
		 * The Driver will Actually Set to Zero the Synchronization Flags tha are Loacated in the FPGA BRAM.
		 */
		status = ioctl(device_file, COMMAND_RESET_VARIABLES, (unsigned long)0);

		/*
		 * Close the Xilinx PCIe Device Driver.
		 */
		close(device_file);
		
	}
		
				
	return SUCCESS;

}

