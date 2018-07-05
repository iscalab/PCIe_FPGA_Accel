/*******************************************************************************
* Filename:   xilinx_pci_driver.c
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
  **********************************************************************
  * Public Headers
  **********************************************************************
  */
 
#include <linux/version.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/sched.h>


/**
  **********************************************************************
  * Local Headers
  **********************************************************************
  */
 
 #include "xilinx_pci_driver.h"


/**
  **********************************************************************
  * Global Variables
  **********************************************************************
  */ 

/*
 * PCIe Device Structure.
 */
struct pci_dev *dev = NULL;

/*
 * The Driver's Name .
 */
char driver_name[]= "isca_pcie_driver"; 

/*
 * The Semaphores that are Used to Explicitly Lock Part of the Code to a Thread.
 */
struct rw_semaphore ioctl_sem;

struct rw_semaphore case_0_sem;
struct rw_semaphore case_1_sem;
struct rw_semaphore case_2_sem;
struct rw_semaphore case_3_sem;
struct rw_semaphore case_4_sem;
struct rw_semaphore case_5_sem;
struct rw_semaphore case_6_sem;

struct rw_semaphore msi_1_sem;
struct rw_semaphore msi_2_sem;
struct rw_semaphore msi_3_sem;
struct rw_semaphore msi_4_sem;
struct rw_semaphore msi_5_sem;
struct rw_semaphore msi_6_sem;
struct rw_semaphore msi_7_sem;
struct rw_semaphore msi_8_sem;

struct rw_semaphore set_pages_sem;
struct rw_semaphore unmap_pages_sem;
struct rw_semaphore sg_sem;
struct rw_semaphore write_sem;

struct rw_semaphore search_element_sem;

struct rw_semaphore main_open_sem;
struct rw_semaphore main_release_sem;
struct rw_semaphore shared_repo_mmap_sem;
struct rw_semaphore pre_process_mmap_sem;
struct rw_semaphore post_process_mmap_sem;

/*
 * The ioctl_queue is Used to Queue the Userspace Threads that are Put in Sleep State.
 */
wait_queue_head_t ioctl_queue;

/*
 * BAR0, BAR1 and BAR2 64 Bit Hardware/Physical Addresses.
 */
u64 bar0_address_physical;
u64 bar1_address_physical;
u64 bar2_address_physical;

/*
 * BAR0, BAR1 and BAR2 64 Bit Virtual Addresses.
 */
u64 *bar0_address_virtual;
u64 *bar1_address_virtual;
u64 *bar2_address_virtual;

/*
 * BAR0, BAR1 and BAR2 32 Bit Virtual Addresses.
 */
u32 *u32_bar0_address_virtual;
u32 *u32_bar1_address_virtual;
u32 *u32_bar2_address_virtual;

/*
 * BAR0, BAR1 and BAR2 Lengths.
 */
u64 bar0_length;
u64 bar1_length;
u64 bar2_length;

/*
 * The Debugfs Files.
 */
struct dentry *pre_process_mmap_file;
struct dentry *post_process_mmap_file;
struct dentry *shared_repo_mmap_file;

/*
 * Pointer of Type struct shared_repository that will be Used to Access the FPGA's BRAM.
 * The Reason for Using that Pointer is that we Want to Access the BRAM as Fields of a Structure of Type struct shared_repository.
 */
struct shared_repository *inter_process_shared_info_memory;

/*
 * The IRQ of the Endpoint Device.
 */
int irq;

/*
 * Status Flags that are Used for Cleanup.
 */
int status_flags = 0x00;

/*
 * The Major Number of the Driver Module (Not Dynamic)
 */
int driver_major_number = 240;

/*
 * Used to Store the Value of the Signal that may be Sent to a Userspace Process/Thread.
 */
short int signal_to_pid;

/*
 * Used to Indicate the Head of a Singly Linked List with Nodes of Type struct pid_reserved_memories.
 * 
 * The Usage of the Singly Linked List is Vital in Order for the Driver to Support Multiple Userspace Threads.
 * 
 * Each Node of the Singly Linked List is Actually a Structure That is Corresponding ONLY to a Single Userspace Thread.
 * The Fields of the Node's Structure (struct pid_reserved_memories) are Mostly Pointers to Memories that are Explicitly Allocated for the Node's Corresponding Userspace Thread.
 * As a Result, when the Driver Needs to Access the Memories of a Specific Thread it Does it through the Pointers that are Found inside the Thread's Corresponding Node.
 * 
 * Each Node is Distinguished by an Integer Field with the ID of its Corresponding Thread.
 * 
 * The Pointers of each Node Point to:
 * 
 * --> The Kernel Memory Allocation (Virtual/Physical Addresses) which is Used to Store the Metrics of the Acceleration Procedure of the Current Thread (Shared Repo MMap).
 * --> The Kernel Memory Allocation (Virtual/Physical Addresses) which is Used to Store the Initial Image Data (Pre-Process MMap).
 * --> The Kernel Memory Allocation (Virtual/Physical Addresses) which is Used to Store the Processed Image Data (Post-Process MMap).
 * --> The Userspace Source and Destination Scatter/Gather Lists of this Node's Userspace Thread.
 * --> The Physical Addresses of the Pages of the Source and Destination Memories that Belong to the Node's Userspace Thread.
 */
static struct pid_reserved_memories *pid_list_head = NULL;

/*
 * Used to Move Through the Singly Linked List.
 */
static struct pid_reserved_memories *pid_list_mover = NULL;

/*
 * Used to Point to an Offset of the FPGA's BRAM where the Scatter/Gather List of the Userspace Source Memory will be Stored.
 */
u32 *sg_list_source_base_address; 

/*
 * Used to Point to an Offset of the FPGA's BRAM where the Scatter/Gather List of the Userspace Destination Memory will be Stored.
 */
u32 *sg_list_destination_base_address;


/**
  **********************************************************************
  * Functions Declaration
  **********************************************************************
  */ 
 
irqreturn_t irq_handler_0 (int irq, void *dev_id, struct pt_regs *regs); 
irqreturn_t irq_handler_1 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_handler_2 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_handler_3 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_handler_4 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_handler_5 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_handler_6 (int irq, void *dev_id, struct pt_regs *regs);

irqreturn_t irq_fast_handler_0 (int irq, void *dev_id, struct pt_regs *regs); 
irqreturn_t irq_fast_handler_1 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_fast_handler_2 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_fast_handler_3 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_fast_handler_4 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_fast_handler_5 (int irq, void *dev_id, struct pt_regs *regs);
irqreturn_t irq_fast_handler_6 (int irq, void *dev_id, struct pt_regs *regs);

void initcode(void);
u32 xilinx_pci_driver_read_cfg_register (u32 byte_offset);
void write_remote_register(u64 *, u64, u32);
u32 read_remote_register(u64 *, u64);
int setup_and_send_signal(u8 signal, pid_t pid);


/**
  **********************************************************************
  * Functions Description
  **********************************************************************
  */ 


/** OK
  * xilinx_pci_driver_open()
  * 
  * It is Called when a Userspace Application Opens the Driver File.
  * 
  * When the Driver Module Opens for the First Time the xilinx_pci_driver_open() Function is Responsible to Clear the Acceleration Flags.
  * 
  * For Every new Userspace Thread that Opens the Driver Module the xilinx_pci_driver_open() Function is Responsible to Create and Insert a 
  * new Node to the Signly Linked List which is Referenced by the ID of the Thread.
  */
int xilinx_pci_driver_open(struct inode *inode, struct file *file_pointer)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Create a new Node for the the Singly Linked List.
	 */
	struct pid_reserved_memories *new_element = NULL;
	
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;
	
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Opening Main Driver Module\n", driver_name, current->pid);
	#endif
	
	/*
	 * Lock the main_open_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */	
	down_write(&main_open_sem);

	/*
	 * When the Driver Module Opens for the First Time the xilinx_pci_driver_open() Function is Responsible to Set the inter_process_shared_info_memory Pointer
	 * to Point at the FPGA BRAM (bar1_address_virtual) which is Used, among others, to Store the Acceleration Flags.
	 * 
	 * If the inter_process_shared_info_memory Pointer Has NULL Value then we Know that it is the First Time that we Open the Driver Module.
	 * So, Set the inter_process_shared_info_memory Pointer to Point at the FPGA BRAM and Clear the Acceleration Flags.
	 */
	if(inter_process_shared_info_memory == NULL)
	{
	
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Setting Inter Process Shared Repository\n",driver_name, current->pid);
		#endif	
	
		/*
		 * Set the inter_process_shared_info_memory Pointer to Point where the bar1_address_virtual Pointer Points which is the Base Address of the FPGA BRAM.
		 */
		inter_process_shared_info_memory = (struct shared_repository *)bar1_address_virtual;
		
		/*
		 * Set a 32 Bit Pointer (u32_bar1_address_virtual) to, also, Point at the FPGA BRAM in Case we Need to Make 32 Bit Writes/Reads.
		 * The bar1_address_virtual is a 64 Bit Pointer.
		 */
		u32_bar1_address_virtual = (u32 *)bar1_address_virtual;
		
		/*
		 * The FPGA BRAM is Used, among others, to Store the Scatter/Gather List Physical Addresses of the Source and Destination Userspace Memories.
		 * The Source is the Memory where the Initial Image Data is Stored and the Destination is the Memory where the Processed Image Data is Stored.
		 * 
		 * Set the sg_list_source_base_address Pointer to Point at 32K Offset at the FPGA BRAM where the Scatter Gather List of the Source Userspace Memory 
		 */
		sg_list_source_base_address = (u32 *)(u32_bar1_address_virtual + (32 * KBYTE / 4));
		
		/*
		 * Set the sg_list_destination_base_address Pointer to Point at 64K Offset at the FPGA BRAM where the Scatter Gather List of the Destination Userspace Memory is Stored.
		 */
		sg_list_destination_base_address = (u32 *)(u32_bar1_address_virtual + (64 * KBYTE / 4));	
		
		/*
		 * Clear the open_modules Flag.
		 * 
		 * This Flag is NOT Currently in Use but it is Kept for Possible Future Implementations.
		 */
		inter_process_shared_info_memory->shared_status_flags.open_modules = 0;
		
		/*
		 * Clear the accelerator_busy Flag.
		 * 
		 * This Flag is NOT Currently in Use but it is Kept for Possible Future Implementations.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accelerator_busy = 0;
		
		/*
		 * Clear the agd0_busy, agd1_busy, agi0_busy, agi1_busy, agi2_busy, agi3_busy and agsg_busy Flags.
		 * 
		 * Those Flags are Used to Store the ID of the Userspace Thread that Occupied the Corresponding Acceleration Group.
		 */
		inter_process_shared_info_memory->shared_status_flags.agd0_busy = 0;
		inter_process_shared_info_memory->shared_status_flags.agd1_busy = 0;
		inter_process_shared_info_memory->shared_status_flags.agi0_busy = 0;
		inter_process_shared_info_memory->shared_status_flags.agi1_busy = 0;
		inter_process_shared_info_memory->shared_status_flags.agi2_busy = 0;
		inter_process_shared_info_memory->shared_status_flags.agi3_busy = 0;
		inter_process_shared_info_memory->shared_status_flags.agsg_busy = 0;

		/*
		 * Write a Start Value to the Data Register of the GPIO_PCIE_INTERRUPT Peripheral of the FPGA through the PCIe Bus to Start the Shared Timer (Shared APM).
		 * 
		 * It is NOT Currently in Use Since the Shared Timer of the FPGA is Now Started by the Parent Thread of the Userspace Application.
		 */	
		//write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_PCIE_INTERRUPT, (u32)OPERATION_START_TIMER);
	
	}
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Inter Process Shared Repository is Already Set\n",driver_name, current->pid);
		#endif
	}
		
	/*
	 * Create a new Memory Allocation which Has Size Equal to a pid_reserved_memories Structure and Set the new_element Pointer to Point at this Allocation.
	 * 
	 * This Memory Allocation is Going to be a new Node that will be Inserted in the Singly Linked List.
	 */
	new_element = (struct pid_reserved_memories *) kmalloc(sizeof(struct pid_reserved_memories), GFP_KERNEL);
 
	/*
	 * If the new_element Pointer Has NULL Value then we Failed to Allocate Memory.
	 */
 	if(new_element == NULL)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Failed to Create Element Structure\n", driver_name, current->pid);
		#endif
		
		return FAILURE;
	}
		
	/*
	 * Set the new Node's Process ID Value to be the Current Thread's Process ID.
	 * 
	 * At this Point we Have Created a new Node that Can be Identified by the PID that Belongs to the Current Thread.
	 * When the Current Thread Requires to Access the Structure Fields of Its Node it Should Look the Singly Linked List
	 * to Find the Node with the Same PID Value as the PID of the Thread.
	 */
 	new_element->pid = current->pid;
 	
 	/*
 	 * Clear the Structure Fields of the new Node so that it is Initialized.
 	 */
	new_element->shared_repo_virtual_address = NULL;
	new_element->shared_repo_physical_address = 0;
	new_element->pre_process_mmap_virtual_address = NULL;
	new_element->post_process_mmap_virtual_address = NULL;
	new_element->next_pid = NULL;

	/*
	 * This if-else Condition is where the new Node is Inserted to the Singly Linked List.
	 * 
	 * If the pid_list_head List Head Pointer has NULL Value then the Previously Created Node is the First Node so it Should be Inserted as the Head of the List.
	 * 
	 * Else the new Node is not the First so Insert it at the End of the Singly Linked List.
	 */
	 if(pid_list_head == NULL)
	 {
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Inserting the First Element in the List\n", driver_name, current->pid);
		#endif
		
		/*
		 * Set the pid_list_head and pid_list_mover Pointers to Point at the new Node (new_element).
		 */
		pid_list_head = new_element;
		pid_list_mover = new_element;

		
	 }
	 else
	 {
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Inserting New Element in the List\n", driver_name, current->pid);
		#endif

		/*
		 * The pid_list_mover Pointer is Currently Pointing at the Previous Node.
		 * 
		 * Set the next_pid Pointer (which is a Structure Field of the pid_list_mover Pointer) to Point at the new Node.
		 * This Way the new Node is Inserted in the Tail of the Singly Linked List.
		 */
		pid_list_mover->next_pid = new_element;
		pid_list_mover = new_element;
		 
	 }
	 
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	 
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 * 
	 * The Reason for Searching the Singly Linked List at this Point is to Validate that we Have Successfully Inserted the New Node.
	 */	 
	while(search_element != NULL)
	{
		
		/*
		 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
		 * If this is the Case then we Have Validated that the Node is Successfully Inserted in the Singly Linked List.
		 */		
		if(search_element->pid == current->pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MAIN OPEN (PID %d)] Adding the New Element is Validated\n", driver_name, current->pid);
			#endif
			
			break;
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}	
	
	/*
	 * Unlock the main_open_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */	
	up_write(&main_open_sem); 

	
	return SUCCESS;
}

/** OK
  * xilinx_pci_driver_release()
  * 
  * It is Called when a Userspace Thread Releases the Driver File.
  * 
  * When a Userspace Thread Releases the Driver Module we Know that it will no Longer Require the Kernel Resources.
  * As a Result the Driver is Responsible to Free the Memory Resources that were Explicitly Created for the Current Thread and Remove 
  * the Current Thread's Node from the Singly Linked List.
  */
int xilinx_pci_driver_release(struct inode *inode, struct file *file_pointer)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */		
	struct pid_reserved_memories *search_element = NULL;
	
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used when Removing a Node in Order to Connect the Previous and the Next Nodes of the Removed Node.
	 */		
	struct pid_reserved_memories *previous_element = NULL;
	
	int validate = 0;
	int check_elements = 0;
	
	/*
	 * Lock the main_release_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */			
	down_write(&main_release_sem);

	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	 
	search_element = pid_list_head;
	
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 * 
	 * The Reason for Searching the Singly Linked List at this Point is to Find the Current Thread's Node in Order to Free the Thread's Kernel Resources and Remove the Pointer.
	 */	 	 
	 while(search_element != NULL)
	 {
		/*
		 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
		 * If this is the Case we Can Proceed to Removing the Node from the Singly Linked List.
		 * 
		 * Else we Set the search_element Pointer to Point at the Next List Node and the previous_element Pointer to Point at the Current List Node.
		 * As a Result when Moving Forward in the List we Always Know the Previous Node.
		 */			
		if(search_element->pid == current->pid)
		{
			/*
			 * If the Node that we Want to Remove is the Head of the List then we Should Set the Next Node as the Head of the List before Removing the Current Node.
			 * 
			 * Else we Should Set the Previous Node to Point at the Next Node of the Current Node that we are about to Remove.
			 */
			if(search_element == pid_list_head)
			{
				/*
				 * Set the pid_list_head Pointer to Point at the Next Node (search_element->next_pid).
				 */
				pid_list_head = search_element->next_pid;		
			}
			else
			{
				/*
				 * Set the Previous Node's next_pid Pointer to Point at the Current Node's next_pid Pointer which Actually Points at the Next Node of the Current Node.
				 * As a Result we Connect the Previous and the Next Node of the Current Node.
				 */
				previous_element->next_pid = search_element->next_pid;
			}
			
			/*
			 * If the Current Node's pre_process_mmap_virtual_address Pointer is not NULL then it Points to a Kernel Memory Allocation.
			 * This Kernel Memory Allocation Belongs to the Current Thread and we Should Free it with dma_free_coherent() Since it will no Longer be Required.
			 */
			if(search_element->pre_process_mmap_virtual_address != NULL)
			{
				dma_free_coherent(&dev->dev, MMAP_ALLOCATION_SIZE, search_element->pre_process_mmap_virtual_address, search_element->pre_process_mmap_physical_address);
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> MAIN RELEASE (PID %d)] Pre Process MMAP Memory Freed\n", driver_name, current->pid);
				#endif
			}
			
			/*
			 * If the Current Node's post_process_mmap_virtual_address Pointer is not NULL then it Points to a Kernel Memory Allocation.
			 * This Kernel Memory Allocation Belongs to the Current Thread and we Should Free it with dma_free_coherent() Since it will no Longer be Required.
			 */			
			if(search_element->post_process_mmap_virtual_address != NULL)
			{				
				dma_free_coherent(&dev->dev, MMAP_ALLOCATION_SIZE, search_element->post_process_mmap_virtual_address, search_element->post_process_mmap_physical_address);
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> MAIN RELEASE (PID %d)] Post Process MMAP Memory Freed\n", driver_name, current->pid);
				#endif					
			}
			
			/*
			 * If the Current Node's shared_repo_virtual_address Pointer is not NULL then it Points to a Kernel Memory Allocation.
			 * This Kernel Memory Allocation Belongs to the Current Thread and we Should Free it with dma_free_coherent() Since it will no Longer be Required.
			 */			
			if(search_element->shared_repo_virtual_address != NULL)
			{				
				dma_free_coherent(&dev->dev, MMAP_ALLOCATION_SIZE, search_element->shared_repo_virtual_address, search_element->shared_repo_physical_address);
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> MAIN RELEASE (PID %d)] Shared Repo MMAP Memory Freed\n", driver_name, current->pid);
				#endif					
			}

			/*
			 * Free the Memory Allocation where the Current Node was Stored which Results in Removing the Current Node.
			 */
			kfree(search_element);
		
		
			/*
			 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
			 */		
			search_element = pid_list_head;
			
	 	 	/*
			 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
			 * 
			 * The Reason for Searching the Singly Linked List at this Point is to Validate that we Have Successfully Removed the New Node.
			 */	
			while(search_element != NULL)
			{
				
				/*
				 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
				 * If this is the Case then the Node is Still Present so Increment the Value of the validate Variable to Indicate the Presence of the Node.
				 */					
				if(search_element->pid == current->pid)
				{
					validate++;
				}
				 
				/*
				 * Set the search_element Pointer to Point at the Next List Node.
				 */						 
				search_element = search_element->next_pid;
			}
			
			/*
			 * If the Value of the validate Variable is Zero then the Node was not Found in the List.
			 */
			if(validate == 0)
			{
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> MAIN RELEASE (PID %d)] Deleting Current Element is Validated\n", driver_name, current->pid);
				#endif

			}			
			
		}
		else
		{
			/*
			 * Set the previous_element Pointer to Point at the Current Node.
			 */
			previous_element = search_element;
			
			/*
			 * Set the search_element Pointer to Point at the Next List Node.
			 */				
			search_element = search_element->next_pid;
		}
		 		
	}	
	

	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */		 	
	search_element = pid_list_head;
	 	
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 * 
	 * The Reason for Searching the Singly Linked List at this Point is to Check if there are any Left Nodes in the Singly Linked List.
	 * If the List is Empty then we Should Clean the pid_list_head Pointer.
	 */	 	 
	while(search_element != NULL)
	{
		/*
		 * If the search_element Pointer is not NULL then we Have Nodes in the List so Increment the check_elements Variable.
		 */
		if(search_element != NULL)
		{
			check_elements++;
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */			 
		search_element = search_element->next_pid;
	}
	
	/*
	 * If the check_elements Variable Has Zero Value then the Singly Linked List is Empty.
	 */
	if(check_elements == 0)
	{
		/*
		 * Clear the pid_list_head Pointer.
		 */
		pid_list_head = NULL;
	}

	/*
	 * Unlock the main_release_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */		
	up_write(&main_release_sem);
		
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MAIN RELEASE (PID %d)] Releasing Main Driver Module\n", driver_name, current->pid);
	#endif	
	
	
	return(SUCCESS);
}

/** OK
  * xilinx_pci_driver_unlocked_ioctl()
  * 
  * Input/Output System Control (IOCtl) is a Kind of Device-Specific System Call.
  * The few System Calls in Linux are not Enough to Express all the Unique Functions that Devices may Require.
  * 
  * The Driver Can Define an IOCtl which Allows a Userspace Application to Send Orders/Commands to It and Expand the Number of Functions for a Device.
  * 
  * The xilinx_pci_driver_unlocked_ioctl() is Used by the Userspace Application Threads for the Following Request Commands:
  * 
  * --> Request to Occupy Acceleration Groups
  * --> Request to Explicitly Occupy the Acceleration Group Scatter/Gather
  * --> Request to Create Scatter/Gather Lists
  * --> Request to Unmap Pages which where Mapped when Creating the Scatter/Gather Lists
  * --> Request to Clear the Flags that are Used to Handle the Acceleration Procedure
  */
long xilinx_pci_driver_unlocked_ioctl(struct file *file_pointer, unsigned int command, unsigned long userspace_value)
{
	
/*
 * Pointer of Type struct sg_list_addresses.
 * Used to Point to a Userspace Memory Allocation where the Pointers to the Userspace Source and Destination Memories are Stored.
 */	
struct sg_list_addresses *sg_list_src_dst_addresses;	
	
/*
 * Pointer of Type struct pid_reserved_memories.
 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Thread.
 */	
struct pid_reserved_memories *search_element = NULL;

int sg_table_value_source;
int sg_table_value_destination;
					
int buffer_entries_source = 0;
int buffer_entries_destination = 0;

int repeat;

/*
 * Used to Store the Pages of the Userspace Source Memory.
 */
struct page **buffer_page_array_source;

/*
 * Used to Store the Pages of the Userspace Destination Memory.
 */
struct page **buffer_page_array_destination;

/*
 * The 7 Flags Below are Used to Set the Acceleration Group(s) that will be Assigned to the Current Thread.
 */
int direct_0_accel = 0;
int direct_1_accel = 0;
int indirect_0_accel = 0;
int indirect_1_accel = 0;
int indirect_2_accel = 0;
int indirect_3_accel = 0;
int sg_accel = 0;

/*
 * Dependng on the Acceleration Policy if a Single Image will be Processed by Multiple Acceleration Groups then it Should be
 * Divided into a Number of Segments which is Equal to the Number of the Acceleration Groups that will Collaborate to Process that Single Image.
 * The segments Variable Indicates that Number of Segments.
 */
int segments = 0;

/*
 * The segment_size Variable is the Number of the Image Rows Divided by the Number of Acceleration Groups that will Process a Single Image.
 * This Variable Gives an Initial Reference to how Many Rows of the Image each Acceleration Group Should Process.
 */
int segment_size = 0;

/*
 * If the segment_size Variable is not an Integer Multiple of the Acceleration Groups that will Process the  Image
 * then there Should be Remaining Rows that Should be Shared Among the Acceleration Groups.
 * 
 * For example, if 3 Acceleration Groups were Occupied in Order to Process an Image of 11 Rows then:
 * 
 * Dividing 11 by 3 Does not Return an Integer Multiple of the 3 Acceleration Groups.
 * The Integer Part of the Division is 3 so we Have 3 Rows for each of the Acceleration Groups.
 * 
 * As a Result, 3 Acceleration Groups by 3 Rows is 9 which Leaves 2 Remaining Rows that are Stored in the remaining_rows Variable.
 * 
 * The 2 Remaining Rows will be Shared in such way that 1 Row will be Given to the First Acceleration Group and 1 Row to the Second.
 */
int remaining_rows = 0;

/*
 * The Maximum Number of Segments that an Image can be Divided Into is 6 which is, also, the Maximum
 * Number of Acceleration Groups that can be Occupied to Process a Single Image.
 * 
 * The segment_rows Array Has 6 Fields where each Field is Used to Store the Number of Rows that each Acceleration Group Should Process.
 * 
 * The Final Number of Rows for Each Acceleration Group is Equal to the segment_size or Possibly segment_size + 1 if the 
 * Image Rows Divided by the Number of Acceleration Groups is not an Integer Multiple.
 * 
 * --> segment_rows[0] Corresponds to AGD0.
 * --> segment_rows[1] Corresponds to AGD1.
 * --> segment_rows[2] Corresponds to AGI0.
 * --> segment_rows[3] Corresponds to AGI1.
 * --> segment_rows[4] Corresponds to AGI2.
 * --> segment_rows[5] Corresponds to AGI3.
 */
int segment_rows[6];

/*
 * The segment_count Variable is Used to Access the Fields of the segment_rows Array.
 */
int segment_count = 0;

/*
 * The segment_offset Variable is the Offset in Bytes where the Acceleration Group Should Read/Write the Image Segment.
 */
int segment_offset = 0;

/*
 * Lock the ioctl_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
 */
down_write(&ioctl_sem);

switch(command)
{
	/*
	 * This Case is Used when a Userspace Thread Requests to Occupy Acceleration Group(s) Depending on the Driver's Policy for Distributing the Acceleration Groups.
	 */
	case COMMAND_REQUEST_ACCELERATOR_ACCESS:

		/*
		 * The wait_event_interruptible() Puts a Process to Sleep Until a Condition Evaluates to True.
		 * 
		 * In this Case the Driver Checks in the Below Condition if any of the agd0_busy, agd1_busy, agi0_busy, agi1_busy, agi2_busy, agi3_busy, agsg_busy Flags Has Zero Value 
		 * which Means that there is at Least one Acceleration Group that is not Busy(not Occupied).
		 * 
		 * If the Condition Returns Non-Zero Value then the Process will be Set to Sleep and Put in a Wait Queue.
		 * Once a Wake Up Call Takes Place in the Future the Condition will be Re-evaluated.
		 * 
		 * If the Condition Below Validates that there is at Least One Acceleration Group Available then the Userspace Thread Can Proceed to Request Acceleration.
		 */
		wait_event_interruptible(ioctl_queue, (inter_process_shared_info_memory->shared_status_flags.agd0_busy &
											   inter_process_shared_info_memory->shared_status_flags.agd1_busy &
											   inter_process_shared_info_memory->shared_status_flags.agi0_busy &
											   inter_process_shared_info_memory->shared_status_flags.agi1_busy &
											   inter_process_shared_info_memory->shared_status_flags.agi2_busy &
											   inter_process_shared_info_memory->shared_status_flags.agi3_busy &
											   inter_process_shared_info_memory->shared_status_flags.agsg_busy) == 0);
			
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] New Process Request for Acceleration Group\n", driver_name, current->pid);
		#endif	
		
		/*
		 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
		 */	 		
		search_element = pid_list_head;
	
		/*
		 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
		 * 
		 * The Reason for Searching the Singly Linked List at this Point is to Find the List Node that Belongs to the Current Userspace Thread.
		 * The Structure Fields of the Current Thread's Node will be Needed During the Acceleration Procedure.
		 */			 
		 while(search_element != NULL)
		 {
			/*
			 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
			 * If this is the Case then we Can Proceed to Request an Acceleration Group.
			 */				
			if(search_element->pid == current->pid)
			{
				/*
				 * This Macro if Condition Encloses Part of the Code that is ONLY Applicable in the Greedy Policy.
				 * 
				 * The Greedy Policy Tries to Occupy as many Accelerators as Possible for a Single Image Accelerarion for a Single Userspace Thread.
				 * This Policy Initially Checks if any of the AGD0, AGD1, AGI0, AGI1, AGI2, AGI3 is Available and Locks all of those that are Found Available.
				 * If it Fails with the Previous Step then it Checks if the AGSG is Available and Locks it.
				 * The Reason for this Separation is that the AGSG Requires Special Handling due to Using Scatter/Gather Lists so it Can Only be Used if no Other is Available.
				 * 
				 * This Part of the Code is where the Driver Locks the Available Acceleration Groups so that they Can be Occupied ONLY by the Current Thread.
				 */
				#ifdef GREEDY	
				/*
				 * If this Condition Returns Zero Value then at Least one Acceleration Group from the AGD0, AGD1, AGI0, AGI1, AGI2 or AGI3 is Available for Locking.
				 * 
				 * Else Try to Occupy the AGSG.
				 */			
				if((inter_process_shared_info_memory->shared_status_flags.agd0_busy &
				   inter_process_shared_info_memory->shared_status_flags.agd1_busy &
				   inter_process_shared_info_memory->shared_status_flags.agi0_busy &
				   inter_process_shared_info_memory->shared_status_flags.agi1_busy &
				   inter_process_shared_info_memory->shared_status_flags.agi2_busy &
				   inter_process_shared_info_memory->shared_status_flags.agi3_busy) == 0)
				{
					/*
					 * Check if the agd0_busy Flag has Zero Value which Means that AGD0 is Available.
					 */
					if(inter_process_shared_info_memory->shared_status_flags.agd0_busy == 0)
					{
						/*
						 * Set the direct_0_accel Flag as Occupied.
						 * This Flag will be Used Later to Lock the AGD0 as Occupied.
						 */
						direct_0_accel = OCCUPIED;
						
						/*
						 * Increment the Value of the segments Variable.
						 */
						segments++;
					}
					/*
					 * Check if the agd1_busy Flag has Zero Value which Means that AGD1 is Available.
					 */					
					if(inter_process_shared_info_memory->shared_status_flags.agd1_busy == 0)
					{
						/*
						 * Set the direct_1_accel Flag as Occupied.
						 * This Flag will be Used Later to Lock the AGD1 as Occupied.
						 */						
						direct_1_accel = OCCUPIED;
						
						/*
						 * Increment the Value of the segments Variable.
						 */						
						segments++;
					}
					/*
					 * Check if the agi0_busy Flag has Zero Value which Means that AGI0 is Available.
					 */						
					if(inter_process_shared_info_memory->shared_status_flags.agi0_busy == 0)
					{
						/*
						 * Set the indirect_0_accel Flag as Occupied.
						 * This Flag will be Used Later to Lock the AGI0 as Occupied.
						 */							
						indirect_0_accel = OCCUPIED;
						
						/*
						 * Increment the Value of the segments Variable.
						 */						
						segments++;
					}
					/*
					 * Check if the agi1_busy Flag has Zero Value which Means that AGI1 is Available.
					 */						
					if(inter_process_shared_info_memory->shared_status_flags.agi1_busy == 0)
					{
						/*
						 * Set the indirect_1_accel Flag as Occupied.
						 * This Flag will be Used Later to Lock the AGI1 as Occupied.
						 */								
						indirect_1_accel = OCCUPIED;
						
						/*
						 * Increment the Value of the segments Variable.
						 */						
						segments++;
					}	
					/*
					 * Check if the agi2_busy Flag has Zero Value which Means that AGI2 is Available.
					 */						
					if(inter_process_shared_info_memory->shared_status_flags.agi2_busy == 0)
					{
						/*
						 * Set the indirect_2_accel Flag as Occupied.
						 * This Flag will be Used Later to Lock the AGI2 as Occupied.
						 */								
						indirect_2_accel = OCCUPIED;
						
						/*
						 * Increment the Value of the segments Variable.
						 */						
						segments++;
					}	
					/*
					 * Check if the agi3_busy Flag has Zero Value which Means that AGI3 is Available.
					 */						
					if(inter_process_shared_info_memory->shared_status_flags.agi3_busy == 0)
					{
						/*
						 * Set the indirect_3_accel Flag as Occupied.
						 * This Flag will be Used Later to Lock the AGI3 as Occupied.
						 */								
						indirect_3_accel = OCCUPIED;
						
						/*
						 * Increment the Value of the segments Variable.
						 */						
						segments++;
					}
				}
				/*
				 * Check if the agsg_busy Flag Has Zero Value which Means that AGSG is Available.
				 */
				else if(inter_process_shared_info_memory->shared_status_flags.agsg_busy == 0)
				{
					/*
					 * Set the sg_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGSG as Occupied.
					 */						
					sg_accel = OCCUPIED;

					/*
					 * Increment the Value of the segments Variable.
					 */						
					segments++;
				}						
				
				/*
				 * The shared_repo_virtual_address Points to the Kernel Memory Allocation which is Used so that the Current Thread Can Explicitly Store
				 * Metrics Information fot its own Acceleration Procedure.
				 * 
				 * Set the image_segments Structure Field of the Current Userspace Thread with the Value of the segments Variable.
				 */
				search_element->shared_repo_virtual_address->image_segments = segments;
								
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Assigned Acceleration Group are\n", driver_name, current->pid);
				#endif	
				
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] AGD0 AGD1 AGI0 AGI1 AGI2 AGI3 AGSG\n", driver_name, current->pid);
				#endif		
				
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)]    %d    %d    %d    %d    %d    %d    %d\n", driver_name, current->pid, direct_0_accel, direct_1_accel, indirect_0_accel, indirect_1_accel, indirect_2_accel, indirect_3_accel, sg_accel);
				#endif										

				/*
				 * Get the Number of Rows that each Acceleration Group Should Process.
				 */
				segment_size = search_element->shared_repo_virtual_address->shared_image_info.rows / segments;
				
				/*
				 * Get the Remaining Rows (If any)
				 */
				remaining_rows = search_element->shared_repo_virtual_address->shared_image_info.rows - (segment_size * segments);

				/*
				 * Repeat for as many Times as the Number of Image Segments.
				 */
				for (repeat = 0; repeat < segments; repeat++) 
				{
					/*
					 * Set the Current Array Field with the Number of Rows that the Corresponding Acceleration Group Should Process.
					 */
					segment_rows[repeat] = segment_size;
					
					/*
					 * If we Found Remaining Rows then the Acceleration Group of the Current Array Field Should Process One More Row.
					 */
					if (remaining_rows > 0) 
					{
						/*
						 * Decrement the remaining_rows Value.
						 */
						remaining_rows--;
						
						/*
						 * Increment the Number of Rows of the Current Array Field by 1.
						 */
						segment_rows[repeat]++;
					}
				}				
				#endif
				
				
				/*
				 * This Macro if Condition Encloses Part of the Code that is ONLY Applicable in the Best Available Policy.
				 * 
				 * The Best Available Policy Tries to Occupy a Single Acceleration Group for a Single Image Process for a Single Userspace Thread.
				 * 
				 * It Checks to Occupy the First that is Found Available Starting from the Best Efficient to the Worst Efficient with the Below Priority:
				 * AGD0 --> AGD1 --> AGI0 --> AGI1 --> AGI2 --> AGI3 --> AGSG
				 * 
				 * This Part of the Code is where the Driver Locks a Single Available Acceleration Group so that it Can be Occupied ONLY by the Current Thread.
				 */				
				#ifdef BEST_AVAILABLE
				
				/*
				 * Check if the agd0_busy Flag has Zero Value which Means that AGD0 is Available.
				 */				
				if(inter_process_shared_info_memory->shared_status_flags.agd0_busy == 0)
				{
					/*
					 * Set the direct_0_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGD0 as Occupied.
					 */					
					direct_0_accel = OCCUPIED;
				}
				/*
				 * Else Check if the agd1_busy Flag has Zero Value which Means that AGD1 is Available.
				 */					
				else if(inter_process_shared_info_memory->shared_status_flags.agd1_busy == 0)
				{
					/*
					 * Set the direct_1_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGD1 as Occupied.
					 */						
					direct_1_accel = OCCUPIED;
				}
				/*
				 * Else Check if the agi0_busy Flag has Zero Value which Means that AGI0 is Available.
				 */					
				else if(inter_process_shared_info_memory->shared_status_flags.agi0_busy == 0)
				{
					/*
					 * Set the indirect_0_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGI0 as Occupied.
					 */						
					indirect_0_accel = OCCUPIED;
				}
				/*
				 * Else Check if the agi1_busy Flag has Zero Value which Means that AGI1 is Available.
				 */									
				else if(inter_process_shared_info_memory->shared_status_flags.agi1_busy == 0)
				{
					/*
					 * Set the indirect_1_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGI1 as Occupied.
					 */						
					indirect_1_accel = OCCUPIED;
				}	
				/*
				 * Else Check if the agi2_busy Flag has Zero Value which Means that AGI2 is Available.
				 */									
				else if(inter_process_shared_info_memory->shared_status_flags.agi2_busy == 0)
				{
					/*
					 * Set the indirect_2_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGI2 as Occupied.
					 */						
					indirect_2_accel = OCCUPIED;
				}	
				/*
				 * Else Check if the agi3_busy Flag has Zero Value which Means that AGI3 is Available.
				 */									
				else if(inter_process_shared_info_memory->shared_status_flags.agi3_busy == 0)
				{
					/*
					 * Set the indirect_3_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGI3 as Occupied.
					 */						
					indirect_3_accel = OCCUPIED;
				}
				/*
				 * Else Check if the agsg_busy Flag has Zero Value which Means that AGSG is Available.
				 */									
				else if(inter_process_shared_info_memory->shared_status_flags.agsg_busy == 0)
				{
					/*
					 * Set the sg_accel Flag as Occupied.
					 * This Flag will be Used Later to Lock the AGSG as Occupied.
					 */						
					sg_accel = OCCUPIED;
				}
				
				/*
				 * Set segments Variable with 1 Because in this Policy the Image will not be Processed in Segments.
				 */
				segments = 1;
				
				/*
				 * Set the segment_rows First Array Field with the Number of Image Rows.
				 */
				segment_rows[0] = search_element->shared_repo_virtual_address->shared_image_info.rows;
				
				#endif

				/*
				 * Check if the direct_0_accel Flag is Set as Occupied which Means that AGD0 is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGD0.
				 */
				if(direct_0_accel == OCCUPIED)
				{
					/*
					 * Lock the case_0_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */
					down_write(&case_0_sem);
					
					/*
					 * Set accel_direct_0_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGD0.
					 */
					inter_process_shared_info_memory->shared_status_flags.accel_direct_0_occupied_pid = current->pid;
					
					/*
					 * Set the agd0_busy Flag with Value 1 in order to Lock AGD0 for the Current Thread.
					 */
					inter_process_shared_info_memory->shared_status_flags.agd0_busy	= 1;
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_DIRECT_0_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread and Compared with the accel_completed Mask.
					 * The accel_completed Mask is Set inside the Interrupt Handlers when the Interrupt Manager of the FPGA Sends MSI Completion Interupts.
					 * When both Masks Have the Same Value the Userspace Thread Knows that the Acceleration Has Completed by All the Acceleration Groups that Participated in Processing a Single Image.
					 */
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_DIRECT_0_OCCUPIED;
					
					/*
					 * Unlock the case_0_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */						
					up_write(&case_0_sem);
						
					

					///////////////////////////////////////////////////////////////////////////////////////////
					// Set Up and Start Accelerator Group Direct 0
					///////////////////////////////////////////////////////////////////////////////////////////
					
					/*
					 * Set (through the PCIe Bus) the AXI BAR0 Address Translation Register of the FPGA's PCIe Bridge
					 * with the  Physical Address of the Pre-Process Data Kernel Memory (pre_process_mmap_physical_address).
					 * This Way the DMA of the AGD0 that Uses AXI BAR0 for Accessing the Host Memory Can Directly Target the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_PCIE_CTL + AXI_BAR0_LOWER_ADDRESS_OFFSET, (u32)search_element->pre_process_mmap_physical_address);

					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Direct of the AGD0 with the Host's Source Address where the Pre-Process Image Data is Located which is AXI BAR0.
					 * By Extension AXI BAR0 Targets the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 * 
					 * If the Image is Segmented in order to be Processed by Multiple Acceleration Groups (Greedy Policy) then the Source Address Points to an Offset of AXI BAR0
					 * According to the segment_offset Variable where the Segment that AGD0 will Process is Located.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_HOST_SOURCE_ADDRESS_REGISTER_OFFSET, (u32)(AXI_BAR_0_OFFSET + segment_offset));

					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Direct of the AGD0 with the Host's Destination Address where
					 * the Post-Process Image Data Should be Located which is AXI BAR0.
					 * By Extension AXI BAR0 Targets the Pre-Process Data Kernel Memory which is, also, Used as the Destination Memory for the Post-Processed Data.
					 * Typically, the Driver Creates a Post-Process Data Kernel Memory but Using it Would Require the Usage of Additional AXI BAR.
					 * In Order to Reduce the AXI BARs that are Required for Data Acceleration we Use ONLY the Pre-Process Data Kernel Memory
					 * both to Read the Initial Image Data from and Write the Processed Image Data to.
					 * 
					 * 
					 * If the Image is Segmented in order to be Processed by Multiple Acceleration Groups (Greedy Policy) then the Destination Address Points to an Offset of AXI BAR0
					 * According to the segment_offset Variable where the Segment that AGD0 will Process is Located.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_HOST_DESTINATION_ADDRESS_REGISTER_OFFSET, (u32)(AXI_BAR_0_OFFSET + segment_offset));
					
					/*
					 * Set the FPGA's Acceleration Scheduler Direct of the AGD0 (through the PCIe Bus) with the Number of Image Columns that the AGD0 will Process.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_IMAGE_COLUMNS_REGISTER_OFFSET, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns);

					/*
					 * Set the FPGA's Acceleration Scheduler Direct of the AGD0 (through the PCIe Bus) with the Number of Image Rows that the AGD0 will Process.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_IMAGE_ROWS_REGISTER_OFFSET, (u32)segment_rows[segment_count]);				

					/*
					 * Set the FPGA's Acceleration Scheduler Direct of the AGD0 (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
					 */				
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_0_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_CONTROL_REGISTER_OFFSET, (u32)START);
					
					#ifdef GREEDY
					/*
					 * If we are in Greedy Policy we Have to Calculate the Offset where the Next Image Segment is Located so that the Next Acceleration Group 
					 * Should Know which Image Segment to Process.
					 */
					segment_offset = segment_offset + (segment_rows[segment_count] * search_element->shared_repo_virtual_address->shared_image_info.columns * 4);
					
					/*
					 * Increment the segment_count Variable so that the Next Acceleration Group will Read the Correct Field of the segment_rows Array
					 * in order to Get the Correct Number of Image Rows that it Should Process.
					 */
					segment_count++;
					#endif

					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGD 0\n", driver_name, current->pid);
					#endif
					
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] OFFSET %d\n", driver_name, current->pid, segment_offset);
					#endif					
										
				
				}
				
				/*
				 * Check if the direct_1_accel Flag is Set as Occupied which Means that AGD1 is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGD1.
				 */				
				if(direct_1_accel == OCCUPIED)
				{	
					/*
					 * Lock the case_1_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */					
					down_write(&case_1_sem);
					
					/*
					 * Set accel_direct_1_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGD1.
					 */					
					inter_process_shared_info_memory->shared_status_flags.accel_direct_1_occupied_pid = current->pid;	
					
					/*
					 * Set the agd1_busy Flag with Value 1 in order to Lock AGD1 for the Current Thread.
					 */						
					inter_process_shared_info_memory->shared_status_flags.agd1_busy	= 1;		
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */					
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_DIRECT_1_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread and Compared with the accel_completed Mask.
					 * The accel_completed Mask is Set inside the Interrupt Handlers when the Interrupt Manager of the FPGA Sends MSI Completion Interupts.
					 * When both Masks Have the Same Value the Userspace Thread Knows that the Acceleration Has Completed by All the Acceleration Groups that Participated in Processing a Single Image.
					 */					
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_DIRECT_1_OCCUPIED;
					
					/*
					 * Unlock the case_1_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */						
					up_write(&case_1_sem);
					
					
					
					///////////////////////////////////////////////////////////////////////////////////////////
					// Set Up and Start Accelerator Group Direct 1
					///////////////////////////////////////////////////////////////////////////////////////////
					
					/*
					 * Set (through the PCIe Bus) the AXI BAR1 Address Translation Register of the FPGA's PCIe Bridge
					 * with the  Physical Address of the Pre-Process Data Kernel Memory (pre_process_mmap_physical_address).
					 * This Way the DMA of the AGD1 that Uses AXI BAR1 for Accessing the Host Memory Can Directly Target the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */					
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_PCIE_CTL + AXI_BAR1_LOWER_ADDRESS_OFFSET, (u32)search_element->pre_process_mmap_physical_address);

					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Direct of the AGD1 with the Host's Source Address where the Pre-Process Image Data is Located which is AXI BAR1.
					 * By Extension AXI BAR1 Targets the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 * 
					 * If the Image is Segmented in order to be Processed by Multiple Acceleration Groups (Greedy Policy) then the Source Address Points to an Offset of AXI BAR1
					 * According to the segment_offset Variable where the Segment that AGD1 will Process is Located.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_HOST_SOURCE_ADDRESS_REGISTER_OFFSET, (u32)(AXI_BAR_1_OFFSET + segment_offset));
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Direct of the AGD1 with the Host's Destination Address where
					 * the Post-Process Image Data Should be Located which is AXI BAR1.
					 * By Extension AXI BAR1 Targets the Pre-Process Data Kernel Memory which is, also, Used as the Destination Memory for the Post-Processed Data.
					 * Typically, the Driver Creates a Post-Process Data Kernel Memory but Using it Would Require the Usage of Additional AXI BAR.
					 * In Order to Reduce the AXI BARs that are Required for Data Acceleration we Use ONLY the Pre-Process Data Kernel Memory
					 * both to Read the Initial Image Data from and Write the Processed Image Data to.
					 * 
					 * 
					 * If the Image is Segmented in order to be Processed by Multiple Acceleration Groups (Greedy Policy) then the Destination Address Points to an Offset of AXI BAR1
					 * According to the segment_offset Variable where the Segment that AGD1 will Process is Located.
					 */					
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_HOST_DESTINATION_ADDRESS_REGISTER_OFFSET, (u32)(AXI_BAR_1_OFFSET + segment_offset));
				
					/*
					 * Set the FPGA's Acceleration Scheduler Direct of the AGD1 (through the PCIe Bus) with the Number of Image Columns that the AGD1 will Process.
					 */					
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_IMAGE_COLUMNS_REGISTER_OFFSET, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns);
				
					/*
					 * Set the FPGA's Acceleration Scheduler Direct of the AGD1 (through the PCIe Bus) with the Number of Image Rows that the AGD1 will Process.
					 */				
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_IMAGE_ROWS_REGISTER_OFFSET, (u32)segment_rows[segment_count]);				

					/*
					 * Set the FPGA's Acceleration Scheduler Direct of the AGD1 (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
					 */	
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_DIRECT_1_ACCELERATION_SCHEDULER_DIRECT + ACCELERATION_SCHEDULER_DIRECT_CONTROL_REGISTER_OFFSET, (u32)START);				
					
					#ifdef GREEDY
					/*
					 * If we are in Greedy Policy we Have to Calculate the Offset where the Next Image Segment is Located so that the Next Acceleration Group 
					 * Should Know which Image Segment to Process.
					 */					
					segment_offset = segment_offset + (segment_rows[segment_count] * search_element->shared_repo_virtual_address->shared_image_info.columns * 4);
					
					/*
					 * Increment the segment_count Variable so that the Next Acceleration Group will Read the Correct Field of the segment_rows Array
					 * in order to Get the Correct Number of Image Rows that it Should Process.
					 */					
					segment_count++;
					#endif
					
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGD 1\n", driver_name, current->pid);
					#endif	
					
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] OFFSET %d\n", driver_name, current->pid, segment_offset);
					#endif																
				}		
				
				/*
				 * Check if the indirect_0_accel Flag is Set as Occupied which Means that AGI0 is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGI0.
				 */					
				if(indirect_0_accel == OCCUPIED)
				{	
					/*
					 * Lock the case_2_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */						
					down_write(&case_2_sem);
					
					/*
					 * Set accel_indirect_0_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGI0.
					 */						
					inter_process_shared_info_memory->shared_status_flags.accel_indirect_0_occupied_pid = current->pid;	
					
					/*
					 * Set the agi0_busy Flag with Value 1 in order to Lock AGI0 for the Current Thread.
					 */							
					inter_process_shared_info_memory->shared_status_flags.agi0_busy	= 1;
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */									
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_INDIRECT_0_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread and Compared with the accel_completed Mask.
					 * The accel_completed Mask is Set inside the Interrupt Handlers when the Interrupt Manager of the FPGA Sends MSI Completion Interupts.
					 * When both Masks Have the Same Value the Userspace Thread Knows that the Acceleration Has Completed by All the Acceleration Groups that Participated in Processing a Single Image.
					 */						
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_INDIRECT_0_OCCUPIED;
					
					/*
					 * Unlock the case_2_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */						
					up_write(&case_2_sem);
					
								
					
					///////////////////////////////////////////////////////////////////////////////////////////
					// Set Up and Start Accelerator Group Indirect 0
					///////////////////////////////////////////////////////////////////////////////////////////
					 
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI0 with the Host's Source Address where the Pre-Process Image Data is Located.
					 * The Acceleration Scheduler Indirect will Set the AXI BAR2 Address Translation Register of the FPGA's PCIe Bridge with that Source Address.
					 * Then the CDMA Fetch will Read the Image Data from AXI BAR 2 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */					 
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_FETCH_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));

					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI0 with the Host's Destination Address where the Post-Process Image Data Should be Stored.
					 * 
					 * Typically, the Driver Creates a Post-Process Data Kernel Memory to Store the Processed Data But in order to Reduce Memory Usage the Processed Data are Stored Back to
					 * the Pre-Process Data Kernel Memory.
					 * 
					 * The Acceleration Scheduler Indirect will Set the AXI BAR3 Address Translation Register of the FPGA's PCIe Bridge with that Destination Address.
					 * Then the CDMA Send will Write the Processed Image Data to AXI BAR 3 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */	
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_SEND_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));
					
					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI0 with the Offset of the Source Address where the Image Segment that the AGI0 will Process is Located.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI0 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_FETCH_REGISTER_OFFSET, (u32)segment_offset);

					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI0 with the Offset of the Destination Address
					 * where the Image Segment that the AGI0 will Process Should be Stored.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI0 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_SEND_REGISTER_OFFSET, (u32)segment_offset);
							
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI0 (through the PCIe Bus) with the Number of Image Columns that the AGI0 will Process.
					 */								
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_COLUMNS_REGISTER_OFFSET, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns);

					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI0 (through the PCIe Bus) with the Number of Image Rows that the AGI0 will Process.
					 */	
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_ROWS_REGISTER_OFFSET, (u32)segment_rows[segment_count]);				

					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI0 (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_0_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_CONTROL_REGISTER_OFFSET, (u32)START);				
		
					#ifdef GREEDY
					/*
					 * If we are in Greedy Policy we Have to Calculate the Offset where the Next Image Segment is Located so that the Next Acceleration Group 
					 * Should Know which Image Segment to Process.
					 */						
					segment_offset = segment_offset + (segment_rows[segment_count] * search_element->shared_repo_virtual_address->shared_image_info.columns * 4);
					
					/*
					 * Increment the segment_count Variable so that the Next Acceleration Group will Read the Correct Field of the segment_rows Array
					 * in order to Get the Correct Number of Image Rows that it Should Process.
					 */						
					segment_count++;
					#endif
		
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGI 0\n", driver_name, current->pid);
					#endif								
						
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] OFFSET %d\n", driver_name, current->pid, segment_offset);
					#endif	
				}	
				
				/*
				 * Check if the indirect_1_accel Flag is Set as Occupied which Means that AGI1 is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGI1.
				 */					
				if(indirect_1_accel == OCCUPIED)
				{	
					/*
					 * Lock the case_3_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */					
					down_write(&case_3_sem);
					
					/*
					 * Set accel_indirect_1_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGI1.
					 */							
					inter_process_shared_info_memory->shared_status_flags.accel_indirect_1_occupied_pid = current->pid;			
					
					/*
					 * Set the agi1_busy Flag with Value 1 in order to Lock AGI1 for the Current Thread.
					 */						
					inter_process_shared_info_memory->shared_status_flags.agi1_busy	= 1;
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */								
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_INDIRECT_1_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread and Compared with the accel_completed Mask.
					 * The accel_completed Mask is Set inside the Interrupt Handlers when the Interrupt Manager of the FPGA Sends MSI Completion Interupts.
					 * When both Masks Have the Same Value the Userspace Thread Knows that the Acceleration Has Completed by All the Acceleration Groups that Participated in Processing a Single Image.
					 */						
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_INDIRECT_1_OCCUPIED;
					
					/*
					 * Unlock the case_3_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */						
					up_write(&case_3_sem);
					
												
					
					///////////////////////////////////////////////////////////////////////////////////////////
					// Set Up and Start Accelerator Group Indirect 1
					///////////////////////////////////////////////////////////////////////////////////////////
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI1 with the Host's Source Address where the Pre-Process Image Data is Located.
					 * The Acceleration Scheduler Indirect will Set the AXI BAR2 Address Translation Register of the FPGA's PCIe Bridge with that Source Address.
					 * Then the CDMA Fetch will Read the Image Data from AXI BAR 2 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_FETCH_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI1 with the Host's Destination Address where the Post-Process Image Data Should be Stored.
					 * 
					 * Typically, the Driver Creates a Post-Process Data Kernel Memory to Store the Processed Data But in order to Reduce Memory Usage the Processed Data are Stored Back to
					 * the Pre-Process Data Kernel Memory.
					 * 
					 * The Acceleration Scheduler Indirect will Set the AXI BAR3 Address Translation Register of the FPGA's PCIe Bridge with that Destination Address.
					 * Then the CDMA Send will Write the Processed Image Data to AXI BAR 3 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_SEND_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));
					
					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI1 with the Offset of the Source Address where the Image Segment that the AGI1 will Process is Located.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI1 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */					
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_FETCH_REGISTER_OFFSET, (u32)segment_offset);
					
					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI1 with the Offset of the Destination Address
					 * where the Image Segment that the AGI1 will Process Should be Stored.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI1 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */					
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_SEND_REGISTER_OFFSET, (u32)segment_offset);					
					
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI1 (through the PCIe Bus) with the Number of Image Columns that the AGI1 will Process.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_COLUMNS_REGISTER_OFFSET, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns);
					
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI1 (through the PCIe Bus) with the Number of Image Rows that the AGI1 will Process.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_ROWS_REGISTER_OFFSET, (u32)segment_rows[segment_count]);				

					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI1 (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_1_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_CONTROL_REGISTER_OFFSET, (u32)START);				

					#ifdef GREEDY
					/*
					 * If we are in Greedy Policy we Have to Calculate the Offset where the Next Image Segment is Located so that the Next Acceleration Group 
					 * Should Know which Image Segment to Process.
					 */					
					segment_offset = segment_offset + (segment_rows[segment_count] * search_element->shared_repo_virtual_address->shared_image_info.columns * 4);
					
					/*
					 * Increment the segment_count Variable so that the Next Acceleration Group will Read the Correct Field of the segment_rows Array
					 * in order to Get the Correct Number of Image Rows that it Should Process.
					 */						
					segment_count++;
					#endif

					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGI 1\n", driver_name, current->pid);
					#endif	
					
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] OFFSET %d\n", driver_name, current->pid, segment_offset);
					#endif					
									
				}																	
				
				
				/*
				 * Check if the indirect_2_accel Flag is Set as Occupied which Means that AGI2 is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGI2.
				 */						
				if(indirect_2_accel == OCCUPIED)
				{	
					/*
					 * Lock the case_4_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */						
					down_write(&case_4_sem);
					
					/*
					 * Set accel_indirect_2_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGI2.
					 */						
					inter_process_shared_info_memory->shared_status_flags.accel_indirect_2_occupied_pid = current->pid;		

					/*
					 * Set the agi2_busy Flag with Value 1 in order to Lock AGI2 for the Current Thread.
					 */						
					inter_process_shared_info_memory->shared_status_flags.agi2_busy	= 1;	
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */								
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_INDIRECT_2_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread and Compared with the accel_completed Mask.
					 * The accel_completed Mask is Set inside the Interrupt Handlers when the Interrupt Manager of the FPGA Sends MSI Completion Interupts.
					 * When both Masks Have the Same Value the Userspace Thread Knows that the Acceleration Has Completed by All the Acceleration Groups that Participated in Processing a Single Image.
					 */							
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_INDIRECT_2_OCCUPIED;
					
					/*
					 * Unlock the case_4_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */					
					up_write(&case_4_sem);
																	
					
					///////////////////////////////////////////////////////////////////////////////////////////
					// Set Up and Start Accelerator Group Indirect 2
					///////////////////////////////////////////////////////////////////////////////////////////
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI2 with the Host's Source Address where the Pre-Process Image Data is Located.
					 * The Acceleration Scheduler Indirect will Set the AXI BAR2 Address Translation Register of the FPGA's PCIe Bridge with that Source Address.
					 * Then the CDMA Fetch will Read the Image Data from AXI BAR 2 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_FETCH_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI2 with the Host's Destination Address where the Post-Process Image Data Should be Stored.
					 * 
					 * Typically, the Driver Creates a Post-Process Data Kernel Memory to Store the Processed Data But in order to Reduce Memory Usage the Processed Data are Stored Back to
					 * the Pre-Process Data Kernel Memory.
					 * 
					 * The Acceleration Scheduler Indirect will Set the AXI BAR3 Address Translation Register of the FPGA's PCIe Bridge with that Destination Address.
					 * Then the CDMA Send will Write the Processed Image Data to AXI BAR 3 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_SEND_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));
				
					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI2 with the Offset of the Source Address where the Image Segment that the AGI2 will Process is Located.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI2 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */	
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_FETCH_REGISTER_OFFSET, (u32)segment_offset);
				
					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI2 with the Offset of the Destination Address
					 * where the Image Segment that the AGI2 will Process Should be Stored.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI2 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */					
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_SEND_REGISTER_OFFSET, (u32)segment_offset);
					
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI2 (through the PCIe Bus) with the Number of Image Columns that the AGI2 will Process.
					 */							
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_COLUMNS_REGISTER_OFFSET, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns);
					
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI2 (through the PCIe Bus) with the Number of Image Rows that the AGI2 will Process.
					 */							
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_ROWS_REGISTER_OFFSET, (u32)segment_rows[segment_count]);				

					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI2 (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_2_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_CONTROL_REGISTER_OFFSET, (u32)START);				
			
					#ifdef GREEDY
					/*
					 * If we are in Greedy Policy we Have to Calculate the Offset where the Next Image Segment is Located so that the Next Acceleration Group 
					 * Should Know which Image Segment to Process.
					 */						
					segment_offset = segment_offset + (segment_rows[segment_count] * search_element->shared_repo_virtual_address->shared_image_info.columns * 4);	
					
					/*
					 * Increment the segment_count Variable so that the Next Acceleration Group will Read the Correct Field of the segment_rows Array
					 * in order to Get the Correct Number of Image Rows that it Should Process.
					 */	
					segment_count++;
					#endif
			
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGI 2\n", driver_name, current->pid);
					#endif	
					
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] OFFSET %d\n", driver_name, current->pid, segment_offset);
					#endif					
												
				}	
				
				/*
				 * Check if the indirect_3_accel Flag is Set as Occupied which Means that AGI3 is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGI3.
				 */					
				if(indirect_3_accel == OCCUPIED)
				{	
					/*
					 * Lock the case_5_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */					
					down_write(&case_5_sem);
					
					/*
					 * Set accel_indirect_3_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGI3.
					 */						
					inter_process_shared_info_memory->shared_status_flags.accel_indirect_3_occupied_pid = current->pid;	
					
					/*
					 * Set the agi3_busy Flag with Value 1 in order to Lock AGI3 for the Current Thread.
					 */							
					inter_process_shared_info_memory->shared_status_flags.agi3_busy	= 1;			
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */						
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_INDIRECT_3_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread and Compared with the accel_completed Mask.
					 * The accel_completed Mask is Set inside the Interrupt Handlers when the Interrupt Manager of the FPGA Sends MSI Completion Interupts.
					 * When both Masks Have the Same Value the Userspace Thread Knows that the Acceleration Has Completed by All the Acceleration Groups that Participated in Processing a Single Image.
					 */						
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_INDIRECT_3_OCCUPIED;
					
					/*
					 * Unlock the case_5_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */					
					up_write(&case_5_sem);
																						

					///////////////////////////////////////////////////////////////////////////////////////////
					// Set Up and Start Accelerator Group Indirect 3
					///////////////////////////////////////////////////////////////////////////////////////////
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI3 with the Host's Source Address where the Pre-Process Image Data is Located.
					 * The Acceleration Scheduler Indirect will Set the AXI BAR2 Address Translation Register of the FPGA's PCIe Bridge with that Source Address.
					 * Then the CDMA Fetch will Read the Image Data from AXI BAR 2 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_SOURCE_ADDRESS_FETCH_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));
					
					/* 
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI3 with the Host's Destination Address where the Post-Process Image Data Should be Stored.
					 * 
					 * Typically, the Driver Creates a Post-Process Data Kernel Memory to Store the Processed Data But in order to Reduce Memory Usage the Processed Data are Stored Back to
					 * the Pre-Process Data Kernel Memory.
					 * 
					 * The Acceleration Scheduler Indirect will Set the AXI BAR3 Address Translation Register of the FPGA's PCIe Bridge with that Destination Address.
					 * Then the CDMA Send will Write the Processed Image Data to AXI BAR 3 and by Extension the Pre-Process Data Kernel Memory that Belongs to the Current Userspace Thread.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_DESTINATION_ADDRESS_SEND_REGISTER_OFFSET, (u32)(search_element->pre_process_mmap_physical_address));

					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI3 with the Offset of the Source Address where the Image Segment that the AGI3 will Process is Located.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI3 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */	
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_FETCH_REGISTER_OFFSET, (u32)segment_offset);
					
					/*
					 * Set (through the PCIe Bus) the FPGA's Acceleration Scheduler Indirect of the AGI3 with the Offset of the Destination Address
					 * where the Image Segment that the AGI3 will Process Should be Stored.
					 * 
					 * @note This is Applicable ONLY for the Greedy Policy.
					 * @note The Best Available Policy Assigns a whole Image in the AGI3 so there is no Need for an Offset(The segment_offset Variable Has Zero Value).
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_OFFSET_SEND_REGISTER_OFFSET, (u32)segment_offset);
					
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI3 (through the PCIe Bus) with the Number of Image Columns that the AGI3 will Process.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_COLUMNS_REGISTER_OFFSET, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns);
					
					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI3 (through the PCIe Bus) with the Number of Image Rows that the AGI3 will Process.
					 */						
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_IMAGE_ROWS_REGISTER_OFFSET, (u32)segment_rows[segment_count]);				

					/*
					 * Set the FPGA's Acceleration Scheduler Indirect of the AGI3 (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
					 */
					write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_INDIRECT_3_ACCELERATION_SCHEDULER_INDIRECT + ACCELERATION_SCHEDULER_INDIRECT_CONTROL_REGISTER_OFFSET, (u32)START);				

					#ifdef GREEDY
					/*
					 * If we are in Greedy Policy we Have to Calculate the Offset where the Next Image Segment is Located so that the Next Acceleration Group 
					 * Should Know which Image Segment to Process.
					 */						
					segment_offset = segment_offset + (segment_rows[segment_count] * search_element->shared_repo_virtual_address->shared_image_info.columns * 4);
					
					/*
					 * Increment the segment_count Variable so that the Next Acceleration Group will Read the Correct Field of the segment_rows Array
					 * in order to Get the Correct Number of Image Rows that it Should Process.
					 */						
					segment_count++;
					#endif

					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGI 3\n", driver_name, current->pid);
					#endif	
					
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] OFFSET %d\n", driver_name, current->pid, segment_offset);
					#endif		
									
				}	
				
				/*
				 * Check if the sg_accel Flag is Set as Occupied which Means that AGSG is Assigned to the Current Thread.
				 * If this is the Case then Setup and Start the AGSG.
				 */					
				if(sg_accel == OCCUPIED)
				{		
					/*
					 * Lock the case_6_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
					 */	
					down_write(&case_6_sem);
					
					/*
					 * Set accel_sg_0_occupied_pid Flag with the PID of the Current Thread so that we Later Know which Thread Occupied the AGSG.
					 */						
					inter_process_shared_info_memory->shared_status_flags.accel_sg_0_occupied_pid = current->pid;
										
					/*
					 * Set the agsg_busy Flag with Value 1 in order to Lock AGSG for the Current Thread.
					 */											
					inter_process_shared_info_memory->shared_status_flags.agsg_busy	= 1;
					
					/*
					 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
					 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
					 */					
					search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
					
					/*
					 * Add the ACCELERATOR_SG_OCCUPIED Flag in the accel_occupied Mask of the Metrics Kernel Memory.
					 * 
					 * The Metrics Kernel Memory that the search_element->shared_repo_virtual_address Pointer Refers to is Shared Only with the Current Userspace Thread.
					 * As a Result, the accel_occupied Mask is Read in Polling Mode by the Current Userspace Thread.
					 * 
					 * The Difference Between the AGSG and the Rest Acceleration Groups is that if the accel_occupied Mask is Set with the ACCELERATOR_SG_OCCUPIED Flag
					 * the Current Userspace Thread Has to Make a Request to the Driver to Create Scatter/Gather Lists before Requesting to Occupy the AGSG.
					 */		
					search_element->shared_repo_virtual_address->accel_occupied |= ACCELERATOR_SG_OCCUPIED;
					
					/*
					 * Unlock the case_6_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
					 */						
					up_write(&case_6_sem);
					
												
							
					#ifdef DEBUG_MESSAGES
					printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Request from Process to Allocate Usersapce Memory for Using AGSG\n", driver_name, current->pid);
					#endif	
				
				}														

			}
				 
			/*
			 * Set the search_element Pointer to Point at the Next List Node.
			 */			 
			search_element = search_element->next_pid;
		}
		
				
		break;
	
	/*
	 * This Case is Used when a Userspace Thread Requests to Create Scatter/Gather Lists.
	 * 
	 * The AGSG is Used in Cases where the Initial Image Data are Read Directly from the Userspace Memory and the Processed Data are Written Directly to the Userspace Memory.
	 * The Particularity of a Userspace Memory is that its Allocation is Chunked in Pages of 4K rather than being Contiguous Memory.
	 * In Order for the AGSG to Access the Source and Destination Userspace Memories it Has to be Aware of the Physical Addresses of all the Pages that Constitute the Source and Destination Memories.
	 * This is Accomplished in Driver Level by Creating the Scatter/Gather Lists of the Previous Memories.
	 * 
	 * When a Userspace Thread is Assigned the AGSG it Makes a IOCtl Call with the COMMAND_SET_PAGES Flag in Order to Create the Scatter/Gather Lists for its Source and Destination Memories.
	 */		
	case COMMAND_SET_PAGES:

		/*
		 * Lock the set_pages_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
		 */
		down_write(&set_pages_sem);

		/*
		 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
		 */
		search_element = pid_list_head;
		 
		/*
		 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
		 * 
		 * The Reason for Searching the Singly Linked List at this Point is to Find the List Node that Belongs to the Current Userspace Thread.
		 * The Structure Fields of the Current Thread's Node will be Needed During the Creation of the Scatter/Gather List.
		 */			 
		 while(search_element != NULL)
		 {
			
			/*
			 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
			 * If this is the Case then we Can Proceed to Create the Scatter/Gather Lists.
			 */				
			if(search_element->pid == current->pid)
			{
				
				/*
				 * Calculate the Number of Pages According to the Image Size and the Page Size for the Userspace Source and Destination Memories.
				 * 
				 * For Example, an Image of 1920x1080 Resolution Has Size 8294400 Bytes (1920 x 1080 x 4Bytes).
				 * For an Image Size of 8294400 Bytes and a Page Size of 4096 Bytes we Require 2025 Pages (8294400/4096).
				 */
				buffer_entries_source = search_element->shared_repo_virtual_address->shared_image_info.size / PAGE_SIZE;
				buffer_entries_destination = search_element->shared_repo_virtual_address->shared_image_info.size / PAGE_SIZE;

				
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Set Scatter/Gather Pages\n", driver_name, current->pid);
				#endif	

				/*
				 * The userspace_value is a Variable with Data from the Current Userspace Thread that are Carried Along with the IOCtl Command.
				 * The Data of the userspace_value herein is a Pointer to a Structure of Type struct sg_list_addresses which Contains the Pointers of the Source and Destination Userspace Memories
				 * that the Current Userspace Thread Created in order to Occupy the AGSG.
				 * 
				 * We Cast the Remote Pointer that is Carried by the userspace_value Variable to the Local sg_list_src_dst_addresses Pointer of Structure Type struct sg_list_addresses.
				 * This Way we Can Access the Virtual Addresses of the Userspace Source and Destination Memories in order to Create their Scatter/Gather Lists.
				 */
				sg_list_src_dst_addresses = (struct sg_list_addresses *)userspace_value;
			
			/*
			 * Check to Make Sure that the Pointers of the Userspace Source and Destination Memories is not NULL.
			 * This is the Way to Validate that the Memories are Allocated Succesfully Before Trying to Create Their Scatter/Gather Lists.
			 */
			if(sg_list_src_dst_addresses->sg_list_source_address != NULL && sg_list_src_dst_addresses->sg_list_destination_address != NULL)
			{
				/*
				 * Allocate a Kernel Memory Large Enough to Fit as many Structures of Type struct page as the Number of Pages that we Earlier Calculated in the buffer_entries_source Variable
				 * and Set the buffer_page_array_source to Point at this Memory Allocation.
				 * 
				 * This is a Page Array.
				 */
				buffer_page_array_source = kmalloc(sizeof(struct page *) * buffer_entries_source, GFP_KERNEL);
				
				/*
				 * Allocate a Kernel Memory Large Enough to Fit as many Structures of Type struct page as the Number of Pages that we Earlier Calculated in the buffer_entries_destination Variable
				 * and Set the buffer_page_array_destination to Point at this Memory Allocation.
				 * 
				 * This is a Page Array.
				 */				
				buffer_page_array_destination = kmalloc(sizeof(struct page *) * buffer_entries_destination, GFP_KERNEL);
				
				/*
				 * Lock the mmap_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
				 */					
				down_read(&current->mm->mmap_sem);
				
				/*
				 * Pin the the Source User Pages in Memory.
				 * Upon Successful Completion, the Caller of get_user_pages() Has a Pages Array (buffer_page_array_source) Pointing to the Source Userspace Memory Allocation, which is Locked into Memory.
				 * 
				 * The get_user_pages() Returns the Number of Pages that were Succesfully Pinned(search_element->buffer_mapped_pages_source)
				 * which is not Necessarily Equal to the Requested Pages(buffer_entries_source).
				 */
				search_element->buffer_mapped_pages_source = get_user_pages(current, current->mm, (unsigned long)(sg_list_src_dst_addresses->sg_list_source_address), buffer_entries_source, 1, 1, buffer_page_array_source, NULL);
				
				/*
				 * Pin the the Destination User Pages in Memory.
				 * Upon Successful Completion, the Caller of get_user_pages() Has a Pages Array (buffer_page_array_destination) Pointing to the Destination Userspace Memory Allocation, which is Locked into Memory.
				 * 
				 * The get_user_pages() Returns the Number of Pages that were Succesfully Pinned(search_element->buffer_mapped_pages_destination)
				 * which is not Necessarily Equal to the Requested Pages(buffer_entries_destination).
				 */				
				search_element->buffer_mapped_pages_destination = get_user_pages(current, current->mm, (unsigned long)(sg_list_src_dst_addresses->sg_list_destination_address), buffer_entries_destination, 1, 1, buffer_page_array_destination, NULL);

				/*
				 * Now that we Got the Source Page Array (buffer_page_array_source) we Can Release the Source Pages.
				 * Loop for as Many Times as the Number of Pinned Pages of the Source Userspace Memory.
				 */
				for(repeat = 0; repeat < search_element->buffer_mapped_pages_source; repeat++)	
				{
					/*
					 * Release the Page of the Current Field of the Source Page Array.
					 */
					put_page(buffer_page_array_source[repeat]);
			
				}
				
				/*
				 * Now that we Got the Destination Page Array (buffer_page_array_destination) we Can Release the Destination Pages.
				 * Loop for as Many Times as the Number of Pinned Pages of the Destinaton Userspace Memory.
				 */				
				for(repeat = 0; repeat < search_element->buffer_mapped_pages_destination; repeat++)	
				{
					/*
					 * Release the Page of the Current Field of the Destination Page Array.
					 */
					put_page(buffer_page_array_destination[repeat]);
				}				
					
				/*
				 * Unlock the mmap_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
				 */							
				up_read(&current->mm->mmap_sem);

				/*
				 * Allocate Memory of Size Equal to struct sg_table.
				 * Set the search_element->dma_sg_table_source to Point at this Memory Allocation.
				 */
				search_element->dma_sg_table_source = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
				
				/*
				 * Allocate Memory of Size Equal to struct sg_table.
				 * Set the search_element->dma_sg_table_destinaiton to Point at this Memory Allocation.
				 */				
				search_element->dma_sg_table_destination = kmalloc(sizeof(struct sg_table), GFP_KERNEL);

				/*
				 * The sg_alloc_table Allocates the Memory to Use for the Actual Scatterlist Arrays of the Source Userspace Memory and Deals with the Process of Chaining them all Together.
				 */
				sg_table_value_source = sg_alloc_table(search_element->dma_sg_table_source, search_element->buffer_mapped_pages_source, GFP_KERNEL);
				
				/*
				 * The sg_alloc_table Allocates the Memory to Use for the Actual Scatterlist Arrays of the Destinaiton Userspace Memory and Deals with the Process of Chaining them all Together.
				 */				
				sg_table_value_destination = sg_alloc_table(search_element->dma_sg_table_destination, search_element->buffer_mapped_pages_destination, GFP_KERNEL);
				
				/*
				 * The search_element->dma_sg_table_source->sgl Points to the Memory where the Source Scatter/Gather List will be Stored.
				 * Set the search_element->scatterlist_pointer_source Pointer that Belongs to the Current Thread to Point at the Same Memory where the Source Scatter/Gather List will be Stored.
				 */
				search_element->scatterlist_pointer_source = search_element->dma_sg_table_source->sgl;
				
				/*
				 * The search_element->dma_sg_table_destination->sgl Points to the Memory where the Destination Scatter/Gather List will be Stored.
				 * Set the search_element->scatterlist_pointer_destination Pointer that Belongs to the Current Thread to Point at the Same Memory where the Destination Scatter/Gather List will be Stored.
				 */				
				search_element->scatterlist_pointer_destination = search_element->dma_sg_table_destination->sgl;	
				
				/*
				 * Loop for as Many Times as the Number of Pinned Pages of the Source Userspace Memory.
				 */					
				for(repeat = 0; repeat < search_element->buffer_mapped_pages_source; repeat++)
				{
					/*
					 * Get the Current Page from the Source Page Array and Set Accordingly the Current Scatter/Gather List Entry (search_element->scatterlist_pointer_source) of that Page.
					 */
					sg_set_page(search_element->scatterlist_pointer_source, buffer_page_array_source[repeat], PAGE_SIZE, 0);
					
					/*
					 * Use sg_next() to Walk to the Next Scatter/Gather List Entry that will be Set in the Next Iteration.
					 */
					search_element->scatterlist_pointer_source = sg_next(search_element->scatterlist_pointer_source);
				}
				
		
				/*
				 * Loop for as Many Times as the Number of Pinned Pages of the Destination Userspace Memory.
				 */					
				for(repeat = 0; repeat < search_element->buffer_mapped_pages_destination; repeat++)
				{
					/*
					 * Get the Current Page from the Destination Page Array and Set Accordingly the Current Scatter/Gather List Entry (search_element->scatterlist_pointer_destinaiton) of that Page.
					 */					
					sg_set_page(search_element->scatterlist_pointer_destination, buffer_page_array_destination[repeat], PAGE_SIZE, 0);
					
					/*
					 * Use sg_next() to Walk to the Next Scatter/Gather List Entry that will be Set in the Next Iteration.
					 */					
					search_element->scatterlist_pointer_destination = sg_next(search_element->scatterlist_pointer_destination);
				}		
				
				/*
				 * The Usage of sg_next() Made the search_element->scatterlist_pointer_source Pointer to Point at the Last Scatter/Gather List Entry.
				 * So, Set the search_element->scatterlist_pointer_source to Point again at the Beginning of the Scatter/Gathet List (search_element->dma_sg_table_source->sgl)
				 */
				search_element->scatterlist_pointer_source = search_element->dma_sg_table_source->sgl;
				
				/*
				 * The Usage of sg_next() Made the search_element->scatterlist_pointer_destination Pointer to Point at the Last Scatter/Gather List Entry.
				 * So, Set the search_element->scatterlist_pointer_destination to Point again at the Beginning of the Scatter/Gathet List (search_element->dma_sg_table_destination->sgl)
				 */				
				search_element->scatterlist_pointer_destination = search_element->dma_sg_table_destination->sgl;
				
				/*
				 * Use dma_map_sg() which Fills the dma_address Field of each Entry of the Source Scatter/Gather List with the Physical Address of each Page of the Source Userspace Memory.
				 * The Physical Address Can be Later Passed to the AGSG in Order to Access the Userspace Source Memory.
				 */
				search_element->buffer_dma_buffers_source = dma_map_sg(&dev->dev, search_element->scatterlist_pointer_source, search_element->buffer_mapped_pages_source, DMA_BIDIRECTIONAL);

				/*
				 * Use dma_map_sg() which Fills the dma_address Field of each Entry of the Destination Scatter/Gather List with the Physical Address of each Page of the Destination Userspace Memory.
				 * The Physical Address Can be Later Passed to the AGSG in Order to Access the Userspace Destination Memory.
				 */
				search_element->buffer_dma_buffers_destination = dma_map_sg(&dev->dev, search_element->scatterlist_pointer_destination, search_element->buffer_mapped_pages_destination, DMA_BIDIRECTIONAL);
				
				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] DMA Source SG Pages Number: %d\n", driver_name, current->pid, search_element->buffer_dma_buffers_source);
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] DMA Destination SG Pages Number: %d\n", driver_name, current->pid, search_element->buffer_dma_buffers_destination);
				#endif
				
				/*
				 * Clear the Pages Array of the Source Userspace Memory as it is no Longer Needed.
				 */
				kfree(buffer_page_array_source);
				
				/*
				 * Clear the Pages Array of the Destination Userspace Memory as it is no Longer Needed.
				 */				
				kfree(buffer_page_array_destination);
				
				/*
				 * Allocate Memory (64K) that will be Used as an Array to Store all the Physical Addresses of the Pages that Belong to the Source Userspace Memory.
				 * The Allocation Size is Large Enough to Support Images of 4K Resolution that Require 8100 Pages.
				 * 
				 * The Physical Addresses of this Allocation will be Copied in the FPGA BRAM so that the AGSG Can Use them to Fetch the Initial Image Data from the Source Userspace Memory.
				 */
				search_element->u64_sg_list_source = (uint64_t *)kmalloc( 64 * KBYTE, GFP_KERNEL);
				
				/*
				 * Allocate Memory (64K) that will be Used as an Array to Store all the Physical Addresses of the Pages that Belong to the Destination Userspace Memory.
				 * The Allocation Size is Large Enough to Support Images of 4K Resolution that Require 8100 Pages.
				 * 
				 * The Physical Addresses of this Allocation will be Copied in the FPGA BRAM so that the AGSG Can Use them to Send the Processed Image Data to the Destination Userspace Memory.
				 */				
				search_element->u64_sg_list_destination = (uint64_t *)kmalloc( 64 * KBYTE, GFP_KERNEL);


				for(repeat = 0; repeat < search_element->buffer_dma_buffers_source; repeat++)
				{
					/*
					 * Use sg_dma_address() to Get the Physical Address of the Source Page of the Current Source Scatter/Gather List Entry.
					 * Store the Physical Address to the Current Field of the search_element->u64_sg_list_source Array Pointer.
					 */
					search_element->u64_sg_list_source[repeat] = sg_dma_address(search_element->scatterlist_pointer_source);
					
					/*
					 * The sg_dma_len() is Useful If we Needed to Know the Data Size in Each Page.
					 * It is not Currently Used but it is Kept for Future Implementations.
					 */
					//sg_list_length_source_base_address[repeat] = sg_dma_len(scatterlist_pointer_source);
				
					/*
					 * Use sg_next() to Walk to the Next Scatter/Gather List Entry.
					 */					
					search_element->scatterlist_pointer_source = sg_next(search_element->scatterlist_pointer_source);
							
				}					
				
				
				for(repeat = 0; repeat < search_element->buffer_dma_buffers_destination; repeat++)
				{
					/*
					 * Use sg_dma_address() to Get the Physical Address of the Destination Page of the Current Destination Scatter/Gather List Entry.
					 * Store the Physical Address to the Current Field of the search_element->u64_sg_list_destination Array Pointer.
					 */					
					search_element->u64_sg_list_destination[repeat] = sg_dma_address(search_element->scatterlist_pointer_destination);
					
					/*
					 * The sg_dma_len() is Useful If we Needed to Know the Data Size in Each Page.
					 * It is not Currently Used but it is Kept for Future Implementations.
					 */					
					//sg_list_length_destination_base_address[repeat] = sg_dma_len(scatterlist_pointer_destination);
				
					/*
					 * Use sg_next() to Walk to the Next Scatter/Gather List Entry.
					 */					
					search_element->scatterlist_pointer_destination = sg_next(search_element->scatterlist_pointer_destination);
							
				}	
	
				/*
				 * The Usage of sg_next() Made the search_element->scatterlist_pointer_source Pointer to Point at the Last Scatter/Gather List Entry.
				 * So, Set the search_element->scatterlist_pointer_source to Point again at the Beginning of the Scatter/Gathet List (search_element->dma_sg_table_source->sgl)
				 */	
				search_element->scatterlist_pointer_source = search_element->dma_sg_table_source->sgl;
				
				/*
				 * The Usage of sg_next() Made the search_element->scatterlist_pointer_destination Pointer to Point at the Last Scatter/Gather List Entry.
				 * So, Set the search_element->scatterlist_pointer_destination to Point again at the Beginning of the Scatter/Gathet List (search_element->dma_sg_table_destination->sgl)
				 */					
				search_element->scatterlist_pointer_destination = search_element->dma_sg_table_destination->sgl;
				
				
				/*
				 * Use pci_dma_sync_sg_for_device() that Synchronizes the Source Userspace Memory so that the Device Can See the Most Up to Date Data.
				 * This Step Should Take Place Before Actually Giving the Physical Addresses of the Scatter/Gather List to the Hardware (AGSG).
				 */
				pci_dma_sync_sg_for_device(dev, search_element->scatterlist_pointer_source, search_element->buffer_dma_buffers_source, PCI_DMA_TODEVICE);
				
				/*
				 * Use pci_dma_sync_sg_for_device() that Synchronizes the Destination Userspace Memory so that the Device Can See the Most Up to Date Data.
				 * This Step Should Take Place Before Actually Giving the Physical Addresses of the Scatter/Gather List to the Hardware (AGSG).
				 */				
				pci_dma_sync_sg_for_device(dev, search_element->scatterlist_pointer_destination, search_element->buffer_dma_buffers_destination, PCI_DMA_TODEVICE);
								
				}				
			}
			
			/*
			 * Set the search_element Pointer to Point at the Next List Node.
			 */			
			search_element = search_element->next_pid;			
		}
		
		/*
		 * Unlock the set_pages_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
		 */			
		up_write(&set_pages_sem);
	
		
		break;	
		
		
		/*
		 * This Case is Used when the Current Thread of the Userspace Application Requests to Unmap the Pages that were Mapped
		 * when Creating the Scatter/Gather Lists for the Need of the Current Thread.
		 * 
		 * It, also, Releases the Scatter/Gather Lists and any Resources that were Required for the Scatter/Gather Operations of the Current Thread.
		 */			
		case COMMAND_UNMAP_PAGES:
	
		/*
		 * Lock the unmap_pages_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
		 */		
		down_write(&unmap_pages_sem);
		
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Going to Unmap Scatter/Gather Pages\n", driver_name, current->pid);
		#endif	
		
		/*
		 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
		 */		
		search_element = pid_list_head;
		 
		/*
		 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
		 * 
		 * The Reason for Searching the Singly Linked List at this Point is to Find the List Node that Belongs to the Current Userspace Thread.
		 * The Structure Fields of the Current Thread's Node will be Needed in order to Unmap the Pages from the Kernel Space and Free the Scatter/Gather Lists and the Resources
		 * that were Created when Setting the Scatter/Gather Lists for the Userspace Memories of the Current Thread.
		 */			 
		 while(search_element != NULL)
		 {
			/*
			 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
			 * If this is the Case then we Can Proceed to Unmapping and Releasing.
			 */				
			if(search_element->pid == current->pid)
			{		
				/*
				 * Unmap the Scatter/Gather List of the Userspace Source Memory.
				 * If we Avoid Unmapping, the Userspace Thread will not be able to Use the Source Memory Correctly.
				 */
				dma_unmap_sg(&dev->dev, search_element->scatterlist_pointer_source, search_element->buffer_mapped_pages_source, DMA_BIDIRECTIONAL);
				
				/*
				 * Unmap the Scatter/Gather List of the Userspace Destination Memory.
				 * If we Avoid Unmapping, the Userspace Thread will not be able to Use the Destination Memory Correctly.
				 */				
				dma_unmap_sg(&dev->dev, search_element->scatterlist_pointer_destination, search_element->buffer_mapped_pages_destination, DMA_BIDIRECTIONAL);
				
				/*
				 * Free the Scatter/Gather List Table of the Source Userspace Memory.
				 */
				sg_free_table(search_element->dma_sg_table_source);
				
				/*
				 * Free the Scatter/Gather List Table of the Destination Userspace Memory.
				 */				
				sg_free_table(search_element->dma_sg_table_destination);
				
				/*
				 * Free the Memory Allocation where the Scatter/Gather List Table of the Source Memory was Stored.
				 */
				kfree(search_element->dma_sg_table_source);
				
				/*
				 * Free the Memory Allocation where the Scatter/Gather List Table of the Destination Memory was Stored.
				 */				
				kfree(search_element->dma_sg_table_destination);
				
				/*
				 * Free the Memory Allocation where the 64 Bit Physical Addresses of the Pages of the Source Userspace Memory were Stored.
				 */
				kfree(search_element->u64_sg_list_source);
				
				/*
				 * Free the Memory Allocation where the 64 Bit Physical Addresses of the Pages of the Destination Userspace Memory were Stored.
				 */				
				kfree(search_element->u64_sg_list_destination);
					
			}
			
			/*
			 * Set the search_element Pointer to Point at the Next List Node.
			 */					
			search_element = search_element->next_pid;
			
		}	
		
		/*
		 * Unlock the unmap_pages_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
		 */			
		up_write(&unmap_pages_sem);
				
		break;	
	
		/*
		 * This Case is Used when a Userspace Thread Requests to Reset the Acceleration Flags that are Used to Indicate which Threads (PIDs) Use the Acceleration Groups.
		 * It is, also, Used to Reset the accelerator_busy Mask.
		 */			
		case COMMAND_RESET_VARIABLES:
		
		
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Going to Reset Driver Variables\n", driver_name, current->pid);
		#endif	
		
		/*
		 * Reset the accelerator_busy Mask.
		 */
		inter_process_shared_info_memory->shared_status_flags.accelerator_busy = 0;

		/*
		 * Reset the accel_direct_0_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGD0.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accel_direct_0_occupied_pid = 0;
		
		/*
		 * Reset the accel_direct_1_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGD1.
		 */				
		inter_process_shared_info_memory->shared_status_flags.accel_direct_1_occupied_pid = 0;
		
		/*
		 * Reset the accel_indirect_0_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGI0.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accel_indirect_0_occupied_pid = 0;
		
		/*
		 * Reset the accel_indirect_1_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGI1.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accel_indirect_1_occupied_pid = 0;
		
		/*
		 * Reset the accel_indirect_2_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGI2.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accel_indirect_2_occupied_pid = 0;
		
		/*
		 * Reset the accel_indirect_3_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGI3.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accel_indirect_3_occupied_pid = 0;
				
		/*
		 * Reset the accel_sg_0_occupied_pid Flag that Indicates which Thread (PID) Occupies the AGSG.
		 */		
		inter_process_shared_info_memory->shared_status_flags.accel_sg_0_occupied_pid = 0;
				
		break;		
		
	/*
	 * This IOCtl Call is Made After the Current Thread Has Called the COMMAND_REQUEST_ACCELERATOR_ACCESS IOCtl Call and the Driver Assigned AGSG to the Current Thread.
	 * Then, the Current Thread Had to Make the COMMAND_SET_PAGES IOCtl Call to Create the Scatter/Gather Lists that are Required for the AGSG to Operate.
	 * Finally the Current Userspace Thread Makes the COMMAND_REQUEST_ACCELERATOR_SG_ACCESS IOCtl Call in order to Start the AGSG.
	 */		
	case COMMAND_REQUEST_ACCELERATOR_SG_ACCESS:

		/*
		 * Lock the sg_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
		 */
		down_write(&sg_sem);
		
		/*
		 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
		 */						
		search_element = pid_list_head;
		 
		/*
		 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
		 * 
		 * The Reason for Searching the Singly Linked List at this Point is to Find the List Node that Belongs to the Current Userspace Thread.
		 * The Structure Fields of the Current Thread's Node will be Needed in order to Get the Source and Destination Scatter/Gather Lists of the Current Thread.
		 * The Scatter/Gather Lists will be Transferred to the FPGA's BRAM so that the AGSG Can Use them to Start the Acceleration.
		 */			 
		 while(search_element != NULL)
		 {
			
			/*
			 * Check if the Current Node's PID Value is Equal to the Current Userspace Thread's PID.
			 * If this is the Case then we Can Proceed to Acceleration.
			 */					
			if(search_element->pid == current->pid)
			{

				/*
				 * Lock the case_6_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
				 */				
				down_write(&case_6_sem);
				
				/*
				 * Read a 64 Bit Time Value from the FPGA's Shared Timer (Shared APM) which is the Time Moment that the Sleep State (If the Thread was in Sleep State) of the Current Thread Has Ended.
				 * Store this Time Value in the Metrics Structure which is inside the Metrics Kernel Memory of the Current Thread.
				 */					
				search_element->shared_repo_virtual_address->process_metrics.sleep_time_end = readq((u64 *)bar0_address_virtual + BAR0_OFFSET_TIMER / 8);
				
				/*
				 * Unlock the case_6_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
				 */					
				up_write(&case_6_sem);
		
				/////////////////////////////////////////////////////////////////////////////////////////
				//Set Up and Start Accelerator Group SG
				/////////////////////////////////////////////////////////////////////////////////////////
	
				/*
				 * Loop for as Many Times as the Number of Pinned Pages of the Source Userspace Memory.
				 */			
				for(repeat = 0; repeat < search_element->buffer_dma_buffers_source; repeat++)
				{
					/*
					 * Write the 64 Bit SG List Source Addresses of the Current Thread to FPGA BRAM at 32K Offset.
					 */ 					 
					writeq(search_element->u64_sg_list_source[repeat], (u64 *) bar1_address_virtual + repeat + 8192);					
				}	
				
				
				/*
				 * Loop for as Many Times as the Number of Pinned Pages of the Destination Userspace Memory.
				 */						
				for(repeat = 0; repeat < search_element->buffer_dma_buffers_destination; repeat++)
				{
					/*
					 * Write the 64 Bit SG List Destination Addresses of the Current Thread to FPGA BRAM at 64K Offset.
					 */ 	
					writeq(search_element->u64_sg_list_destination[repeat], (u64 *) bar1_address_virtual + repeat + 16384);						 
					 						
				}

				/*
				 * Set the FPGA's Acceleration Scheduler Scatter/Gather of the AGSG (through the PCIe Bus) with the Number of Image Columns that the AGSG will Process.
				 */		
				write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG + XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_IMAGE_COLS_DATA, (u32)search_element->shared_repo_virtual_address->shared_image_info.columns); 

				/*
				 * Set the FPGA's Acceleration Scheduler Scatter/Gather of the AGSG (through the PCIe Bus) with the Number of Image Rows that the AGSG will Process.
				 */	
				write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG + XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_IMAGE_ROWS_DATA, (u32)search_element->shared_repo_virtual_address->shared_image_info.rows); 							
			
				/*
				 * Set the FPGA's Acceleration Scheduler Scatter/Gather of the AGSG (through the PCIe Bus) with the START Flag in Order to Start the Acceleration Procedure.
				 */					
				write_remote_register(bar0_address_virtual, BAR0_OFFSET_ACCEL_GROUP_SG_ACCELERATION_SCHEDULER_SG + XACCELERATION_SCHEDULER_SG_XDMA_CFG_ADDR_AP_CTRL, (u32)START);

				#ifdef DEBUG_MESSAGES
				printk(KERN_ALERT "[%s-DBG -> UNLOCKED IOCTL (PID %d)] Sending Start Request to AGSG\n", driver_name, current->pid);
				#endif

			}
			 	
			/*
			 * Set the search_element Pointer to Point at the Next List Node.
			 */					 			 
			search_element = search_element->next_pid;
		}
		
		/*
		 * Unlock the sg_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
		 */		
		up_write(&sg_sem);
		
		break;	
		
	

	default:
		break;
}

/*
 * Unlock the ioctl_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
 */
up_write(&ioctl_sem);

return(SUCCESS);
}

/** OK
  * The xilinx_kc705_driver_file_operations Structure Indicates which Driver Function Routines Correspond (Called) to the 
  * File Operations that a Userspace Application Makes on the Driver File.
  * 
  * @note Older Versions of the Driver, also, Used the Write and Read File Operations which are no Longer Needed Since only DMAs Make Data Transfers.
  */
struct file_operations xilinx_kc705_driver_file_operations = {
    unlocked_ioctl: xilinx_pci_driver_unlocked_ioctl,
    open:           xilinx_pci_driver_open,
    release:        xilinx_pci_driver_release,
};




/** OK
  * shared_repo_open()
  * 
  * It is Called when a Userspace Application Opens the shared_repo_mmap_value Debugfs File.
  */
int shared_repo_open(struct inode *inode, struct file *file_pointer)
{	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> SHARED REPO OPEN (PID %d)] Opening Shared Repo File\n", driver_name, current->pid);
	#endif

	return SUCCESS;
}

/** OK
  * shared_repo_release()
  * 
  * It is Called when a Userspace Application Releases the shared_repo_mmap_value Debugfs File.
  */
int shared_repo_release(struct inode *inode, struct file *file_pointer)
{
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> SHARED REPO RELEASE (PID %d)] Releasing Shared Repo File\n", driver_name, current->pid);
	#endif
	
	return(SUCCESS);
}

/** OK
  * shared_repo_mmap()
  * 
  * It is Called when a Userspace Application Makes a Mmap File Operation to the shared_repo_mmap_value Debugfs File.
  * By Calling the shared_repo_mmap() a Userspace Application Intends to Map a Kernel Space Memory Allocation to Userspace.
  */
static int shared_repo_mmap(struct file *file, struct vm_area_struct *vma)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;
	
	/*
	 * The Virtual Address Pointer of the Kernel Memory that will be Allocated by the dma_alloc_coherent().
	 */
	u64 *shared_repo_mmap_virtual_address = NULL;
	
	/*
	 * The Physical Address of the Kernel Memory that will be Allocated by the dma_alloc_coherent().
	 */	
	dma_addr_t shared_repo_mmap_physical_address;
	
	int mmap_return_value;

	/*
	 * Get the Size of Kernel Memory that the Userspace Application Requested to Map.
	 */
    long length = vma->vm_end - vma->vm_start;
    
    
  	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] Going to MMAP Kernel Memory of Size %ld\n",driver_name, current->pid, (unsigned long)length);
	#endif  
    
	/*
	 * The dma_alloc_coherent() is Used to Allocate Physically Contiguous Consistent Memory which is Suitable for DMA Operations.
	 * Consistent Memory Refers to Write Operations by either the Device or the Processor that can Immediately be Read by the Processor or Device
	 * without Having to Worry about Caching Effects.
	 * 
	 * The dma_alloc_coherent() herein is Called to Allocate 4M of Kernel Contiguous Memory that will be Used to Store/Gather Metrics from the FPGA, the Kernel Driver and the Userspace Application.
	 * 
	 * The dma_alloc_coherent() Returns a Pointer (shared_repo_mmap_virtual_address) with the Virtual Address of the Allocated Memory.
	 * It, also, Returns the shared_repo_mmap_physical_address Pointer with the Physical Address of the Same Allocated Memory.
	 * 
	 * The Physical Address will be Used by the FPGA Peripherals (DMA, Microblaze etc) to Access the Kernel Memory.
	 * The Virtual Address will be Used by the Kernel Driver and the Userspace Application to Access the Kernel Memory.
	 */
	shared_repo_mmap_virtual_address = dma_alloc_coherent( &dev->dev, MMAP_ALLOCATION_SIZE, &shared_repo_mmap_physical_address, GFP_ATOMIC);

	/*
	 * If the Returned Value of the shared_repo_mmap_virtual_address Pointer is NULL then the dma_alloc_coherent() Failed to Allocate Memory.
	 */
	if(shared_repo_mmap_virtual_address == NULL)
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] Allocating MMAP Coherent Memory [FAILURE]\n",driver_name, current->pid);
		#endif	
		
		return FAILURE;			
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] Allocating MMAP Coherent Memory (Virtual 0x%016lX)(Physical 0x%016lX)[SUCCESS]\n",driver_name, current->pid, (unsigned long)shared_repo_mmap_virtual_address, (unsigned long)shared_repo_mmap_physical_address);
		#endif
	}
 
	
	/*
	 * Do Not Allow Larger Mappings than the Number of Allocated Pages in the Kernel.
	 */ 
	 
	if (length > KERNEL_ALLOCATION_SIZE)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] Cannot MMAP Kernel Memory [Process Requested Larger Number of Pages than the Allocated Ones]\n", driver_name, current->pid);
		#endif
       
		return FAILURE;
	}
	
	
	/*
	 * If Architecture Supports dma_mmap_coherent().
	 */ 
	if (vma->vm_pgoff == 0) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] Going to MMAP with dma_mmap_coherent()\n", driver_name, current->pid);
		#endif
		
		/*
		 * The dma_mmap_coherent() is Used to Map the Kernel Memory to Userspace as Non-Cacheable.
		 * It Requires the Virtual and Physical Addresses as well as the Length of the Memory that we Want to Map to Userspace.
		 */
		mmap_return_value = dma_mmap_coherent(&dev->dev, vma, shared_repo_mmap_virtual_address, shared_repo_mmap_physical_address, length);
	} 
	/*
	 * If Architecture Does not Support dma_mmap_coherent() Use the remap_pfn_range().
	 */ 	
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] Going to MMAP with remap_pfn_range()\n", driver_name, current->pid);
		#endif		
		
		/*
		 * Set the Memory Area as Non-Cacheable.
		 */		
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		
		/*
		 * Set the vm_flags with the VM_IO Flag.
		 * The VM_IO flag specifies that this memory area is a mapping of a device's I/O space.
		 * It, also, Specifies, among Other Things, that the Memory Allocated Area Must not be Included in any Process's Core Dump.
		 */
		vma->vm_flags |= VM_IO;

		/*
		 * Kernel Memory Has a Page Table Entry with an Architecture Specific Bit that Defines that this Page Table Entry is Only Valid while the CPU is in Kernel Mode.
		 * The remap_pfn_range() Creates Another Page Table Entry, with a Different Virtual Address to the Same Physical Memory Page that Does not Have that Bit Set.
		 * As s Result, by Using the New Virtual Address the Userspace Application is Capable of Accessing the Kernel Memory Allocation.
		 */
		mmap_return_value = remap_pfn_range(vma, vma->vm_start, PFN_DOWN(virt_to_phys(bus_to_virt(shared_repo_mmap_physical_address))) + vma->vm_pgoff, length, vma->vm_page_prot);	
	}	
	
	/*
	 * If mmap_return_value is Less than 0 then Mmap Failed.
	 */
	if (mmap_return_value < 0) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] MMAP [FAILURE]: %d\n", driver_name, current->pid, mmap_return_value);
		#endif		
		
		return mmap_return_value;
     } 
     else
     {
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SHARED REPO MMAP (PID %d)] MMAP [SUCCESS]\n", driver_name, current->pid);
		#endif	
	 }     
	 
	/*
	 * Lock the shared_repo_mmap_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&shared_repo_mmap_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */		
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */		 
	 while(search_element != NULL)
	 {
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Made the MMap File Operation Call.
		 */		
		if(search_element->pid == current->pid)
		{
			/*
			 * Save the shared_repo_mmap_virtual_address Virtual Pointer inside the Singly Linked List Node of the Current PID so that we Know that the Kernel Allocated Memory Belongs to the Current PID.
			 * From this Moment the Virtual Address of that Kernel Memory can be Accessed only through the Current Singly List Node and Only on Behalf of the PID that this Node Belongs to.
			 */
			search_element->shared_repo_virtual_address = (struct shared_repository_process *)shared_repo_mmap_virtual_address;
			
			/*
			 * Save the shared_repo_mmap_physical_address Physical Address inside the Singly Linked List Node of the Current PID so that we Know that the Kernel Allocated Memory Belongs to the Current PID.
			 * From this Moment the Physical Address of that Kernel Memory can be Accessed only through the Current Singly List Node and Only on Behalf of the PID that this Node Belongs to.
			 */			
			search_element->shared_repo_physical_address = (u32)shared_repo_mmap_physical_address;
			
			break;
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */				 
		search_element = search_element->next_pid;
	}
	
	/*
	 * Unlock the shared_repo_mmap_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */	
	up_write(&shared_repo_mmap_sem);	 
 
	       
    return SUCCESS;
}

/** OK
  * The shared_repo_ops Structure Indicates which Driver Function Routines Correspond (Called) to the 
  * File Operations that a Userspace Application Makes on the shared_repo_mmap_value Debugfs File.
  */
struct file_operations shared_repo_ops = {
	open:			shared_repo_open,
	release:		shared_repo_release,
	mmap:			shared_repo_mmap,
};




/** OK
  * pre_process_mmap_open()
  * 
  * It is Called when a Userspace Application Opens the pre_process_mmap_value Debugfs File.
  */
int pre_process_mmap_open(struct inode *inode, struct file *file_pointer)
{	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS OPEN (PID %d)] Opening Pre-Processed File\n", driver_name, current->pid);
	#endif

	return SUCCESS;	
}

/** OK
  * pre_process_mmap_release()
  * 
  * It is Called when a Userspace Application Releases the pre_process_mmap_value Debugfs File.
  */
int pre_process_mmap_release(struct inode *inode, struct file *file_pointer)
{
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS RELEASE (PID %d)] Releasing Pre-Processed File\n", driver_name, current->pid);
	#endif
	
	return(SUCCESS);	
}

/** OK
  * pre_process_mmap_mmap()
  * 
  * It is Called when a Userspace Application Makes a Mmap File Operation to the pre_process_mmap_value Debugfs File.
  * By Calling the pre_process_mmap_mmap() a Userspace Application Intends to Map a Kernel Space Memory Allocation to Userspace.
  */
static int pre_process_mmap_mmap(struct file *file, struct vm_area_struct *vma)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;
	
	/*
	 * The Virtual Address Pointer of the Kernel Memory that will be Allocated by the dma_alloc_coherent().
	 */	
	u64 *pre_process_mmap_virtual_address = NULL;
	
	/*
	 * The Physical Address of the Kernel Memory that will be Allocated by the dma_alloc_coherent().
	 */		
	dma_addr_t pre_process_mmap_physical_address;
	
	int mmap_return_value;

	/*
	 * Get the Size of Kernel Memory that the Userspace Application Requested to Map.
	 */
    long length = vma->vm_end - vma->vm_start;
    
    
  	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] Going to MMAP Kernel Memory of Size %ld\n",driver_name, current->pid, (unsigned long)length);
	#endif  
    
	/*
	 * The dma_alloc_coherent() is Used to Allocate Physically Contiguous Consistent Memory which is Suitable for DMA Operations.
	 * Consistent Memory Refers to Write Operations by either the Device or the Processor that can Immediately be Read by the Processor or Device
	 * without Having to Worry about Caching Effects.
	 * 
	 * The dma_alloc_coherent() herein is Called to Allocate 4M of Kernel Contiguous Memory that will be Used by the Userspace Application to Directly Load the Image Data Before Processing.
	 * 
	 * The dma_alloc_coherent() Returns a Pointer (pre_process_mmap_virtual_address) with the Virtual Address of the Allocated Memory.
	 * It, also, Returns the pre_process_mmap_physical_address Pointer with the Physical Address of the Same Allocated Memory.
	 * 
	 * The Physical Address will be Used by the FPGA Peripherals (DMA, Microblaze etc) to Access the Kernel Memory.
	 * The Virtual Address will be Used by the Kernel Driver and the Userspace Application to Access the Kernel Memory.
	 */
	pre_process_mmap_virtual_address = dma_alloc_coherent ( &dev->dev, MMAP_ALLOCATION_SIZE, &pre_process_mmap_physical_address, GFP_ATOMIC);

	/*
	 * If the Returned Value of the pre_process_mmap_virtual_address Pointer is NULL then the dma_alloc_coherent() Failed to Allocate Memory.
	 */	
	if(pre_process_mmap_virtual_address == NULL)
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] Allocating MMAP Coherent Memory [FAILURE]\n",driver_name, current->pid);
		#endif	
		
		return FAILURE;			
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] Allocating MMAP Coherent Memory (Virtual 0x%016lX)(Physical 0x%016lX)[SUCCESS]\n",driver_name, current->pid, (unsigned long)pre_process_mmap_virtual_address, (unsigned long)pre_process_mmap_physical_address);
		#endif
	}
 
	
	/*
	 * Do Not Allow Larger Mappings than the Number of Allocated Pages in the Kernel
	 */ 	 
	if (length > KERNEL_ALLOCATION_SIZE)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] Cannot MMAP Kernel Memory [Process Requested Larger Number of Pages than the Allocated Ones]\n", driver_name, current->pid);
		#endif
       
		return FAILURE;
	}
	
	
	/*
	 * If Architecture Supports dma_mmap_coherent().
	 */ 
	if (vma->vm_pgoff == 0) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] Going to MMAP with dma_mmap_coherent()\n", driver_name, current->pid);
		#endif
		
		/*
		 * The dma_mmap_coherent() is Used to Map the Kernel Memory to Userspace as Non-Cacheable.
		 * It Requires the Virtual and Physical Addresses as well as the Length of the Memory that we Want to Map to Userspace.
		 */		
		mmap_return_value = dma_mmap_coherent(&dev->dev, vma, pre_process_mmap_virtual_address, pre_process_mmap_physical_address, length);
	} 
	/*
	 * If Architecture Does not Support dma_mmap_coherent() Use the remap_pfn_range().
	 */ 	
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] Going to MMAP with remap_pfn_range()\n", driver_name, current->pid);
		#endif		
			
		/*
		 * Set the Memory Area as Non-Cacheable.
		 */						
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		
		/*
		 * Set the vm_flags with the VM_IO Flag.
		 * The VM_IO flag specifies that this memory area is a mapping of a device's I/O space.
		 * It, also, Specifies, among Other Things, that the Memory Allocated Area Must not be Included in any Process's Core Dump.
		 */		
		vma->vm_flags |= VM_IO;

		/*
		 * Kernel Memory Has a Page Table Entry with an Architecture Specific Bit that Defines that this Page Table Entry is Only Valid while the CPU is in Kernel Mode.
		 * The remap_pfn_range() Creates Another Page Table Entry, with a Different Virtual Address to the Same Physical Memory Page that Does not Have that Bit Set.
		 * As s Result, by Using the New Virtual Address the Userspace Application is Capable of Accessing the Kernel Memory Allocation.
		 */				
		mmap_return_value = remap_pfn_range(vma, vma->vm_start, PFN_DOWN(virt_to_phys(bus_to_virt(pre_process_mmap_physical_address))) + vma->vm_pgoff, length, vma->vm_page_prot);	
	}	
	

	/*
	 * If mmap_return_value is Less than 0 then Mmap Failed.
	 */	
	if (mmap_return_value < 0) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] MMAP [FAILURE]: %d\n", driver_name, current->pid, mmap_return_value);
		#endif		
		
		return mmap_return_value;
     } 
     else
     {
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> PRE-PROCESS MMAP (PID %d)] MMAP [SUCCESS]\n", driver_name, current->pid);
		#endif	
	 }     
	 
	/*
	 * Lock the pre_process_mmap_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */	 
	down_write(&pre_process_mmap_sem);
	 
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */		 
	search_element = pid_list_head;
	 
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */		 
	 while(search_element != NULL)
	 {
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Made the MMap File Operation Call.
		 */			
		if(search_element->pid == current->pid)
		{
			/*
			 * Save the pre_process_mmap_virtual_address Virtual Pointer inside the Singly Linked List Node of the Current PID so that we Know that the Kernel Allocated Memory Belongs to the Current PID.
			 * From this Moment the Virtual Address of that Kernel Memory can be Accessed only through the Current Singly List Node and Only on Behalf of the PID that this Node Belongs to.
			 */			
			search_element->pre_process_mmap_virtual_address = pre_process_mmap_virtual_address;
			
			/*
			 * Save the pre_process_mmap_physical_address Physical Address inside the Singly Linked List Node of the Current PID so that we Know that the Kernel Allocated Memory Belongs to the Current PID.
			 * From this Moment the Physical Address of that Kernel Memory can be Accessed only through the Current Singly List Node and Only on Behalf of the PID that this Node Belongs to.
			 */				
			search_element->pre_process_mmap_physical_address = (u32)pre_process_mmap_physical_address;
			
			break;
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */			 
		search_element = search_element->next_pid;
	}
	
	/*
	 * Unlock the pre_process_mmap_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */		
	up_write(&pre_process_mmap_sem);
	       
	       
    return SUCCESS;
}

/** OK
  * The pre_process_mmap_ops Structure Indicates which Driver Function Routines Correspond (Called) to the 
  * File Operations that a Userspace Application Makes on the pre_process_mmap_value Debugfs File.
  */
struct file_operations pre_process_mmap_ops = {
	open:			pre_process_mmap_open,
	release:		pre_process_mmap_release,
	mmap:			pre_process_mmap_mmap,
};




/** OK
  * post_process_mmap_open()
  * 
  * It is Called when a Userspace Application Opens the post_process_mmap_value Debugfs File.
  */
int post_process_mmap_open(struct inode *inode, struct file *file_pointer)
{	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> POST-PROCESS OPEN (PID %d)] Opening Post-Processed File\n", driver_name, current->pid);
	#endif

	return SUCCESS;	
}

/** OK
  * post_process_mmap_release()
  * 
  * It is Called when a Userspace Application Releases the post_process_mmap_value Debugfs File.
  */
int post_process_mmap_release(struct inode *inode, struct file *file_pointer)
{
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> POST-PROCESS RELEASE (PID %d)] Releasing Post-Processed File\n", driver_name, current->pid);
	#endif
	
	return(SUCCESS);	
}

/** OK
  * post_process_mmap_mmap()
  * 
  * It is Called when a Userspace Application Makes a Mmap File Operation to the post_process_mmap_value Debugfs File.
  * By Calling the post_process_mmap_mmap() a Userspace Application Intends to Map a Kernel Space Memory Allocation to Userspace.
  */
static int post_process_mmap_mmap(struct file *file, struct vm_area_struct *vma)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */		
	struct pid_reserved_memories *search_element = NULL;
	
	/*
	 * The Virtual Address Pointer of the Kernel Memory that will be Allocated by the dma_alloc_coherent().
	 */		
	u64 *post_process_mmap_virtual_address = NULL;
	
	/*
	 * The Physical Address of the Kernel Memory that will be Allocated by the dma_alloc_coherent().
	 */		
	dma_addr_t post_process_mmap_physical_address;
	
	int mmap_return_value;

	/*
	 * Get the Size of Kernel Memory that the Userspace Application Requested to Map.
	 */
    long length = vma->vm_end - vma->vm_start;
    
    
  	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] Going to MMAP Kernel Memory of Size %ld\n",driver_name, current->pid, (unsigned long)length);
	#endif  
    
	/*
	 * The dma_alloc_coherent() is Used to Allocate Physically Contiguous Consistent Memory which is Suitable for DMA Operations.
	 * Consistent Memory Refers to Write Operations by either the Device or the Processor that can Immediately be Read by the Processor or Device
	 * without Having to Worry about Caching Effects.
	 * 
	 * The dma_alloc_coherent() herein is Called to Allocate 4M of Kernel Contiguous Memory that will be Used by the DMA to Store the Processed Image Data 
	 * and by the Userspace Application to Directly Save the Image Data After Processing.
	 * 
	 * The dma_alloc_coherent() Returns a Pointer (post_process_mmap_virtual_address) with the Virtual Address of the Allocated Memory.
	 * It, also, Returns the post_process_mmap_physical_address Pointer with the Physical Address of the Same Allocated Memory.
	 * 
	 * The Physical Address will be Used by the FPGA Peripherals (DMA, Microblaze etc) to Access the Kernel Memory.
	 * The Virtual Address will be Used by the Kernel Driver and the Userspace Application to Access the Kernel Memory.
	 */
	post_process_mmap_virtual_address = dma_alloc_coherent ( &dev->dev, MMAP_ALLOCATION_SIZE, &post_process_mmap_physical_address, GFP_ATOMIC);

	/*
	 * If the Returned Value of the post_process_mmap_virtual_address Pointer is NULL then the dma_alloc_coherent() Failed to Allocate Memory.
	 */		
	if(post_process_mmap_virtual_address == NULL)
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] Allocating MMAP Coherent Memory [FAILURE]\n",driver_name, current->pid);
		#endif	
		
		return FAILURE;			
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] Allocating MMAP Coherent Memory (Virtual 0x%016lX)(Physical 0x%016lX)[SUCCESS]\n",driver_name, current->pid, (unsigned long)post_process_mmap_virtual_address, (unsigned long)post_process_mmap_physical_address);
		#endif
	}
 
	
	/*
	 * Do Not Allow Larger Mappings than the Number of Allocated Pages in the Kernel
	 */ 	 
	if (length > KERNEL_ALLOCATION_SIZE)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] Cannot MMAP Kernel Memory [Process Requested Larger Number of Pages than the Allocated Ones]\n", driver_name, current->pid);
		#endif
       
		return FAILURE;
	}
	
	
	/*
	 * If Architecture Supports dma_mmap_coherent().
	 */ 
	if (vma->vm_pgoff == 0) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] Going to MMAP with dma_mmap_coherent()\n", driver_name, current->pid);
		#endif
		
		/*
		 * The dma_mmap_coherent() is Used to Map the Kernel Memory to Userspace as Non-Cacheable.
		 * It Requires the Virtual and Physical Addresses as well as the Length of the Memory that we Want to Map to Userspace.
		 */			
		mmap_return_value = dma_mmap_coherent(&dev->dev, vma, post_process_mmap_virtual_address, post_process_mmap_physical_address, length);
	} 
	/*
	 * If Architecture Does not Support dma_mmap_coherent() Use the remap_pfn_range().
	 */ 	
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] Going to MMAP with remap_pfn_range()\n", driver_name, current->pid);
		#endif		
				
		/*
		 * Set the Memory Area as Non-Cacheable.
		 */					
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		
		/*
		 * Set the vm_flags with the VM_IO Flag.
		 * The VM_IO flag specifies that this memory area is a mapping of a device's I/O space.
		 * It, also, Specifies, among Other Things, that the Memory Allocated Area Must not be Included in any Process's Core Dump.
		 */			
		vma->vm_flags |= VM_IO;

		/*
		 * Kernel Memory Has a Page Table Entry with an Architecture Specific Bit that Defines that this Page Table Entry is Only Valid while the CPU is in Kernel Mode.
		 * The remap_pfn_range() Creates Another Page Table Entry, with a Different Virtual Address to the Same Physical Memory Page that Does not Have that Bit Set.
		 * As s Result, by Using the New Virtual Address the Userspace Application is Capable of Accessing the Kernel Memory Allocation.
		 */					
		mmap_return_value = remap_pfn_range(vma, vma->vm_start, PFN_DOWN(virt_to_phys(bus_to_virt(post_process_mmap_physical_address))) + vma->vm_pgoff, length, vma->vm_page_prot);	
	}	
	
	/*
	 * If mmap_return_value is Less than 0 then Mmap Failed.
	 */		
	if (mmap_return_value < 0) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] MMAP [FAILURE]: %d\n", driver_name, current->pid, mmap_return_value);
		#endif		
		
		return mmap_return_value;
     } 
     else
     {
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> POST-PROCESS MMAP (PID %d)] MMAP [SUCCESS]\n", driver_name, current->pid);
		#endif	
	 } 
	 
	/*
	 * Lock the post_process_mmap_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */	 	 
	down_write(&post_process_mmap_sem);
	 
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */			 
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */			 
	 while(search_element != NULL)
	 {
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Made the MMap File Operation Call.
		 */			
		if(search_element->pid == current->pid)
		{
			/*
			 * Save the post_process_mmap_virtual_address Virtual Pointer inside the Singly Linked List Node of the Current PID so that we Know that the Kernel Allocated Memory Belongs to the Current PID.
			 * From this Moment the Virtual Address of that Kernel Memory can be Accessed only through the Current Singly List Node and Only on Behalf of the PID that this Node Belongs to.
			 */					
			search_element->post_process_mmap_virtual_address = post_process_mmap_virtual_address;
			
			/*
			 * Save the post_process_mmap_physical_address Physical Address inside the Singly Linked List Node of the Current PID so that we Know that the Kernel Allocated Memory Belongs to the Current PID.
			 * From this Moment the Physical Address of that Kernel Memory can be Accessed only through the Current Singly List Node and Only on Behalf of the PID that this Node Belongs to.
			 */				
			search_element->post_process_mmap_physical_address = (u32)post_process_mmap_physical_address;
			
			break;
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */			 
		search_element = search_element->next_pid;
	}
	     
	/*
	 * Unlock the post_process_mmap_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */		     
	up_write(&post_process_mmap_sem);       
	       
    return SUCCESS;
}

/** OK
  * The post_process_mmap_ops Structure Indicates which Driver Function Routines Correspond (Called) to the 
  * File Operations that a Userspace Application Makes on the post_process_mmap_value Debugfs File.
  */
struct file_operations post_process_mmap_ops = {
	open:			post_process_mmap_open,
	release:		post_process_mmap_release,
	mmap:			post_process_mmap_mmap,
};




/** OK
  * xilinx_pci_driver_init()
  * 
  * It is Called when the Driver Module is Inserted into the Linux Kernel.
  * 
  * @return SUCCESS when the whole Initialization Procedure Completes Successfully.
  */
static int xilinx_pci_driver_init(void)
{
	int interrupts_number = 0;
	
	/*
	 * Initialize the Wait Queue.
	 */
	init_waitqueue_head(&ioctl_queue);
	
	/*
	 * Initialize the Read/Write Semaphores Present in the Driver.
	 */
	init_rwsem(&ioctl_sem);
	
	init_rwsem(&case_0_sem);
	init_rwsem(&case_1_sem);
	init_rwsem(&case_2_sem);
	init_rwsem(&case_3_sem);
	init_rwsem(&case_4_sem);
	init_rwsem(&case_5_sem);
	init_rwsem(&case_6_sem);
	
	init_rwsem(&msi_1_sem);
	init_rwsem(&msi_2_sem);
	init_rwsem(&msi_3_sem);
	init_rwsem(&msi_4_sem);
	init_rwsem(&msi_5_sem);
	init_rwsem(&msi_6_sem);
	init_rwsem(&msi_7_sem);
		
	init_rwsem(&set_pages_sem);
	init_rwsem(&unmap_pages_sem);
	init_rwsem(&sg_sem);
	
	init_rwsem(&write_sem);
	init_rwsem(&search_element_sem);
	
	init_rwsem(&main_open_sem);
	init_rwsem(&main_release_sem);
	
	init_rwsem(&shared_repo_mmap_sem);
	init_rwsem(&pre_process_mmap_sem);
	init_rwsem(&post_process_mmap_sem);

	
	/*
	 * Check if Hardware Exists According to the Vendor and Device ID of the PCIe Device.
	 * 
	 */
	dev = pci_get_device (VENDOR_ID, DEVICE_ID, dev);
	if (dev == NULL) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking for Hardware [NOT FOUND]\n", driver_name, current->pid);
		#endif			
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking for Hardware [FOUND]\n", driver_name, current->pid);
		#endif			
	}

	/*
	 * Create Debugfs File Which Will Be Used to Provide Additional System Calls from the Userspace Application to the Driver
	 */			 
	pre_process_mmap_file = debugfs_create_file("pre_process_mmap_value", 0644, NULL, NULL, &pre_process_mmap_ops);
	
	
	if(pre_process_mmap_file == NULL)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Creating Pre-Process MMAP Debugfs File [FAILURE]\n", driver_name, current->pid);
		#endif	
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Creating Pre-Process MMAP Debugfs File [SUCCESS]\n", driver_name, current->pid);
		#endif			
	}

	
	/*
	 * Create Debugfs File Which Will Be Used to Provide Additional System Calls from the Userspace Application to the Driver
	 */			 
	post_process_mmap_file = debugfs_create_file("post_process_mmap_value", 0644, NULL, NULL, &post_process_mmap_ops);
	
	
	if(post_process_mmap_file == NULL)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Creating Post-Process MMAP Debugfs File [FAILURE]\n", driver_name, current->pid);
		#endif	
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Creating Post-Process MMAP Debugfs File [SUCCESS]\n", driver_name, current->pid);
		#endif			
	}
	
	/*
	 * Create Debugfs File Which Will Be Used to Provide Additional System Calls from the Userspace Application to the Driver
	 */			 
	shared_repo_mmap_file = debugfs_create_file("shared_repo_mmap_value", 0644, NULL, NULL, &shared_repo_ops);
	
	
	if(shared_repo_mmap_file == NULL)
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Creating Shared Repo MMAP Debugfs File [FAILURE]\n", driver_name, current->pid);
		#endif	
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Creating Shared Repo MMAP Debugfs File [SUCCESS]\n", driver_name, current->pid);
		#endif			
	}
	
	/*
	 * Configure the status_flags with the HAVE_DEBUGFS Flag to Let the Driver Know that we Have Created debugfs Files.
	 */
	status_flags = status_flags | HAVE_DEBUGFS;	
	
	/*
	 * Enable the PCIe Endpoint Device
	 */
	if (pci_enable_device(dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Enabling Device [FAILURE]\n", driver_name, current->pid);
		#endif			
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Enabling Device [SUCCESS]\n", driver_name, current->pid);
		#endif
	}

	/*
	 * Provide Master Capabilities to the device
	 */	
	pci_set_master(dev);

	/*
	 * Get BAR 0 Physical Address from PCI Structure
	 * BAR 0 Represents the AXI Address Space of the Peripheral Devices inside the FPGA.
	 */
	bar0_address_physical = pci_resource_start(dev, BAR0_64BIT);
	if (bar0_address_physical<0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Assign a Physical Address to BAR 0 [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
    } 
    else
    {
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Assign a Physical Address to BAR 0 [0x%lX]\n", driver_name, current->pid, (unsigned long)bar0_address_physical);
		#endif	
    }

	/*
	 * Get the Size of Address Space or Memory that BAR 0 Represents
	 */
	bar0_length = pci_resource_len (dev, BAR0_64BIT);
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] BAR 0 Length is: %d Bytes\n", driver_name, current->pid, (unsigned int)bar0_length);
	#endif		
	

	/*
	 * Get BAR 1 Physical Address from PCI Structure
	 * BAR 1 Represents the AXI BRAM inside the FPGA.
	 */
	bar1_address_physical = pci_resource_start(dev, BAR1_64BIT);
	if (bar1_address_physical<0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Assign a Physical Address to BAR 1 [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
    } 
    else
    {
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Assign a Physical Address to BAR 1 [0x%lX]\n", driver_name, current->pid, (unsigned long)bar1_address_physical);
		#endif	
    }

	/*
	 * Get the Size of Address Space or Memory that BAR 1 Represents which is Equal to the Size of the BRAM Memory
	 */
	bar1_length = pci_resource_len (dev, BAR1_64BIT);
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] BAR 1 Length is: %d Bytes\n", driver_name, current->pid, (unsigned int)bar1_length);
	#endif		

	/*
	 * Get BAR 2 Physical Address from PCI Structure
	 * BAR 2 Represents the AXI DDR3 Memory inside the FPGA.
	 */
	bar2_address_physical = pci_resource_start(dev, BAR2_64BIT);
	if (bar2_address_physical<0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Assign a Physical Address to BAR 2 [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
    } 
    else
    {
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Assign a Physical Address to BAR 2 [0x%lX]\n", driver_name, current->pid, (unsigned long)bar2_address_physical);
		#endif	
    }

	/*
	 * Get the Size of Address Space or Memory that BAR 2 Represents which is Equal to the Size of the DDR3 Memory
	 */
	bar2_length = pci_resource_len (dev, BAR2_64BIT);
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] BAR 2 Length is: %d Bytes\n", driver_name, current->pid, (unsigned int)bar2_length);
	#endif		
	
	/* 
	 * Remap the I/O Register Block for BAR 0 so that It Can Be Safely Accessed from a Virtual Addresss(bar0_address_virtual).
	 * I/O Register Block Starts at bar0_address_physical and is 4M Bytes Long
	 */
	bar0_address_virtual = ioremap_nocache(bar0_address_physical, bar0_length);
	if (!bar0_address_virtual) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Remap BAR 0 Memory to Virtual Address Space with Virtual Address [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
	} 
    else
    {
		#ifdef DEBUG_MESSAGES
    	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Remap BAR 0 Memory to Virtual Address Space with Virtual Address [Ox%lX]\n", driver_name, current->pid, (unsigned long)bar0_address_virtual);
		#endif			
    }

	/* 
	 * Remap the I/O Register Block for BAR 1 so that It Can Be Safely Accessed from a Virtual Addresss(bar1_address_virtual).
	 * I/O Register Block Starts at bar1_address_physical and is 256K Bytes Long
	 */
	bar1_address_virtual = ioremap(bar1_address_physical, bar1_length);
	if (!bar1_address_virtual) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Remap BAR 1 Memory to Virtual Address Space with Virtual Address [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
	} 
    else
    {
		#ifdef DEBUG_MESSAGES
    	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Remap BAR 1 Memory to Virtual Address Space with Virtual Address [Ox%lX]\n", driver_name, current->pid, (unsigned long)bar1_address_virtual);
		#endif			
    }

	/* 
	 * Remap the I/O Register Block for BAR 2 so that It Can Be Safely Accessed from a Virtual Addresss(bar2_address_virtual).
	 * I/O Register Block Starts at bar2_address_physical and is 512M Bytes Long
	 */
	bar2_address_virtual = ioremap(bar2_address_physical, bar2_length);
	if (!bar2_address_virtual) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Remap BAR 2 Memory to Virtual Address Space with Virtual Address [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
	} 
    else
    {
		#ifdef DEBUG_MESSAGES
    	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Remap BAR 2 Memory to Virtual Address Space with Virtual Address [Ox%lX]\n", driver_name, current->pid, (unsigned long)bar2_address_virtual);
		#endif			
    }

	/*
	 * Get IRQ from pci_dev structure. It May have been Remapped by the Kernel and this Value will be the Correct One.
	 */
	irq = dev->irq;
	
	#ifdef DEBUG_MESSAGES
	printk("[%s-DBG -> DRIVER INIT (PID %d)] Getting the Device IRQ [IRQ %d]\n", driver_name, current->pid, irq);
	#endif	
		
	/* 
	 * Check Memory Region for BAR 0 Before Requesting Control
	 */
	if (check_mem_region(bar0_address_physical, bar0_length)<0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking BAR 0 Memory Region [IN USE]\n", driver_name, current->pid);
		#endif		
       	
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking BAR 0 Memory Region [OK FOR MEMORY REQUEST]\n", driver_name, current->pid);
		#endif	
	}

	/* 
	 * Check Memory Region for BAR 1 Before Requesting Control
	 */
	if (check_mem_region(bar1_address_physical, bar1_length)<0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking BAR 1 Memory Region [IN USE]\n", driver_name, current->pid);
		#endif		
       	
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking BAR 1 Memory Region [OK FOR MEMORY REQUEST]\n", driver_name, current->pid);
		#endif	
	}

	/* 
	 * Check Memory Region for BAR 2 Before Requesting Control
	 */
	if (check_mem_region(bar2_address_physical, bar2_length)<0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking BAR 2 Memory Region [IN USE]\n", driver_name, current->pid);
		#endif		
       	
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Checking BAR 2 Memory Region [OK FOR MEMORY REQUEST]\n", driver_name, current->pid);
		#endif	
	}

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Requesting Memory Regions for BAR 0, BAR 1 and BAR 2\n",driver_name, current->pid);
	#endif

	/* 
	 * Request BAR 0, BAR 1 and BAR 2 Memory Regions 
	 */		
	request_mem_region(bar0_address_physical, bar0_length, driver_name);
	request_mem_region(bar1_address_physical, bar1_length, driver_name);
	request_mem_region(bar2_address_physical, bar2_length, driver_name);
	
	/*
	 * Configure the status_flags with the HAVE_REGION Flag to Let the Driver Know that we have Claimed the BAR 0, BAR1 and BAR 2 Memory Regions.
	 */	
	status_flags = status_flags | HAVE_REGION;

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Initializing Hardware Done\n",driver_name, current->pid);
	#endif	

	/*
	 * Enable the MSI Interrupts with Range 1 to 7 which is 7 Interrupts.
	 * The MSI Can be Configured with as many as 32 Interrupts.
	 * 
	 * The pci_enable_msi_range() Returns the Number of MSI Interrupts that were Allocated which is not Necessarily Equal to the Requested Range.
	 */
	interrupts_number = pci_enable_msi_range(dev, 1, 7);	
	
	if (interrupts_number < 0) 
	{	
		#ifdef DEBUG_MESSAGES
		printk("[%s-DBG -> DRIVER INIT (PID %d)] Enabling MSI Interrupts with Re-Assigned IRQ [FAILURE-ERROR %d]\n", driver_name, current->pid, interrupts_number);	
		#endif		
        return (-1);
	}
	else
	{
		irq=dev->irq;
		#ifdef DEBUG_MESSAGES
		printk("[%s-DBG -> DRIVER INIT (PID %d)] Enabling MSI Interrupts with Re-Assigned IRQ [IRQ %d]\n", driver_name, current->pid, irq);	
		#endif	
		
		
		#ifdef DEBUG_MESSAGES
		printk("[%s-DBG -> DRIVER INIT (PID %d)] The Number of Assigned Interrupts Is: %d\n", driver_name, current->pid, interrupts_number);	
		#endif		
	}	
	

	/**
	  * Request Threaded IRQ 1 Allocation from OS which is the Base Interrupt from dev->irq.
	  * 
	  * Interrupts are Conditions that should be Served by an Interrupt Handler as Fast as Possible.
	  * Threaded IRQs are Used in cases where an Interrupt Requires Large Code Executions in Order to be Handled.
	  * 
	  * In such cases the MSI Interrupt is Handled by a Fast Interrupt Handler (irq_fast_handler_0).
	  * The irq_fast_handler_0() returns the IRQ_WAKE_THREAD Flag which Starts a Threaded Function (irq_handler_0)
	  * The Threaded Function can Serve the Interrupt Requirements Independently while the MSI Interrupt can be Triggered again.
	  * 
	  * @note The Same Apply for the Rest Allocated IRQs.
	  * 
	  */	
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq, (void *) irq_fast_handler_0, (void *) irq_handler_0, IRQF_SHARED |IRQF_TRIGGER_RISING/*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 0 [FAILURE]\n",driver_name, current->pid, irq);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 0 [SUCCESS]\n",driver_name, current->pid, irq);
		#endif	
	}
	#endif
	

	/*
	 * Request Threaded IRQ 2 Allocation from OS which is the Base Interrupt from dev->irq + 1.
	 */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq + 1, (void *) irq_fast_handler_1, (void *) irq_handler_1, IRQF_SHARED |IRQF_TRIGGER_RISING /*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 1 [FAILURE]\n",driver_name, current->pid, irq + 1);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 1 [SUCCESS]\n",driver_name, current->pid, irq + 1);
		#endif	
	}
	#endif	
	

	/*
	 * Request Threaded IRQ 3 Allocation from OS which is the Base Interrupt from dev->irq + 2.
	 */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq + 2, (void *) irq_fast_handler_2, (void *) irq_handler_2, IRQF_SHARED |IRQF_TRIGGER_RISING /*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 2 [FAILURE]\n",driver_name, current->pid, irq + 2);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 2 [SUCCESS]\n",driver_name, current->pid, irq + 2);
		#endif	
	}
	#endif	
	
	
	/*
	 * Request Threaded IRQ 4 Allocation from OS which is the Base Interrupt from dev->irq + 3.
	 */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq + 3, (void *) irq_fast_handler_3, (void *) irq_handler_3, IRQF_SHARED |IRQF_TRIGGER_RISING /*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 3 [FAILURE]\n",driver_name, current->pid, irq + 3);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 3 [SUCCESS]\n",driver_name, current->pid, irq + 3);
		#endif	
	}
	#endif	
	
	/*
	 * Request Threaded IRQ 5 Allocation from OS which is the Base Interrupt from dev->irq + 4.
	 */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq + 4, (void *) irq_fast_handler_4, (void *) irq_handler_4, IRQF_SHARED |IRQF_TRIGGER_RISING /*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 4 [FAILURE]\n",driver_name, current->pid, irq + 4);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 4 [SUCCESS]\n",driver_name, current->pid, irq + 4);
		#endif	
	}
	#endif	
	
	/*
	 * Request Threaded IRQ 6 Allocation from OS which is the Base Interrupt from dev->irq + 5.
	 */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq + 5, (void *) irq_fast_handler_5, (void *) irq_handler_5, IRQF_SHARED |IRQF_TRIGGER_RISING /*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 5 [FAILURE]\n",driver_name, current->pid, irq + 5);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 5 [SUCCESS]\n",driver_name, current->pid, irq + 5);
		#endif	
	}
	#endif		
	
	/*
	 * Request Threaded IRQ 7 Allocation from OS which is the Base Interrupt from dev->irq + 6.
	 */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (request_threaded_irq(irq + 6, (void *) irq_fast_handler_6, (void *) irq_handler_6, IRQF_SHARED |IRQF_TRIGGER_RISING /*| IRQF_SAMPLE_RANDOM*/, driver_name, dev) < 0) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 6 [FAILURE]\n",driver_name, current->pid, irq + 6);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Allocate IRQ %d to Interrupt Handler 6 [SUCCESS]\n",driver_name, current->pid, irq + 6);
		#endif	
	}
	#endif	
	

	/*
	 * Configure the status_flags with the HAVE_IRQ Flag to Let the Driver Know that we Allocated the MSI Interrupts.
	 */
	status_flags = status_flags | HAVE_IRQ;

	/*
	 * Call this Function to Make Additional Initializations (If Required).
	 * 
	 * Currently initcode() is Empty.
	 */
	initcode();      

	
	/*
	 * Register Driver in the Kernel as a Character Device
	 */
	if (0 > register_chrdev(driver_major_number, driver_name, &xilinx_kc705_driver_file_operations)) 
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Registering the Driver Module [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
	}
	else
	{
		#ifdef DEBUG_MESSAGES
        printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Registering the Driver Module [SUCCESS]\n", driver_name, current->pid);
		#endif			
	}

	/*
	 * Configure the status_flags with the HAVE_KREG Flag to Let the Driver Know that we Have Registered the Driver in Kernel.
	 */
	status_flags = status_flags | HAVE_KREG;

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER INIT (PID %d)] Driver is Successfully Loaded\n", driver_name, current->pid);	
	#endif
	
	return SUCCESS;
}

/** OK
  * xilinx_pci_driver_exit()
  * 
  * It is Called when the Driver Module is Removed from the Linux Kernel.
  */
static void xilinx_pci_driver_exit(void)
{
	
	if(inter_process_shared_info_memory != NULL)
	{	
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] Releasing Inter Process Shared Repository\n",driver_name, current->pid);
		#endif	
				
		inter_process_shared_info_memory = NULL;		
		
	}
	
	/*
	 * If the status_flags Includes the HAVE_DEBUGFS Flag then Remove the Debugfs Files
	 */	
	if (status_flags & HAVE_DEBUGFS)
	{
		debugfs_remove(pre_process_mmap_file);
		debugfs_remove(post_process_mmap_file);
		debugfs_remove(shared_repo_mmap_file);
	
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] Debugfs Files are Removed\n", driver_name, current->pid);
		#endif	
	}
	
	/*
	 * If the status_flags Includes the HAVE_REGION Flag then Release the BAR 0, BAR 1 and BAR 2 Memory Regions
	 */
	if (status_flags & HAVE_REGION) 
	{
		(void)release_mem_region(bar0_address_physical, bar0_length);
		(void)release_mem_region(bar1_address_physical, bar1_length);
		(void)release_mem_region(bar2_address_physical, bar2_length);
		
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] Memory Regions are Released\n", driver_name, current->pid);
		#endif		
	}
	
	/*
	 * If the status_flags Includes the HAVE_IRQ Flag then Release the IRQs
	 */
	if (status_flags & HAVE_IRQ) 
	{
        (void)free_irq(irq, dev);
        (void)free_irq(irq + 1, dev);
        (void)free_irq(irq + 2, dev);
        (void)free_irq(irq + 3, dev);
        (void)free_irq(irq + 4, dev);
        (void)free_irq(irq + 5, dev);
        (void)free_irq(irq + 6, dev);
        (void)free_irq(irq + 7, dev);

        
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] IRQs are Released\n", driver_name, current->pid);
		#endif        
	}

	/*
	 * Disable the MSI Interrupts.
	 */	
	pci_disable_msi(dev);
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] MSI is Disabled\n", driver_name, current->pid);
	#endif		
	
	
	/*
	 * Unmap the BAR 0, BAR 1 and BAR 2 Virtual Addresses.
	 */	
	if (bar0_address_virtual != NULL) 
	{
        iounmap(bar0_address_virtual);        	
 	}

	if (bar1_address_virtual != NULL) 
	{
		iounmap(bar1_address_virtual);
 	}
    
	if (bar2_address_virtual != NULL) 
	{
		iounmap(bar2_address_virtual);
 	}

	/*
	 * Clear the BAR 0, BAR 1 and BAR 2 Pointers.
	 */	
	bar0_address_virtual = NULL;
	bar1_address_virtual = NULL;	
	bar2_address_virtual = NULL;
			
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] Virtual Addresses are Unmapped\n", driver_name, current->pid);
	#endif	
	
	/*
	 * Unregister the Device Driver 
	 */
	if (status_flags & HAVE_KREG) 
	{
		unregister_chrdev(driver_major_number, driver_name);
	}

	/*
	 * Clear the status_flags.
	 */
	status_flags = 0;
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] Driver is Unloaded\n", driver_name, current->pid);
	#endif	

	/*
	 * Disable the PCIe Device
	 */
	pci_disable_device(dev);
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> DRIVER EXIT (PID %d)] The Device is Disabled\n", driver_name, current->pid);
	#endif	
	
}


module_init(xilinx_pci_driver_init);
module_exit(xilinx_pci_driver_exit);

module_param(signal_to_pid, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(signal_to_pid, "Signal to Send");


/** OK
  * irq_fast_handler_0()
  * 
  * Fast Interrupt Handler which is Called when MSI 0 Interrupt is Triggered.
  * There is no Need to Clear any Interrupt.
  * The only Requirement is to Return the IRQ_WAKE_THREAD Flag that Starts the Threaded Function irq_handler_0()
  * which can Handle the Current Interrupt Event Independently without the Risk to Loose the Next Triggered MSI 0 Interrupt.
  * 
  * @note The same Apply for the Rest Fast Interrrupt Handlers.
  */
irqreturn_t irq_fast_handler_0(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}

/* OK
 * irq_fast_handler_1()
 * 
 * Fast Interrupt Handler which is Called when MSI 1 Interrupt is Triggered.
 */
irqreturn_t irq_fast_handler_1(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}

/* OK
 * irq_fast_handler_2()
 * 
 * Fast Interrupt Handler which is Called when MSI 2 Interrupt is Triggered.
 */
irqreturn_t irq_fast_handler_2(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}

/* OK
 * irq_fast_handler_3()
 * 
 * Fast Interrupt Handler which is Called when MSI 3 Interrupt is Triggered.
 */
irqreturn_t irq_fast_handler_3(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}

/* OK
 * irq_fast_handler_4()
 * 
 * Fast Interrupt Handler which is Called when MSI 4 Interrupt is Triggered.
 */
irqreturn_t irq_fast_handler_4(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}

/* OK
 * irq_fast_handler_5()
 * 
 * Fast Interrupt Handler which is Called when MSI 5 Interrupt is Triggered.
 */
irqreturn_t irq_fast_handler_5(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}

/* OK
 * irq_fast_handler_6()
 * 
 * Fast Interrupt Handler which is Called when MSI 6 Interrupt is Triggered.
 */
irqreturn_t irq_fast_handler_6(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_WAKE_THREAD;
}




/** OK
  * irq_handler_0()
  * 
  * Started as a Threaded Function by the irq_fast_handler_0() when a MSI 0 Interrupt Occurs.
  * 
  * MSI 0 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Direct 0 (AGD0).
  * In such Condition the irq_handler_0() Should Gather the Metrics Information that AGD0 Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGD0.
  * 
  */
irqreturn_t irq_handler_0(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */
	struct pid_reserved_memories *search_element = NULL;
	
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 0 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group Direct 0 Completed\n", driver_name, current->pid);
	#endif	
	
	/*
	 * Lock the msi_1_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&msi_1_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */
	while(search_element != NULL)
	{
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Direct 0 (AGD0).
		 * If this is the Case then we can Copy the AGDO Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_direct_0_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif			

			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_read_transactions = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_read_bytes = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_read_bytes;
			
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_write_transactions = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_write_transactions;
			
			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_write_bytes = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_write_bytes;
			
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_packets = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_packets;
			
			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_bytes = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_gcc_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd0.apm_gcc_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_fetch_time_end_u;
			
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_send_time_start_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_send_time_start_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_send_time_start_u;	
			
			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_send_time_end_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd0.cdma_send_time_end_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.cdma_send_time_end_u;	
			
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd0.dma_accel_time_start_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd0.dma_accel_time_start_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.dma_accel_time_start_u;	
			
			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd0.dma_accel_time_end_l = inter_process_shared_info_memory->accel_direct_0_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd0.dma_accel_time_end_u = inter_process_shared_info_memory->accel_direct_0_shared_metrics.dma_accel_time_end_u;	
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGD0 
			 * to Indicate the Completion of the Acceleration Procedure by the AGD0.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */
			//setup_and_send_signal(DEFAULT_SIGNAL_0, inter_process_shared_info_memory->shared_status_flags.accel_direct_0_occupied_pid);	
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_DIRECT_0_OCCUPIED Value which, also, Indicates the Completion of AGD0.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGD0 has Completed.
			 */
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_DIRECT_0_OCCUPIED;
			
			/*
			 * Clear the agd0_busy Field of the BRAM to Indicate that the AGD0 is Available.
			 */
			inter_process_shared_info_memory->shared_status_flags.agd0_busy	= 0;	
			
			/*
			 * Clear the accel_direct_0_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGD0.
			 */			
			inter_process_shared_info_memory->shared_status_flags.accel_direct_0_occupied_pid = 0;
			
			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGD0.
			 */
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);
					
		}
		
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_1_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */
	up_write(&msi_1_sem);

	/*
	 * The Fact that an Interrupt Occured it Means that the AGD0 Has Completed which in other Words Means that the AGD0 is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGD0.
	 */
	wake_up_interruptible(&ioctl_queue);

	return IRQ_HANDLED;
}

/** OK
  * irq_handler_1()
  * 
  * Started as a Threaded Function by the irq_fast_handler_1() when a MSI 1 Interrupt Occurs.
  * 
  * MSI 1 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Direct 1 (AGD0).
  * In such Condition the irq_handler_1() Should Gather the Metrics Information that AGD1 Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGD1.
  * 
  */
irqreturn_t irq_handler_1(int irq, void *dev_id, struct pt_regs *regs)
{	
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;
	
	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 1 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group Direct 1 Completed\n", driver_name, current->pid);
	#endif
		
	/*
	 * Lock the msi_2_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */	
	down_write(&msi_2_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */	 
	while(search_element != NULL)
	{

		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Direct 1 (AGD1).
		 * If this is the Case then we can Copy the AGD1 Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */		
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_direct_1_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif			
			
			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_read_transactions = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_read_bytes = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_read_bytes;
			
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_write_transactions = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_write_transactions;
		
			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_write_bytes = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_write_bytes;
			
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_packets = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_packets;
			
			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_bytes = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_gcc_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.apm_gcc_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_fetch_time_end_u;
				
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_send_time_start_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_send_time_start_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_send_time_start_u;	
			
			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_send_time_end_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agd1.cdma_send_time_end_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.cdma_send_time_end_u;	
			
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.dma_accel_time_start_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.dma_accel_time_start_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.dma_accel_time_start_u;	

			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.dma_accel_time_end_l = inter_process_shared_info_memory->accel_direct_1_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agd1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agd1.dma_accel_time_end_u = inter_process_shared_info_memory->accel_direct_1_shared_metrics.dma_accel_time_end_u;
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGD1 
			 * to Indicate the Completion of the Acceleration Procedure by the AGD1.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */			
			//setup_and_send_signal(DEFAULT_SIGNAL_1, inter_process_shared_info_memory->shared_status_flags.accel_direct_1_occupied_pid);		
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_DIRECT_1_OCCUPIED Value which, also, Indicates the Completion of AGD1.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGD1 has Completed.
			 */			
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_DIRECT_1_OCCUPIED;		
			
			/*
			 * Clear the agd1_busy Field of the BRAM to Indicate that the AGD1 is Available.
			 */			
			inter_process_shared_info_memory->shared_status_flags.agd1_busy	= 0;	
			
			/*
			 * Clear the accel_direct_1_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGD1.
			 */				
			inter_process_shared_info_memory->shared_status_flags.accel_direct_1_occupied_pid = 0;	
			
			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGD1.
			 */			
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);				

		}
		
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_2_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */
	up_write(&msi_2_sem);	

	/*
	 * The Fact that an Interrupt Occured it Means that the AGD1 Has Completed which in other Words Means that the AGD1 is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGD1.
	 */
	wake_up_interruptible(&ioctl_queue);	

	return IRQ_HANDLED;
}

/** OK
  * irq_handler_2()
  * 
  * Started as a Threaded Function by the irq_fast_handler_2() when a MSI 2 Interrupt Occurs.
  * 
  * MSI 2 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Indirect 0 (AGI0).
  * In such Condition the irq_handler_2() Should Gather the Metrics Information that AGI0 Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGI0.
  * 
  */
irqreturn_t irq_handler_2(int irq, void *dev_id, struct pt_regs *regs)
{	

	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */
	struct pid_reserved_memories *search_element = NULL;
	
		
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 2 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group Indirect 0 Completed\n", driver_name, current->pid);
	#endif
	
	/*
	 * Lock the msi_3_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&msi_3_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */	 
	while(search_element != NULL)
	{
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Indirect 0 (AGI0).
		 * If this is the Case then we can Copy the AGIO Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */		
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_indirect_0_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif
					
			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_read_transactions = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_read_bytes = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_read_bytes;
		
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_write_transactions = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_write_transactions;
			
			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_write_bytes = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_write_bytes;
			
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_packets = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_packets;
			
			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_bytes = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_gcc_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.apm_gcc_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_fetch_time_end_u;
			
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_send_time_start_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_send_time_start_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_send_time_start_u;	
			
			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_send_time_end_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi0.cdma_send_time_end_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.cdma_send_time_end_u;	
					
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agi0.dma_accel_time_start_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.dma_accel_time_start_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.dma_accel_time_start_u;	
			
			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.dma_accel_time_end_l = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi0 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi0.dma_accel_time_end_u = inter_process_shared_info_memory->accel_indirect_0_shared_metrics.dma_accel_time_end_u;	
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGI0 
			 * to Indicate the Completion of the Acceleration Procedure by the AGI0.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */			
			//setup_and_send_signal(DEFAULT_SIGNAL_2, inter_process_shared_info_memory->shared_status_flags.accel_indirect_0_occupied_pid);	
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_INDIRECT_0_OCCUPIED Value which, also, Indicates the Completion of AGI0.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGI0 has Completed.
			 */				
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_INDIRECT_0_OCCUPIED;
			
			/*
			 * Clear the agi0_busy Field of the BRAM to Indicate that the AGI0 is Available.
			 */				
			inter_process_shared_info_memory->shared_status_flags.agi0_busy	= 0;	
			
			/*
			 * Clear the accel_indirect_0_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGI0.
			 */				
			inter_process_shared_info_memory->shared_status_flags.accel_indirect_0_occupied_pid = 0;	
			
			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGI0.
			 */			
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);							
			
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_3_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */
	up_write(&msi_3_sem);	

	/*
	 * The Fact that an Interrupt Occured it Means that the AGI0 Has Completed which in other Words Means that the AGI0 is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGI0.
	 */
	wake_up_interruptible(&ioctl_queue);	

	return IRQ_HANDLED;
}

/** OK
  * irq_handler_3()
  * 
  * Started as a Threaded Function by the irq_fast_handler_3() when a MSI 3 Interrupt Occurs.
  * 
  * MSI 3 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Indirect 1 (AGI1).
  * In such Condition the irq_handler_3() Should Gather the Metrics Information that AGI1 Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGI1.
  * 
  */
irqreturn_t irq_handler_3(int irq, void *dev_id, struct pt_regs *regs)
{	
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;

	
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 3 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group Indirect 1 Completed\n", driver_name, current->pid);
	#endif
	
	/*
	 * Lock the msi_4_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&msi_4_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	
	search_element = pid_list_head;
	 
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */
	while(search_element != NULL)
	{
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Indirect 1 (AGI1).
		 * If this is the Case then we can Copy the AGI1 Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */			
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_indirect_1_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif			

			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_read_transactions = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_read_bytes = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_read_bytes;
			
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_write_transactions = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_write_transactions;

			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_write_bytes = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_write_bytes;
			
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_packets = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_packets;

			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_bytes = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_gcc_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi1.apm_gcc_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_fetch_time_end_u;
			
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_send_time_start_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_send_time_start_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_send_time_start_u;	
			
			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_send_time_end_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi1.cdma_send_time_end_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.cdma_send_time_end_u;	
			
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi1.dma_accel_time_start_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agi1.dma_accel_time_start_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.dma_accel_time_start_u;	
			
			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agi1.dma_accel_time_end_l = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi1 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agi1.dma_accel_time_end_u = inter_process_shared_info_memory->accel_indirect_1_shared_metrics.dma_accel_time_end_u;		
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGI1 
			 * to Indicate the Completion of the Acceleration Procedure by the AGI1.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */					
			//setup_and_send_signal(DEFAULT_SIGNAL_3, inter_process_shared_info_memory->shared_status_flags.accel_indirect_1_occupied_pid);	
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_INDIRECT_1_OCCUPIED Value which, also, Indicates the Completion of AGI1.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGI1 has Completed.
			 */				
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_INDIRECT_1_OCCUPIED;	
		
			/*
			 * Clear the agi1_busy Field of the BRAM to Indicate that the AGI1 is Available.
			 */			
			inter_process_shared_info_memory->shared_status_flags.agi1_busy	= 0;	
			
			/*
			 * Clear the accel_indirect_1_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGI1.
			 */				
			inter_process_shared_info_memory->shared_status_flags.accel_indirect_1_occupied_pid = 0;	
			
			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGI1.
			 */			
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);								
			
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_4_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */
	up_write(&msi_4_sem);
	
	/*
	 * The Fact that an Interrupt Occured it Means that the AGI1 Has Completed which in other Words Means that the AGI1 is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGI1.
	 */	
	wake_up_interruptible(&ioctl_queue);
		
	return IRQ_HANDLED;
}

/** OK
  * irq_handler_4()
  * 
  * Started as a Threaded Function by the irq_fast_handler_4() when a MSI 4 Interrupt Occurs.
  * 
  * MSI 4 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Indirect 2 (AGI2).
  * In such Condition the irq_handler_4() Should Gather the Metrics Information that AGI2 Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGI2.
  * 
  */
irqreturn_t irq_handler_4(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;


	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 4 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group Indirect 2 Completed\n", driver_name, current->pid);
	#endif

	/*
	 * Lock the msi_5_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&msi_5_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	
	search_element = pid_list_head;


	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */	 
	while(search_element != NULL)
	{
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Indirect 2 (AGI2).
		 * If this is the Case then we can Copy the AGI2 Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */			
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_indirect_2_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif			

			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_read_transactions = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_read_bytes = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_read_bytes;
			
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_write_transactions = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_write_transactions;
			
			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_write_bytes = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_write_bytes;
		
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_packets = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_packets;
			
			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_bytes = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_gcc_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi2.apm_gcc_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_fetch_time_end_u;
			
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_send_time_start_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_send_time_start_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_send_time_start_u;	

			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_send_time_end_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi2.cdma_send_time_end_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.cdma_send_time_end_u;	
			
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */					
			search_element->shared_repo_virtual_address->process_metrics.agi2.dma_accel_time_start_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.dma_accel_time_start_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.dma_accel_time_start_u;	
			
			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.dma_accel_time_end_l = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi2 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi2.dma_accel_time_end_u = inter_process_shared_info_memory->accel_indirect_2_shared_metrics.dma_accel_time_end_u;	
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGI2 
			 * to Indicate the Completion of the Acceleration Procedure by the AGI2.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */					
			//setup_and_send_signal(DEFAULT_SIGNAL_4, inter_process_shared_info_memory->shared_status_flags.accel_indirect_2_occupied_pid);
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_INDIRECT_2_OCCUPIED Value which, also, Indicates the Completion of AGI2.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGI2 has Completed.
			 */				
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_INDIRECT_2_OCCUPIED;		
					
			/*
			 * Clear the agi2_busy Field of the BRAM to Indicate that the AGI2 is Available.
			 */						
			inter_process_shared_info_memory->shared_status_flags.agi2_busy	= 0;	
			
			/*
			 * Clear the accel_indirect_2_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGI2.
			 */				
			inter_process_shared_info_memory->shared_status_flags.accel_indirect_2_occupied_pid = 0;
			
			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGI2.
			 */			
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);					
			
		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_5_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */
	up_write(&msi_5_sem);
	
	/*
	 * The Fact that an Interrupt Occured it Means that the AGI2 Has Completed which in other Words Means that the AGI2 is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGI2.
	 */	
	wake_up_interruptible(&ioctl_queue);

	return IRQ_HANDLED;
}

/** OK
  * irq_handler_5()
  * 
  * Started as a Threaded Function by the irq_fast_handler_5() when a MSI 5 Interrupt Occurs.
  * 
  * MSI 5 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Indirect 3 (AGI3).
  * In such Condition the irq_handler_5() Should Gather the Metrics Information that AGI3 Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGI3.
  * 
  */
irqreturn_t irq_handler_5(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;


	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 5 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group Indirect 3 Completed\n", driver_name, current->pid);
	#endif	

	/*
	 * Lock the msi_6_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&msi_6_sem);
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	
	search_element = pid_list_head;


	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */	 
	while(search_element != NULL)
	{
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Indirect 3 (AGI3).
		 * If this is the Case then we can Copy the AGI3 Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */			
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_indirect_3_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif			

			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_read_transactions = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_read_bytes = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_read_bytes;
			
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_write_transactions = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_write_transactions;
			
			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_write_bytes = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_write_bytes;
			
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_packets = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_packets;
			
			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_bytes = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_gcc_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agi3.apm_gcc_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_fetch_time_end_u;
			
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */			
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_send_time_start_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_send_time_start_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_send_time_start_u;	
			
			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_send_time_end_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.cdma_send_time_end_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.cdma_send_time_end_u;	
			
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.dma_accel_time_start_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.dma_accel_time_start_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.dma_accel_time_start_u;	
			
			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.dma_accel_time_end_l = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agi3 Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agi3.dma_accel_time_end_u = inter_process_shared_info_memory->accel_indirect_3_shared_metrics.dma_accel_time_end_u;	
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGI3 
			 * to Indicate the Completion of the Acceleration Procedure by the AGI3.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */					
			//setup_and_send_signal(DEFAULT_SIGNAL_5, inter_process_shared_info_memory->shared_status_flags.accel_indirect_3_occupied_pid);
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_INDIRECT_3_OCCUPIED Value which, also, Indicates the Completion of AGI3.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGI3 has Completed.
			 */				
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_INDIRECT_3_OCCUPIED;
		
			/*
			 * Clear the agi3_busy Field of the BRAM to Indicate that the AGI3 is Available.
			 */			
			inter_process_shared_info_memory->shared_status_flags.agi3_busy	= 0;
			
			/*
			 * Clear the accel_indirect_3_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGI3.
			 */	
			inter_process_shared_info_memory->shared_status_flags.accel_indirect_3_occupied_pid = 0;		
			
			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGI3.
			 */			
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);			

		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_6_sem Semaphore so that other Userspace Threads can Acces that Part of the Code.
	 */
	up_write(&msi_6_sem);
	
	/*
	 * The Fact that an Interrupt Occured it Means that the AGI3 Has Completed which in other Words Means that the AGI3 is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGI3.
	 */	
	wake_up_interruptible(&ioctl_queue);

	return IRQ_HANDLED;
}

/** OK
  * irq_handler_6()
  * 
  * Started as a Threaded Function by the irq_fast_handler_6() when a MSI 6 Interrupt Occurs.
  * 
  * MSI 6 Interrupt Signifies the Completion of the Acceleration Procedure for the Acceleration Group Scatter/Gather (AGSG).
  * In such Condition the irq_handler_6() Should Gather the Metrics Information that AGSG Stored to the FPGA BRAM and
  * Copy it to the Metrics Kernel Memory Allocation that Corresponds to the Userspace Thread that Occupied the AGSG.
  * 
  */
irqreturn_t irq_handler_6(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 * Pointer of Type struct pid_reserved_memories.
	 * Used to Access the Singly Linked List where each Node Hold Metrics Information and Pointers for each Userspace Application.
	 */	
	struct pid_reserved_memories *search_element = NULL;


	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Kernel Interrupted from Handler 6 [IRQ: %d]\n", driver_name, current->pid, irq);
	#endif

	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Acceleration Group SG Completed\n", driver_name, current->pid);
	#endif	

	/*
	 * Lock the msi_7_sem Semaphore so that only the Current Userspace Thread can Access that part of the Code.
	 */
	down_write(&msi_7_sem);
	
	
	/*
	 * Set the search_element Pointer to Point at the Head (pid_list_head) of the Singly Linked List so that we Can Search the List of Nodes from the Beginning.
	 */	
	search_element = pid_list_head;
	
	/*
	 * Keep Moving Forward in the Singly Linked List for as long as the search_element Pointer has not Reached a NULL Value.
	 */	 
	while(search_element != NULL)
	{
		
		/*
		 * Check if the Current Node's PID Value is Equal to the PID that Occupied the Acceleration Group Scatter/Gather (AGSG).
		 * If this is the Case then we can Copy the AGSG Metrics Information from the FPGA BRAM to the Kernel Metrics Memory.
		 * 
		 * The search_element->shared_repo_virtual_address is a Pointer of the Current Node that Points to a Metrics Kernel Memory Allocation which is
		 * Allocated Specifically for the Userspace Thread with PID Equal to the Current Node's PID (search_element->pid).
		 */			
		if(search_element->pid == inter_process_shared_info_memory->shared_status_flags.accel_sg_0_occupied_pid)
		{
			#ifdef DEBUG_MESSAGES
			printk(KERN_ALERT "[%s-DBG -> MSI IRQ (PID %d)] Found Search Element\n", driver_name, current->pid);
			#endif			

			/*
			 * Copy the "Read Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_read_transactions = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_read_transactions;
			
			/*
			 * Copy the "Read Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_read_bytes = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_read_bytes;
			
			/*
			 * Copy the "Write Transactions" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */						
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_write_transactions = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_write_transactions;
			
			/*
			 * Copy the "Write Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_write_bytes = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_write_bytes;
			
			/*
			 * Copy the "Stream Packets" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_packets = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_packets;
			
			/*
			 * Copy the "Stream Bytes" Metric from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_bytes = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_bytes;

			/*
			 * Copy the "Global Clock Counter Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */	
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_gcc_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_gcc_l;
			
			/*
			 * Copy the "Global Clock Counter Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */							
			search_element->shared_repo_virtual_address->process_metrics.agsg.apm_gcc_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.apm_gcc_u;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_fetch_time_start_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_fetch_time_start_l;
			
			/*
			 * Copy the "CDMA Fetch Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_fetch_time_start_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_fetch_time_start_u;	
			
			/*
			 * Copy the "CDMA Fetch Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_fetch_time_end_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_fetch_time_end_l;
			
			/*
			 * Copy the "CDMA Fetch Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_fetch_time_end_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_fetch_time_end_u;
			
			/*
			 * Copy the "CDMA Send Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_send_time_start_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_send_time_start_l;
			
			/*
			 * Copy the "CDMA Send Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_send_time_start_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_send_time_start_u;	
			
			/*
			 * Copy the "CDMA Send Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_send_time_end_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_send_time_end_l;
			
			/*
			 * Copy the "CDMA Send Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.cdma_send_time_end_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.cdma_send_time_end_u;	
			
			/*
			 * Copy the "Acceleration Starting Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.dma_accel_time_start_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.dma_accel_time_start_l;
			
			/*
			 * Copy the "Acceleration Starting Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.dma_accel_time_start_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.dma_accel_time_start_u;	
						
			/*
			 * Copy the "Acceleration Ending Point Lower Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.dma_accel_time_end_l = inter_process_shared_info_memory->accel_sg_0_shared_metrics.dma_accel_time_end_l;
			
			/*
			 * Copy the "Acceleration Ending Point Upper Register" Time Value from the FPGA BRAM to the Current Node's Kernel Memory Allocation in the Appropriate agsg Structure Field.
			 */				
			search_element->shared_repo_virtual_address->process_metrics.agsg.dma_accel_time_end_u = inter_process_shared_info_memory->accel_sg_0_shared_metrics.dma_accel_time_end_u;
			
			/*
			 * setup_and_send_signal() is Used to Send a Signal to the Userspace Thread that Occupied the AGSG 
			 * to Indicate the Completion of the Acceleration Procedure by the AGSG.
			 * 
			 * setup_and_send_signal() is no longer Used Since it is Replaced by Another Method for Informing the Userspace Thread for the Completion of the Acceleration Procedure.
			 * It is Reserved, though, for Possible Future Usage.
			 */					
			//setup_and_send_signal(DEFAULT_SIGNAL_6, inter_process_shared_info_memory->shared_status_flags.accel_sg_0_occupied_pid);	
			
			/*
			 * Set the Current Node's accel_completed Field with the ACCELERATOR_SG_OCCUPIED Value which, also, Indicates the Completion of AGSG.
			 * The accel_completed Field is Stored inside the Metrics Kernel Memory Allocation that is, also, Mapped to the Corresponding Userspace Thread of the Current List Node.
			 * As a Result, the Userspace Thread Reads the accel_completed Field in Polling Mode to Know when the AGSG has Completed.
			 */				
			search_element->shared_repo_virtual_address->accel_completed |= ACCELERATOR_SG_OCCUPIED;
				
			/*
			 * Clear the agsg_busy Field of the BRAM to Indicate that the AGSG is Available.
			 */					
			inter_process_shared_info_memory->shared_status_flags.agsg_busy	= 0;
			
			/*
			 * Clear the accel_sg_0_occupied_pid Field of the BRAM which Indicates which PID has Occupied the AGSG.
			 */	
			inter_process_shared_info_memory->shared_status_flags.accel_sg_0_occupied_pid = 0;			

			/*
			 * Write an Acknowledgment Value to the Data Register of the GPIO_ACK Peripheral of the FPGA.
			 * The GPIO_ACK Peripheral will then Trigger an Interrupt to the Interrupt Manager (FPGA) to Indicate that the Driver Successfully Handled the MSI Interrupt for the AGSG.
			 */
			write_remote_register(bar0_address_virtual, BAR0_OFFSET_GPIO_ACK, (u32)ACK);	

		}
		 
		/*
		 * Set the search_element Pointer to Point at the Next List Node.
		 */		 
		search_element = search_element->next_pid;
	}

	/*
	 * Unlock the msi_7_sem Semaphore so that other Userspace Threads can Access that Part of the Code.
	 */
	up_write(&msi_7_sem);		
	
	/*
	 * The Fact that an Interrupt Occured it Means that the AGSG Has Completed which in other Words Means that the AGSG is Available.
	 * Wake up the Sleeping Userspace Threads of the ioctl_queue Queue so that they can Claim the AGSG.
	 */	
	wake_up_interruptible(&ioctl_queue);

	return IRQ_HANDLED;
}




/** OK
  * initcode()
  * 
  * Called by the xilinx_pci_driver_init() when Inserting the Driver.
  * It is Used to Execute any Required Code Routine when Initializing the Driver.
  * 
  * @note It is Currently Empty. 
  * 
  */
void initcode(void)
{

}

/** OK
  * write_remote_register()
  * 
  * Write to a 32 Bit Register of the AXI Address Space of the FPGA over the PCIe Bus.
  * 
  * @param address the Base Address to Write to.
  * 
  * @param offset the Offset of the Register.
  * 
  * @param data a 32 Bit Data Value to Write.
  * 
  */
void write_remote_register(u64 *address, u64 offset, u32 data)
{
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> WRITE REMOTE REGISTER (PID %d)] Writing to Remote Register with Offset [0x%016llX]\n", driver_name, current->pid, offset);
	#endif	
	
	/*
	 * Write to the Register of a Given Address and Offset in Byte Alignment(u8 *).
	 */
	iowrite32(data,(u8 *)address+offset);
}

/** OK
  * read_remote_register()
  * 
  * Read a 32 Bit Register from the AXI Address Space of the FPGA over the PCIe Bus.
  * 
  * @param address the Base Address to Read from.
  * 
  * @param offset the Offset of the Register.
  * 
  * @return the Read Data from the Register.
  * 
  */
u32 read_remote_register(u64 *address, u64 offset)
{
	#ifdef DEBUG_MESSAGES
	printk(KERN_ALERT "[%s-DBG -> READ REMOTE REGISTER (PID %d)] Reading from Remote Register with Offset [0x%016llX]\n", driver_name, current->pid, offset);
	#endif
	
	/*
	 * Read the Register from the Given Address and Offset in Byte Alignment(u8 *) and Return the Read Data.
	 */		
	return ioread32((u8 *)address+offset);
}

/** OK
  * setup_and_send_signal()
  * 
  * Used to Send a Signal to a Userspace Process According.
  * 
  * @param signal is the Signal Number that will be Sent.
  * 
  * @param pid is the Process ID of the Process that Should Receive the Signal.
  * 
  */
int setup_and_send_signal(u8 signal, pid_t pid)
{
	struct task_struct *task_from_pid;
	struct siginfo info;
  
	signal_to_pid  = signal;
	
	/*
	 * Set to Zero the Fields of the struct siginfo
	 */
	memset(&info, 0, sizeof(struct siginfo));
	
	/*
	 * Configure the struct siginfo with the Signal to Send 
	 */	
	info.si_signo = signal_to_pid;
	
	/*
	 * Configure the struct siginfo with the SI_USER Flag 
	 */	
	info.si_code  = SI_USER; // Configure SI_QUEUE for STD or SI_KERNEL for RTC
	
	/*
	 * Get the Task According to the Process ID Given for the "send_sig_info" Function
	 */
	if ((task_from_pid = pid_task(find_vpid(pid), PIDTYPE_PID)) == NULL) 
		{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SETUP AND SEND SIGNAL (PID %d)] Cannot Find the Task Associated with the Process ID %d\n", driver_name, current->pid, pid);
		#endif
		} 
 
	/*
	 * Send the Signal to the Process ID which in fact is the Userspace Application
	 */ 
	if (send_sig_info(signal_to_pid, &info, task_from_pid) < 0 ) 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SETUP AND SEND SIGNAL (PID %d)] Signal %d Could Not Be Sent\n", driver_name, current->pid, signal_to_pid);
		#endif
	} 
	else 
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> SETUP AND SEND SIGNAL (PID %d)] Signal %d was Sent to Process ID %d\n", driver_name, current->pid, signal_to_pid, pid);
		#endif		
	}
	
	return(SUCCESS);
}

/** OK
  * xilinx_pci_driver_read_cfg_register()
  * 
  * Used to Read the Registers of the Configuration Space of the AXI Memory Mapped PCIe Endpoint Device.
  * 
  * @param byte_offset is the Offset of the Register that we Desire to Read.
  * 
  * @return The Data that was Read from the Configuration Register
  * 
  * @note This Function is not Used in this Implementation but it is Reserved for Possible Future Usage.
  * 
  */
u32 xilinx_pci_driver_read_cfg_register (u32 byte_offset)
{
	u32 register_data;	
	
	/*
	 * Read the Requested Configuration Register from the PCIe Endpoint Device	 
	 */
	if (pci_read_config_dword(dev, byte_offset, &register_data) < 0) 
	{		
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> READ CONFIGURATION REGISTER (PID %d)] Reading Configuration Register [FAILURE]\n", driver_name, current->pid);
		#endif		
        return (-1);
	} 
	else
	{
		#ifdef DEBUG_MESSAGES
		printk(KERN_ALERT "[%s-DBG -> READ CONFIGURATION REGISTER (PID %d)] Reading Configuration Register [SUCCESS]\n", driver_name, current->pid);
		#endif
	} 
   
	
	/*
	 * Return the Data that was Read by the Configuration Register of the PCIe Endpoint Device.
	 */
	return (register_data);
   
}
       
       
       
   
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dimitrios Bakoyiannis");
MODULE_DESCRIPTION("Xilinx AXI Memory Mapped to PCI Express Core Driver");   
	
