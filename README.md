# FPGA Hardware Acceleration over PCIe
This README file contains the following sections:
1. [Overview](#overview)
2. [Requirements](#requirements)
3. [Project structure illustration](#project-structure-illustration)
4. [Instructions on how to setup and run the project](#instructions-on-how-to-setup-and-run-the-project)
5. [Authors-Contact Information](#authors-contact-information)

## Overview
> This project intends to provide a system for hardware acceleration over PCIe on FPGA devices. 
> The hardware part of the system is implemented on the Xilinx's Virtex 7 VC707 FPGA development board. It is actually a
> hardware design that comprizes of hardware accelerators as well as several IP blocks that are required for the acceleration
> process. Part of the hardware design is a PCIe bridge which is used for the communication needs of the FPGA with the
> host system through the PCIe infrastructure. The hardware accelerator implements a Sobel filter algorithm for image processing.
> Several IP blocks such as the Sobel accelerator where developed with Xilinx's Vivado HLS (High Level Synthesis).


> The software components of the system include a userspace application and a kernel driver for the Linux host system and a 
> standalone application for the Microblaze soft processor. The userspace application is developed as a use case where the host
> system offloads image processing tasks from a multi-threaded environment to hardware acceleration units over PCIe. The kernel
> driver establishes PCIe communication between a multi-threaded userspace application and the FPGA hardware design as well 
> as distributing the hardware acceleration resources to the userspace threads. The Microblaze's standalone application is mainly
> required to initialize the hardware design. 

For detailed information regarding the developed system refer to:

1. [Documentation/PCIe_FPGA_Accelerators.pdf](Documentation/PCIe_FPGA_Accelerators.pdf) 
2. [Documentation/Sample_ACM.pdf](Documentation/Sample_ACM.pdf)  
This material is presented to ensure timely dissemination of scholarly and technical work.  
Copyright and all rights therein are retained by authors or by other copyright holders.  
All persons copying this information are expected to adhere to the terms and constraints  
invoked by each author’s copyright. In most cases, these works may not be reposted without  
the explicit permission of the copyright holder.  
3. Publication:  
**Energy-Performance Considerations for Data Offloading to FPGA-based Accelerators over PCIe**  
D. Bakoyannis, O. Tomoutzoglou and G. Kornaros,  
ACM Transactions on Architecture and Code Optimization (TACO), Vol 15, 1, Apr 2018, Article 14  
[[ACM DL](https://dl.acm.org/citation.cfm?id=3180263)]


![System Overview][system_overview]

## Requirements
- Vivado 2015.4 with SDK in order to reqenerate and synthesize the provided hardware block design.
- Vivado HLS 2015.4 to export the custom hardware IPs developed for the needs of the project.
- Debian 8.6.0-amd64 (Jessie) kernel version 3.16 was used to develop and test the userspace 
application and the kernel driver, thus, it is recommended for guaranteed functionality.

## Project Structure Illustration
The main components of the project's structure can be separated in the [`Hardware/`](Hardware/) and [`Software/`](Software/) directories:
- The [`Hardware/`](Hardware/) directory contains the required files to recreate the hardware design that should be downloaded to the VC707 FPGA board. It consists of the following directories and files:
    - [`Vivado_Block_Design/`](Hardware/Vivado_Block_Design/) Contains a TCL script with the description of the FPGA hardware design.
    - [`HDL_Wrapper/`](Hardware/HDL_Wrapper/) The HDL wrapper of the hardware design.
    - [`Constraints/`](Hardware/Constraints/) The constraints for the hardware design.
    - [`Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) Includes the custom IP blocks created in Vivado HLS for the needs of the hardware design.
    - [`create_project.tcl`](Hardware/) The TCL script that should be executed in order to setup the project so that it can be
    ready for synthesis. This script creates a new project, imports the custom IP blocks to the project's repository and
    regenerates the provided block design. Finally it adds the [`hdl_wrapper.v`](Hardware/HDL_Wrapper/) and [`constraints.xdc`](Hardware/Constraints/) files that are
    required for the hardware design.
- The [`Software/`](Software/) directory contains the following sub-directories:
    - [`Linux_App_Driver/`](Software/Linux_App_Driver/) Contains the userspace application and the kernel driver of the Linux host system.
    - [`Microblaze_XSDK/`](Software/Microblaze_XSDK/) Contains the Microblaze's standalone application that was developed with Xilinx's SDK.
- The [`Documentation/`](Documentation/) directory contains the documentation of the developed system.

## Instructions on how to setup and run the project  
1. Download or clone the current Git project.
2. First you must generate the 9 custom IPs with Vivado HLS:
    * Move to the [`Acceleration_Scheduler_Direct/`](Hardware/Vivado_HLS_IPs/Acceleration_Scheduler_Direct/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Acceleration Scheduler Direct IP.
    * Move to the [`Acceleration_Scheduler_Indirect/`](Hardware/Vivado_HLS_IPs/Acceleration_Scheduler_Indirect/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Acceleration Scheduler Indirect IP.
    * Move to the [`Acceleration_Scheduler_SG_XDMA/`](Hardware/Vivado_HLS_IPs/Acceleration_Scheduler_SG_XDMA/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Acceleration Scheduler Scatter/Gather IP.
    * Move to the [`DMA_SG_PCIe_Scheduler/`](Hardware/Vivado_HLS_IPs/DMA_SG_PCIe_Scheduler/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the DMA Scatter/Gather Scheduler IP.
    * Move to the [`Fetch_Scheduler/`](Hardware/Vivado_HLS_IPs/Fetch_Scheduler/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Fetch Scheduler IP.
    * Move to the [`Interrupt_Manager/`](Hardware/Vivado_HLS_IPs/Interrupt_Manager/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Interrupt Manager IP.
    * Move to the [`Info_Memory_Block/`](Hardware/Vivado_HLS_IPs/Info_Memory_Block/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Info Memory Block IP.
    * Move to the [`Interrupt_Manager/`](Hardware/Vivado_HLS_IPs/Interrupt_Manager/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Interrupt Manager IP.
    * Move to the [`Send_Scheduler/`](Hardware/Vivado_HLS_IPs/Send_Scheduler/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Send Scheduler IP.
    * Move to the [`Sobel_Filter/`](Hardware/Vivado_HLS_IPs/Sobel_Filter/) directory which is located in [`Hardware/Vivado_HLS_IPs/`](Hardware/Vivado_HLS_IPs/) directory.
    * Type `/opt/Xilinx/Vivado_HLS/2015.4/bin/vivado_hls ./run_hls.tcl` and press `Enter`.  This action will run the Vivado HLS tool and generate the Sobel Filter (ccelerator) IP.
3. Generate the block design:
    * Move to the [`Hardware/`](Hardware/) directory.
    * Type `/opt/Xilinx/Vivado/2015.4/bin/vivado` and press `Enter`.  This action will launch the Vivado 2015.4 tool.
    * Locate the TCL Console at the bottom of the Vivado GUI, type `source create_project.tcl` and press `Enter`. This action will regenerate the block design. It might require a few minutes to complete.
    * Right click on the `Diagram` layout and choose `Validate Design` or alternatively press `F6`. This action is important to validate that the block design was generated correctly.
    * In the menu bar click on the `Flow` menu item and choose the `Generate Bitstream` option. The Vivado tool will request to run synthesis and implementation. Click `Yes`. The bitstream generation will take long time to complete.
4. Set the Xilinx SDK project:
    * In Vivado click the `File` menu item of the menu bar and choose the `Export -> Export Hardware` option.
    * In the dialog box that appears make sure to check the `Include bitstream` option and then click `OK`. This action will set a directory that contains the hardware bitstream for the Xilinx SDK.
    * In Vivado click the `File` menu item of the menu bar and choose the `Launch SDK` option to start the Xilinx SDK.
    * In the `Project Explorer` on the left pane locate the `pcie_acceleration_vc707_design_wrapper_hw_platform_0` folder which contains the hardware description as well as the drivers of the custom HLS IPs. Right click that folder and choose `New->Project`.
    * In the wizard that appears expand the `Xilinx` option, choose `Application Project` and click `Next`.
    * Give a Project name for your application (e.g pcie_acceleration_vc707) and click `Next` leaving the rest options intact.
    * Choose the `Hello World` template and click `Finish`.
    * Open a file explorer and move to the `/Hardware/pcie_acceleration_vc707/pcie_acceleration_vc707.sdk/pcie_acceleration_vc707/` directory.
    * Replace the `src folder` of this directory with the [`src/`](Software/Microblaze_XSDK/) directory located in the [`Software/Microblaze_XSDK/`](Software/Microblaze_XSDK/). The src folder contains the .c and .h files that are required for the project as well as the linker script file.
5. Program the FPGA and launch the Microblaze application:
    * In the menu bar of the Xilinx SDK click the `Xilinx Tools` menu item and choose the `Program FPGA` option.
    * In the dialog box that appears click `Program` leaving the rest options intact. Wait until the hardware bitstream is downloaded to the FPGA.
6. Open a serial terminal (e.g Minicom) which will be used by the Microblaze to print messages:
    * in a terminal type `minicom -s` to configure the Minicom options.
    * In the menu options that appear choose `Serial port setup`.
    * Type `A` to set the serial device. Set it as `dev/ttyUSB0` and click `Enter`. If the FPGA does not print any messages check also as `dev/ttyUSB1` and `dev/ttyUSB2`.
    * Type `E` to set the Baud rate, then type `C` to set it as `9600` and click twice `Enter to close the configurations`.
    * Choose `Exit` in the menu options. This action will start the Minicom with the latest configurations.
7. Run the Microblaze application:
    * In the menu bar of the Xilinx SDK click the `Project` menu item and choose the `Build All` option or alernatively press `Ctrl+B`.
    * In the menu bar of the Xilinx SDK click the `Run` menu item and choose the `Run As -> 4 Launch on Hardware (GDB)`. This action will load the application to the Microblaze. Normally, you will see at minicom several messages that the Microblaze prints while it makes the system initiation.
8. At this moment the FPGA is configured with the new hardware system. Restart the host machine so that it can locate the new endpoint device of the VC707 FPGA board:
    * Once the host machine is restarted open a terminal and run `lspci -v` to list the PCIe endpoint devices. You should locate a record of the `Co-processor: Xilinx Corporation Device 7022`. If you fail to locate the device try restarting the host machine.
9. Load the kernel driver:
    * Open a terminal and move to the [`Software/Linux_App_Driver`](Software/Linux_App_Driver/) directory.
    * Type `make` to build the driver and the application.
    * Type `./make_device` which runs a script that creates a new node of the driver under the `/dev/` directory.
    * Type `insmod ./xilinx_pci_driver.ko` to load the pcie driver of the FPGA device.
10. Run the user application:
    * Type `./ui path_file thread_iterations threads_number save_flag test_iterations`.
    * Replace the the arguments above with the desired values.
        * path_file: The directory along with the file name of the image that should be processed.
        * thread_iterations: You can request  multiple times to access the acceleration resources.
        * threads_number: The number of threads that will be generated.
        * save_flag:
            * 0 Do not save the processed image.
            * 1 Save the processed image in each iteration of the thread.
            * 2 Save the processed image in the last iteration of the thread.
        * test_iterations: Run the same test multiple times.
    * For example type `./ui Results/vga.bmp 100 16 1 10`. The [`Results`](Software/Linux_App_Driver/Results/) directory includes a few bitmap files for testing purposes. This directory is , also, used to save the metrics from the application.

## Authors-Contact Information
Feel free to contact any of the main authors for questions or recommendations:  
Dimitrios Bakoyiannis (d.bakoyiannis@gmail.com)  
Othon Tomoutzoglou (otto_sta@hotmail.com)  
Georgios Kornaros (kornaros@gmail.com)  

[system_overview]: /GitHub_Images/system_overview.png "Simplistic Overview of the System"
