##################################################################################
#                                                                                #
# This Script                                                                    #
# 1.Creates a New Vivado Project                                                 #
# 2.Generates the Block Design Described in "pcie_acceleration_vc707_design.tcl" #
# 3.Imports the Required Constraint File "constraints.xdc"                       #
# 4.Imports the Required HDL Wrapper File "hdl_wrapper.v"                        #
#                                                                                #
##################################################################################

set relative_directory [pwd]

set project_directory $relative_directory/pcie_acceleration_vc707

set ip_repository $relative_directory/Vivado_HLS_IPs

set constraints_directory $relative_directory/Constraints

set hdl_wrapper_directory $relative_directory/HDL_Wrapper

set block_design_directory $relative_directory/Vivado_Block_Design

set src_bd_design_directory $relative_directory/pcie_acceleration_vc707/pcie_acceleration_vc707.srcs/sources_1/bd/pcie_acceleration_vc707_design

#Create a New Project Named "pcie_accel_demo"
create_project pcie_accel_demo $project_directory -part xc7vx485tffg1761-2

#Set the Board Part which is Required for Certain Configurations such as the Uartlite Controller (RS-232)
set_property board_part xilinx.com:vc707:part0:1.2 [current_project]

#Add the HLS IPs before Opening the Block Design
set_property  ip_repo_paths  {Vivado_HLS_IPs/Acceleration_Scheduler_Direct Vivado_HLS_IPs/Acceleration_Scheduler_Indirect Vivado_HLS_IPs/Acceleration_Scheduler_SG_XDMA Vivado_HLS_IPs/DMA_SG_PCIe_Scheduler Vivado_HLS_IPs/Fetch_Scheduler Vivado_HLS_IPs/Interrupt_Manager Vivado_HLS_IPs/Info_Memory_Block Vivado_HLS_IPs/Send_Scheduler Vivado_HLS_IPs/Sobel_Filter} [current_project]
update_ip_catalog

#Add the Block Design
source $block_design_directory/pcie_acceleration_vc707_design.tcl

#Add Constraint Files
add_files -fileset constrs_1 -norecurse $constraints_directory/constraints.xdc
import_files -fileset constrs_1 $constraints_directory/constraints.xdc

#Add the HDL Wrapper
add_files -norecurse -scan_for_includes $hdl_wrapper_directory/hdl_wrapper.v
import_files -norecurse $hdl_wrapper_directory/hdl_wrapper.v
update_compile_order -fileset sources_1
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1


