open_project Sobel_Filter

set_top sobel_filter

add_files sobel.cpp
add_files sobel_operations.cpp
add_files packet_mode_operations.cpp

open_solution "solution1"

#The Part Refers to the Xilinx Virtex 7 VC707 FPGA Development Board
set_part {xc7vx485tffg1761-2}
create_clock -period 10 -name default

csynth_design

export_design -format ip_catalog -display_name "Sobel Filter" -version "5.8"
