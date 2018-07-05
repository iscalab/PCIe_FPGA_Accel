open_project Info_Memory_Block

set_top info_memory_block

add_files info_memory_block.cpp

open_solution "solution1"

#The Part Refers to the Xilinx Virtex 7 VC707 FPGA Development Board
set_part {xc7vx485tffg1761-2}
create_clock -period 10 -name default

csynth_design

export_design -format ip_catalog -display_name "Info Memory Block" -version "1.0"

exit
