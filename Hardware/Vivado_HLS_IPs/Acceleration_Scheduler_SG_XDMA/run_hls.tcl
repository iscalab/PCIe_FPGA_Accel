open_project Acceleration_Scheduler_SG_XDMA

set_top acceleration_scheduler_sg_xdma

add_files acceleration_scheduler_sg_xdma.cpp

open_solution "solution1"

#The Part Refers to the Xilinx Virtex 7 VC707 FPGA Development Board
set_part {xc7vx485tffg1761-2}
create_clock -period 10 -name default

csynth_design

export_design -format ip_catalog -display_name "Acceleration Scheduler SG XDMA" -version "3.5"

exit
