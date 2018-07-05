open_project DMA_SG_PCIe_Scheduler

set_top dma_sg_pcie_scheduler

add_files dma_sg_pcie_scheduler.cpp

open_solution "solution1"

#The Part Refers to the Xilinx Virtex 7 VC707 FPGA Development Board
set_part {xc7vx485tffg1761-2}
create_clock -period 10 -name default

csynth_design

export_design -format ip_catalog -display_name "DMA SG PCIe Scheduler" -version "1.0"

exit
