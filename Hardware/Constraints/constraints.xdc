#--------------------------------------------------------------------------------
#Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.
#--------------------------------------------------------------------------------
#Tool Version: Vivado v.2015.4 (lin64) Build 1412921 Wed Nov 18 09:44:32 MST 2015
#Date        : 2018
#--------------------------------------------------------------------------------

set_property IOSTANDARD LVCMOS18 [get_ports perst]
set_property PULLUP true [get_ports perst]
set_property LOC AV35 [get_ports perst]

#PCIe Reference Clock (Differential) Ports
set_property PACKAGE_PIN K8 [get_ports REFCLK_p]
set_property PACKAGE_PIN K7 [get_ports REFCLK_n]

#DDR3 Initial Calibration Complete Led Indication Output Port
set_property PACKAGE_PIN AN39 [get_ports init_calib_complete]
set_property IOSTANDARD LVCMOS18 [get_ports init_calib_complete]

set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets pcie_acceleration_vc707_design_i/clocking_wizard/inst/clk_in1_pcie_acceleration_vc707_design_clk_wiz_1_0]

set_property LOC IBUFDS_GTE2_X1Y5 [get_cells refclk_ibuf]
