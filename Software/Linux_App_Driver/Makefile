obj-m := xilinx_pci_driver.o

XILINX_PCI_DRIVER_HOME := $(shell pwd)

XILINX_PCI_DRIVER_KVER := $(shell uname -r)

all:
	make -C /lib/modules/$(XILINX_PCI_DRIVER_KVER)/build M=$(XILINX_PCI_DRIVER_HOME) modules
	g++ ui.cpp -o ui -pthread
clean:
	make -C /lib/modules/$(XILINX_PCI_DRIVER_KVER)/build M=$(XILINX_PCI_DRIVER_HOME) clean


