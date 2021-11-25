# DiVA bootloader

Based off the [Foboot](https://github.com/im-tomu/foboot) bootloader.

This bootloader is stored in the first few sectors of FLASH and is used to facilitate in the field firmware updates without connecting to the JTAG test-points on the DiVA board.

The bootloader operates using the standard dfu interface defined by the USB standard. The project comes in two parts, a software part and a hardware part. The hardware part creates the USB peripheral and an SoC inside the FPGA, and the software is set up to run on that SoC.

## Requirements

To build the hardware/firmware, you need:

* Python 3.5+
* nextpnr-ecp5
* Yosys
* Git
* RISC-V toolchain

Subproject hardware dependencies will be taken care of with `lxbuildenv`.


## Building the project

The hardware half will take care of building the software half. Therefore, to build, enter the `hw/` directory and run:

```
$ python3 DiVA-bootloader.py
```

This will automatically update a set of local dependencies for the project, build the riscv firmware, create a verilog project and start synthesizing for the ecp5. The resulting bitstream will contain the firmware embedded into a blockram memory.

### Usage

This bitstream is loaded into FLASH of the DiVA board at address 0x00000000, So it runs on a power on reset. 

If the top button is pressed then it will stay in the bootloader and enumerate as a USB DFU device. 

If the button is not held then the device will reconfigure using a bitstream located at 0x400000 in the FLASH.

#### Using `dfu-util` to load a bitstream

Just do
```sh
dfu-util -D new_bitstream.dfu
```
to copy into the SPI flash.
