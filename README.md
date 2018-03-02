# S32V234 vision processing drivers

## Overview

Vision processing pipeline of S32V234 is aimed for fast, massively parallel image operations (APEX) and Image Signal Processing of the camera input
(ISP). The package provides Linux drivers for HW accelerators present on NXP S32V234 device.

## Version

BLN_VISION_SDK_RTM_1.1.0

## Licensing

All driver code is provided under BSD license. See COPYING.BSD for exact terms 

## Kernel modules in Linux OS 

To run an application using the S32V234 vision processing pipeline including camera I/F, ISP and
APEX2 accelerators a number of kernel (*.ko) modules are required. 

In Linux OS, the drivers need to be build and installed as a kernel modules into running OS. User can
build and install the drivers using following steps.

## General Preparation
Export necessary environment variables:
- export ARCH=arm
  - Target architecture
- export CROSS_COMPILE=aarch64-linux-gnu-
  - Compiler prefix specification
- export LINUX_S32V234_DIR=/linux/sources/directory/for/s32v234/
  - Path to the Linux sources (same kernel as is running on the board)
- export PATH=/path/to/your/compiler/binaries:$PATH
  - Path to the compiler binaries

## Deploying the Kernel Modules

Following kernel modules can be built:

Kernel module | Path to makefile 
--------------|-----------------
apex.ko       | s32v234_sdk/libs/apex/drivers/kernel/build-v234ce-gnu-linux-d 
oal_cma.ko    | s32v234_sdk/libs/utils/oal/kernel/build-v234ce-gnu-linux-d
csi.ko        | s32v234_sdk/libs/isp/csi/kernel/build-v234ce-gnu-linux-d
cam.ko        | s32v234_sdk/libs/isp/cam_generic/kernel/build-v234ce-gnu-linux-d
seq.ko        | s32v234_sdk/libs/isp/sequencer/kernel/build-v234ce-gnu-linux-d
fdma.ko       | s32v234_sdk/libs/isp/fdma/kernel/build-v234ce-gnu-linux-d
h264enc.ko    | s32v234_sdk/libs/isp/h264denc/kernel/build-v234ce-gnu-linux-d
h264dcd.ko    | s32v234_sdk/libs/isp/h264dec/kernel/build-v234ce-gnu-linux-d
jpegdcd.ko    | s32v234_sdk/libs/isp/jpegdec/kernel/build-v234ce-gnu-linux-d
viulite.ko    | s32v234_sdk/libs/isp/viu/kernel/build-v234ce-gnu-linux-d
sram.ko       | s32v234_sdk/libs/isp/sram/kernel/build-v234ce-gnu-linux-d


### Building kernel modules
```
./build.sh
```
The result of the build will be in "bin" folder 

### Using the Kernel Modules
Prior to running Vision SDK demos the appropriate kernel modules must be inserted. Please use the following list of commands to insert the modules manually:
insmod /s32v234/apex.ko
insmod /s32v234/oal_cma.ko
insmod /s32v234/csi.ko
insmod /s32v234/cam.ko
insmod /s32v234/seq.ko
insmod /s32v234/fdma.ko
insmod /s32v234/h264enc.ko
insmod /s32v234/h264dcd.ko
insmod /s32v234/jpegdcd.ko
insmod /s32v234/viulite.ko

*NOTE:* While inserting the kernel modules it is important to arrange the insmod commands in an
order (e.g. given above) which complies to several dependencies between the modules. Explicitly:
- oal_cma.ko has to be inserted prior to fdma.ko. FDMA driver relies on the ability to allocate
portion of SRAM through the OAL module.
- csi.ko has to be inserted prior cam.ko (connection to “frame start/end” IRQ handling on CSI)
- If ISP functionality will be used the seq.ko has to be inserted prior to fdma.ko. There is a
symbol dependency between the modules
