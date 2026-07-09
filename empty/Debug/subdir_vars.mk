################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../empty.syscfg 

C_SRCS += \
../LCD.c \
./ti_msp_dl_config.c \
C:/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../st7789.c \
../ui.c \
../ui_demo.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./LCD.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./st7789.d \
./ui.d \
./ui_demo.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./LCD.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./st7789.o \
./ui.o \
./ui_demo.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"LCD.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"st7789.o" \
"ui.o" \
"ui_demo.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"LCD.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"st7789.d" \
"ui.d" \
"ui_demo.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../LCD.c" \
"./ti_msp_dl_config.c" \
"C:/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../st7789.c" \
"../ui.c" \
"../ui_demo.c" 

SYSCFG_SRCS__QUOTED += \
"../empty.syscfg" 


