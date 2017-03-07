# Put your stlink folder here so make burn will work.
STLINK=../../../../stlink-utility

SRCS=main.c system_stm32f4xx.c stm32f4xx_it.c newlib_stubs.c my_printf.c imu.c

# Library modules
SRCS += stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c stm32f4xx_dma.c stm32f4xx_sdio.c stm32f4xx_syscfg.c stm32f4xx_exti.c misc.c stm32f4xx_i2c.c stm32f4xx_spi.c

# Discovery specific
SRCS += $(STM_COMMON)/Utilities/STM32F4-Discovery/stm32f4_discovery.c

SRCS += startup_stm32f4xx.s 

# NGHIA - LIBS have to be placed AFTER the c files, else it will not work!
LIBS = -lm -lc

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=imu

#######################################################################################

STM_COMMON=../../..

CC=../../../../gcc-arm-none-eabi-4_8-2014q1/bin/arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -std=c99 -O3 -Wall -Tstm32_flash.ld -DHSE_VALUE=8000000 -g -pedantic
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb-interwork
CFLAGS += -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I. -I../inc

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -I$(STM_COMMON)/Project/USART_Printf/inc

OBJS = $(SRCS:.c=.o)

vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ $(LIBS) -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).dump
	rm -f $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src/*.o
	rm -f $(STM_COMMON)/Utilities/STM32F4-Discovery/stm32f4_discovery.o

# Flash the STM32F4
burn: proj
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x8000000
	arm-none-eabi-objdump -d -S $(PROJ_NAME).elf  > $(PROJ_NAME).dump
