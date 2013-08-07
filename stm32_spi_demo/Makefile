EXECUTABLE=STM32F4-Discovery_Demo.elf
BIN_IMAGE=STM32F4-Discovery_Demo.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
ARCH=CM3


LIB_STM = ./libstm
FREERTOS_SRC = $(LIB_STM)/FreeRTOS
FREERTOS_INC = $(FREERTOS_SRC)/include/  
FREERTOS_PORT_INC = $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/


TOOLCHAIN_PATH ?= /usr/local/csl/arm-2012.03/arm-none-eabi
C_LIB= $(TOOLCHAIN_PATH)/lib/thumb2

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-ffreestanding -nostdlib




# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

CFLAGS+=-I./

# stm32f4_discovery lib
CFLAGS+=-I$(LIB_STM)/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I$(LIB_STM)/STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-I$(LIB_STM)/STM32F4xx_StdPeriph_Driver/inc/core_support

#STM32F4xx_StdPeriph_Driver\inc
CFLAGS+=-I$(LIB_STM)/STM32F4xx_StdPeriph_Driver/inc

#Utilities
CFLAGS+=-I$(LIB_STM)/Utilities/STM32F4-Discovery -I$(FREERTOS_INC) -I$(FREERTOS_PORT_INC)

#Libs
LIBS += -L./

#Source Files
SRC += system_stm32f4xx.c startup_stm32f4xx.s $(LIB_STM)/Utilities/STM32F4-Discovery/stm32f4_discovery.c \
		$(FREERTOS_SRC)/tasks.c $(FREERTOS_SRC)/list.c $(FREERTOS_SRC)/portable/MemMang/heap_1.c \
		$(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)/port.c 
	

all: $(BIN_IMAGE)


slave:  main.c  $(SRC) 
	$(CC) $(CFLAGS)  $^ -o $(EXECUTABLE)  -L$(LIB_STM)/STM32F4xx_StdPeriph_Driver/build \
		-lSTM32F4xx_StdPeriph_Driver -L$(LIB_STM)/STM32F_USB_OTG_Driver/build -L$(C_LIB) -lc -lm
	$(OBJCOPY) -O binary $(EXECUTABLE) STM32F4-SPI-SLAVE.bin

slave_it:  main.c  $(SRC) 
	$(CC) $(CFLAGS) -DSPI_TI=1 $^ -o $(EXECUTABLE)  -L$(LIB_STM)/STM32F4xx_StdPeriph_Driver/build \
		-lSTM32F4xx_StdPeriph_Driver -L$(LIB_STM)/STM32F_USB_OTG_Driver/build -L$(C_LIB) -lc -lm
	$(OBJCOPY) -O binary $(EXECUTABLE) STM32F4-SPI-SLAVE-IT.bin

master: main.c  $(SRC) 
	$(CC) $(CFLAGS) -DMASTER=1 $^ -o $(EXECUTABLE)  -L$(LIB_STM)/STM32F4xx_StdPeriph_Driver/build \
		-lSTM32F4xx_StdPeriph_Driver -L$(LIB_STM)/STM32F_USB_OTG_Driver/build -L$(C_LIB) -lc -lm
	$(OBJCOPY) -O binary $(EXECUTABLE) STM32F4-SPI-MASTER.bin

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c  $(SRC) 
	$(CC) $(CFLAGS) $^ -o $@  -L$(LIB_STM)/STM32F4xx_StdPeriph_Driver/build \
		-lSTM32F4xx_StdPeriph_Driver -L$(LIB_STM)/STM32F_USB_OTG_Driver/build -L$(C_LIB) -lc -lm 

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf STM32F4-SPI-SLAVE.bin
	rm -rf STM32F4-SPI-MASTER.bin
	rm -rf STM32F4-SPI-SLAVE-IT.bin

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

.PHONY: all clean
