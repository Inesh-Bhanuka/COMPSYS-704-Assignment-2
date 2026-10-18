# Command-line build of the STM32CubeIDE project (same sources and flags as STM32CubeIDE/Debug).
#   make          -> build/ALLMEMS1.elf + .bin
#   make flash    -> writes the BlueMS bootloader (0x08000000) and this app (0x08004000) over ST-Link
# Toolchain: Arm GNU 14.2 + xPack OpenOCD in ~/.local/opt (override TC= / OCD= if elsewhere).

TC   ?= $(HOME)/.local/opt/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin/
OCD  ?= $(HOME)/.local/opt/xpack-openocd-0.12.0-6/bin/openocd
CC    = $(TC)arm-none-eabi-gcc
NAME  = build/ALLMEMS1
ROOT  = ../../../..
BLE   = $(ROOT)/Middlewares/ST/BlueNRG-MS
HAL   = hal hal_adc hal_adc_ex hal_cortex hal_crc hal_crc_ex hal_dfsdm hal_dma hal_flash hal_flash_ex \
        hal_gpio hal_i2c hal_i2c_ex hal_pcd hal_pcd_ex hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_rtc \
        hal_rtc_ex hal_sai hal_sai_ex hal_spi hal_spi_ex hal_tim hal_tim_ex hal_uart hal_uart_ex ll_usb

SRCS  = $(addprefix Src/,main.c heading.c step_detector.c sensor_service.c hci_tl_interface.c \
          stm32l4xx_hal_msp.c stm32l4xx_it.c system_stm32l4xx.c) \
        STM32CubeIDE/Application/User/syscalls.c STM32CubeIDE/Application/User/sysmem.c \
        STM32CubeIDE/Application/Startup/startup_stm32l476xx.s \
        $(ROOT)/Drivers/BSP/SensorTile/SensorTile.c $(ROOT)/Drivers/BSP/SensorTile/SensorTile_bus.c \
        $(HAL:%=$(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_%.c) \
        $(addprefix $(BLE)/hci/controller/,bluenrg_gap_aci.c bluenrg_gatt_aci.c bluenrg_hal_aci.c \
          bluenrg_l2cap_aci.c bluenrg_utils_small.c) \
        $(BLE)/hci/hci_le.c $(BLE)/hci/hci_tl_patterns/Basic/hci_tl.c $(BLE)/utils/ble_list.c
OBJS  = $(patsubst %,build/%.o,$(basename $(notdir $(SRCS))))
vpath % $(sort $(dir $(SRCS)))

ARCH    = -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb --specs=nano.specs
CFLAGS  = $(ARCH) -std=c99 -g3 -Og -Wall -ffunction-sections -MMD -MP \
          -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DSTM32_SENSORTILE \
          -IInc -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32L4xx/Include -I$(ROOT)/Drivers/STM32L4xx_HAL_Driver/Inc \
          -I$(ROOT)/Drivers/BSP/SensorTile -I$(ROOT)/Drivers/CMSIS/Include -I$(ROOT)/Drivers/BSP/Components/Common \
          -I$(BLE)/includes -I$(BLE)/utils -I$(BLE)/hci/hci_tl_patterns/Basic
LDFLAGS = $(ARCH) -TSTM32CubeIDE/STM32L476RGTX.ld -Wl,-Map=$(NAME).map -Wl,--gc-sections -static \
          -Wl,--start-group -lc -lm -Wl,--end-group

all: $(NAME).bin

$(NAME).bin: $(NAME).elf
	$(TC)arm-none-eabi-objcopy -O binary $< $@
	$(TC)arm-none-eabi-size $<

$(NAME).elf: $(OBJS) STM32CubeIDE/STM32L476RGTX.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@

build/%.o: %.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: %.s | build
	$(CC) $(ARCH) -g3 -x assembler-with-cpp -c $< -o $@

build:
	mkdir -p $@

# The app is linked at 0x08004000 and needs the BlueMS bootloader (first 16 KB of the BL image) below it.
build/bootloader.bin: Binary/STM32L476JG-SensorTile_ALLMEMS1_BL_v4.2.0.bin | build
	head -c 16384 $< > $@

flash: $(NAME).elf build/bootloader.bin
	$(OCD) -f interface/stlink.cfg -f target/stm32l4x.cfg -c "init; reset halt; \
	  flash write_image erase build/bootloader.bin 0x08000000; flash write_image erase $(NAME).elf; \
	  verify_image build/bootloader.bin 0x08000000; verify_image $(NAME).elf; reset run; shutdown"

clean:
	rm -rf build

-include $(OBJS:.o=.d)
.PHONY: all flash clean
