# CLI build, for a machine that has no STM32CubeIDE.
#
# .cproject stays authoritative. Every flag below was read out of it rather than
# guessed: Cortex-M4 hard float, -Os, USE_HAL_DRIVER/STM32F411xE, the five source
# folders it compiles, the prebuilt CMSIS-DSP archive and STM32F411CEUX_FLASH.ld.
# Two build systems cannot stay in sync on their own -- if the flags change in
# CubeIDE, they must be changed here too, which is why they are spelled out in
# one place instead of scattered per target.
#
# Sources are globbed per folder, the way CubeIDE treats a source folder, so a
# new .c under Core/Src is picked up by both without editing anything. tools/ is
# deliberately outside the glob: it holds host selftests with their own main()
# and stubbed HAL, and linking those into firmware would be silent nonsense.

TARGET  := PKM_Triase_Proto_1
BUILD   := build

PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

# fpv4-sp-d16 + hard float is not a preference: Middlewares carries the
# libarm_cortexM4lf_math.a variant, and a soft-float build cannot link it.
MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

SRC_DIRS := Core/Src LoRa MAX30102 $(shell find Drivers -type d -name Src)
SOURCES  := $(foreach d,$(SRC_DIRS),$(wildcard $(d)/*.c))
ASM      := Core/Startup/startup_stm32f411ceux.s
OBJECTS  := $(addprefix $(BUILD)/,$(SOURCES:.c=.o) $(ASM:.s=.o))

INCLUDES := -ICore/Inc \
            -IDrivers/STM32F4xx_HAL_Driver/Inc \
            -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
            -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
            -IDrivers/CMSIS/Include \
            -IMiddlewares/ST/ARM/DSP/Inc \
            -ILoRa -IMAX30102

CFLAGS  := $(MCU) -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE $(INCLUDES) \
           -Os -g3 -Wall -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(MCU) -T STM32F411CEUX_FLASH.ld \
           --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map,--cref -static
LIBS    := Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a -lm

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# -x assembler-with-cpp to match CubeIDE; the ST startup file is plain GNU AS
# today, but ST ships these with #ifdef in other families and the flag is free.
$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $^ $(LDFLAGS) $(LIBS) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

# Needs openocd on PATH and an ST-Link on the SWD header. verify re-reads the
# flash: a bad SWD wire writes happily and boots garbage, which looks like a
# firmware bug rather than a cable.
flash: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	        -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)

.PHONY: all flash clean
