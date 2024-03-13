##########################################################################################################################
# File automatically-generated by STM32forVSCode
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = stm32F4_ax58100_hw


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/stm32f4xx_it.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/misc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cec.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dsi.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash_ramfunc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmpi2c.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_lptim.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_qspi.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sai.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spdifrx.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c \
Lib/bsp/src/EXIT.c \
Lib/bsp/src/RST.c \
Lib/bsp/src/SPI1.c \
Lib/bsp/src/Timer.c \
Lib/bsp/src/bsp_can.c \
Lib/bsp/src/bsp_delay.c \
Lib/bsp/src/bsp_led.c \
Lib/bsp/src/bsp_tim.c \
Lib/bsp/src/bsp_usart.c \
Lib/bsp/src/system_stm32f4xx.c \
Lib/motor_app/src/unitreeA1_cmd.c \
Lib/ssc/src/bootmode.c \
Lib/ssc/src/diag.c \
Lib/ssc/src/ecataoe.c \
Lib/ssc/src/ecatcoe.c \
Lib/ssc/src/ecateoe.c \
Lib/ssc/src/ecatfoe.c \
Lib/ssc/src/ecatslv.c \
Lib/ssc/src/ecatsoe.c \
Lib/ssc/src/emcy.c \
Lib/ssc/src/eoeappl.c \
Lib/ssc/src/fc1100hw.c \
Lib/ssc/src/mailbox.c \
Lib/ssc/src/objdef.c \
Lib/ssc/src/sdoserv.c \
Lib/ssc_app/src/aoeappl.c \
Lib/ssc_app/src/coeappl.c \
Lib/ssc_app/src/ecatappl.c \
Lib/ssc_app/src/el9800appl.c \
Lib/ssc_app/src/el9800hw.c \
Lib/ssc_app/src/foeappl.c


CPP_SOURCES = \


# ASM sources
ASM_SOURCES =  \
startup_stm32f427_437xx.s



#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
POSTFIX = "
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
GCC_PATH="/usr/bin
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++$(POSTFIX)
CC = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX)
AS = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX) -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy$(POSTFIX)
SZ = $(GCC_PATH)/$(PREFIX)size$(POSTFIX)
else
CXX = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32F427_437xx \
-DUSE_STDPERIPH_DRIVER


# CXX defines
CXX_DEFS =  \
-DSTM32F427_437xx \
-DUSE_STDPERIPH_DRIVER


# AS includes
AS_INCLUDES = \

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32F4xx_StdPeriph_Driver/inc \
-ILib/bsp/include \
-ILib/motor_app/include \
-ILib/ssc/include \
-ILib/ssc_app/include



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(CXX_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -feliminate-unused-debug-types

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -ggdb
CXXFLAGS += -g -gdwarf -ggdb
endif

# Add additional flags
CFLAGS += -Wall -fdata-sections -ffunction-sections 
ASFLAGS += -Wall -fdata-sections -ffunction-sections 
CXXFLAGS += 

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F427VGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = \


# Additional LD Flags from config file
ADDITIONALLDFLAGS = -specs=nano.specs 

LDFLAGS = $(MCU) $(ADDITIONALLDFLAGS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of cpp program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of C objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
UPPER_CASE_ASM_SOURCES = $(filter %.S,$(ASM_SOURCES))
LOWER_CASE_ASM_SOURCES = $(filter %.s,$(ASM_SOURCES))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(UPPER_CASE_ASM_SOURCES:.S=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(LOWER_CASE_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cxx STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cxx=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c STM32Make.make | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) STM32Make.make
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	"/usr/bin/openocd" -f ./openocd.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

#######################################
# erase
#######################################
erase: $(BUILD_DIR)/$(TARGET).elf
	"/usr/bin/openocd" -f ./openocd.cfg -c "init; reset halt; stm32f4x mass_erase 0; exit"

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# custom makefile rules
#######################################


	
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***