OPTIMIZATION            = s

#----------------------------------------------------------
#PATH must be set to CodeSourcery/bin
CROSS                      = arm-none-eabi-

GCC                        = $(CROSS)gcc
AS                         = $(CROSS)as
SIZE                       = $(CROSS)size
OBJCOPY                    = $(CROSS)objcopy
OBJDUMP                    = $(CROSS)objdump
NM                         = $(CROSS)nm

#----------------------------------------------------------
MCU 						= STM32L052K8
#MCU 						= STM32L151C8
#MCU                         = STM32F072RB
TARGET_NAME                 = rboot-main
#----------------------------------------------------------
BUILD_DIR                   = build
OUTPUT_DIR                  = output

#----------------------------------------------------------
CMSIS_DIR                   = CMSIS
CMSIS_DEVICE_DIR            = $(CMSIS_DIR)/Device/ST/STM32L0xx
#CMSIS_DEVICE_DIR            = $(CMSIS_DIR)/Device/ST/STM32L1xx
#CMSIS_DEVICE_DIR            = $(CMSIS_DIR)/Device/ST/STM32F0xx
#----------------------------------------------------------
INCLUDE_FOLDERS 			= ./ core board board/ST 
INCLUDE_FOLDERS             += $(CMSIS_DIR)/Include $(CMSIS_DEVICE_DIR)/Include

INCLUDES                    = $(INCLUDE_FOLDERS:%=-I%)
VPATH                      += $(INCLUDE_FOLDERS)
#----------------------------------------------------------
LINKER_SCRIPT				= core/script.ld
#core-dependent part
SRC_AS                      = cortexm.S
SRC_C						+= system.c board.c
SRC_C						+= stm32_power.c stm32_pin.c stm32_uart.c
SRC_C                       += main.c dbg.c printf.c
#SRC_C						+= f_upd_test.c 
#SRC_C						+= flash_update.c 
#test.c


OBJ                         = $(SRC_AS:%.S=%.o) $(SRC_C:%.c=%.o)
#----------------------------------------------------------
DEFINES                     = -D$(MCU) -D__CORTEX
MCU_FLAGS                   = -mcpu=cortex-m3 -mthumb -D__CORTEX_M3 -D__thumb2__=1 -mtune=cortex-m3 -msoft-float -mapcs-frame -std=gnu99
NO_DEFAULTS                 = -fdata-sections -ffunction-sections -fno-hosted -fno-builtin  -nostdlib -nodefaultlibs
FLAGS_CC                    = $(INCLUDES) $(DEFINES) -I. -O$(OPTIMIZATION) -Wall -c -fmessage-length=0 $(MCU_FLAGS) $(NO_DEFAULTS)
FLAGS_LD                    = -Xlinker --gc-sections $(MCU_FLAGS)
#----------------------------------------------------------
all: $(TARGET_NAME).elf

%.elf: $(OBJ)
	@$(GCC) $(FLAGS_LD) -T $(LINKER_SCRIPT) -o $(BUILD_DIR)/$@ $(OBJ:%.o=$(BUILD_DIR)/%.o)
	@echo '-----------------------------------------------------------'
	@$(SIZE) $(BUILD_DIR)/$(TARGET_NAME).elf
	@$(OBJCOPY) -O binary $(BUILD_DIR)/$(TARGET_NAME).elf $(BUILD_DIR)/$(TARGET_NAME).bin
	@$(OBJDUMP) -h -S -z $(BUILD_DIR)/$(TARGET_NAME).elf > $(BUILD_DIR)/$(TARGET_NAME).lss
	@$(NM) -n $(BUILD_DIR)/$(TARGET_NAME).elf > $(BUILD_DIR)/$(TARGET_NAME).sym
	@mkdir -p $(OUTPUT_DIR)
	@mv $(BUILD_DIR)/$(TARGET_NAME).bin $(OUTPUT_DIR)/$(TARGET_NAME).bin
#	@python checksum_calc.py $(OUTPUT_DIR)/$(TARGET_NAME).bin

.c.o:
	@-mkdir -p $(BUILD_DIR)
	@echo CC: $<
	@$(GCC) $(FLAGS_CC) -c ./$< -o $(BUILD_DIR)/$@

.S.o:
	@-mkdir -p $(BUILD_DIR)
	@echo AS_C: $<
	@$(GCC) $(INCLUDES) -I. $(DEFINES) -c -x assembler-with-cpp ./$< -o $(BUILD_DIR)/$@

program:
	ST-LINK_CLI -c SWD -P $(OUTPUT_DIR)/$(TARGET_NAME).bin 0x08000000 -Rst

clean:
	@echo '-----------------------------------------------------------'
	@rm -f build/*.*

test:
	@echo $(VPATH)

.PHONY : all clean program flash
