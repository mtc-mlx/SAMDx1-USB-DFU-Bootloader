##############################################################################
BUILD = build
BIN = D11boot

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

ifeq ($(OS)$(MSYSTEM), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif


CFLAGS += -W -Wall --std=gnu99 -Os -g3
CFLAGS += -Wno-address-of-packed-member
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=../linker/samd11d14.ld

INCLUDES += \
  -I../include \
  -I..

SRCS += \
  ../bootloader.c \
  ../usb_descriptors.c

DEFINES += \
  -D__SAMD11D14AM__ \
  -DDONT_USE_CMSIS_INIT \
  -DSTARTUP_FROM_RESET \
  -DAPP_ORIGIN=0x400

CFLAGS += $(INCLUDES) $(DEFINES)
ASFLAGS += $(CFLAGS)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
OBJS += $(BUILD)/startup.o

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).lss size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD)/$(BIN).lss: $(BUILD)/$(BIN).elf
	@echo OBJDUMP $@
	@$(OBJDUMP) -h -S -z $< > $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

$(BUILD)/startup.o: ../startup.s
	@echo CC $@
	@$(CC) -x assembler-with-cpp $(ASFLAGS) -c -o $@ $<

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

