
################
# Default rule #
################

all: default

# consider every intermediate target as final to avoid deleting intermediate files
.SECONDARY:

# disable builtin rules that overload the build process (and the debug log !!)
.SUFFIXES:
MAKEFLAGS += -r

SHELL =       /bin/bash
#.ONESHELL:

DATE        := $(shell date "+%d %B %G")

VERSION     := 0.0
$(info Building version: $(VERSION))

############
# Platform #
############
PROG     := token

CONFIG_PRODUCTIONS := bin/$(PROG)


SOURCE_PATH   := src/ ../../bagl/src ../../seproxyhal/src
SOURCE_FILES  := $(foreach path, $(SOURCE_PATH),$(shell find $(path) | grep "\.c$$") )
INCLUDES_PATH := include/ ../../seproxyhal/include ../../bagl/include $(dir $(foreach path, $(SOURCE_PATH),$(shell find $(path) | grep "\.h$$") ))

DEFINES := STM32F042x6 USE_HAL_DRIVER gcc __IO=volatile
#DEFINES := ARM_CORE_HAS_VTOR # not present on STM32F0xx (ARM Cortex M0)

DEFINES += FREQUENCY_HZ=48000000
#DEFINES += assert_param\(x\)= 

 
DEFINES += HAVE_GLO091 HAVE_SCREEN
#DEFINES += HAVE_BAGL BAGL_WIDTH=128 BAGL_HEIGHT=32
#DEFINES += HAVE_BAGL_FONT_OPEN_SANS_LIGHT_13
#DEFINES += HAVE_32128A
#DEFINES += HAVE_64128M
#DEFINES += HAVE_PT035HV SCREEN_TEST_AT_BOOT
#DEFINES += BLUENRG_NUCLEO_SHIELD 
#DEFINES += NUCLEO_BASE
DEFINES += MAX_USB_BIDIR_ENDPOINTS=4
DEFINES += HAVE_BL CONFIG_TARGET_ID=0x01000001 NVM_PAGE_SIZE_B=1024
#DEFINES += HAVE_DRAW 

DEFINES += PRINTF\(...\)

DEFINES += VERSION=\"$(VERSION)\"

##############
# Compiler #
##############
CCPATH   := $(BOLOS_ENV)/gcc-arm-none-eabi-5_3-2016q1/bin/
TARGET   := arm-none-eabi-
CC       := $(CCPATH)/$(TARGET)gcc
CFLAGS   := -ggdb2 -Werror=int-to-pointer-cast -Wall -Wextra #-save-temps
CFLAGS   += -O3 -Os
CFLAGS   += -fstack-usage
#CFLAGS   += -O0
CFLAGS   += -mcpu=cortex-m0 -fno-common -mtune=cortex-m0 -mthumb -nostartfiles -mlittle-endian -finline-limit-0 -fdata-sections -ffunction-sections -funsigned-char -funsigned-bitfields -fshort-enums --std=gnu99
# NOT SUPPORTED BY STM3L152 CFLAGS   += -fpack-struct
#-pg --coverage
LD       := $(CCPATH)/$(TARGET)gcc
LDFLAGS  :=  -ggdb2 -Wall 
LDFLAGS  += -O3 -Os
LDFLAGS   += -fstack-usage
#LDFLAGS  += -O0 
LDFLAGS   += -mcpu=cortex-m0 -mthumb -fno-common -ffunction-sections -fdata-sections -fwhole-program -nostartfiles -Wl,--gc-sections 
LDFLAGS  += -Tscript.ld -Wl,-Map,debug/$(PROG).map
#LDFLAGS  += -nodefaultlibs
#LDFLAGS  += -nostdlib -nostdinc
LDLIBS   += -lm
# -mno-unaligned-access 
#-pg --coverage

### computed variables
VPATH := $(dir $(SOURCE_FILES))
OBJECT_FILES := $(sort $(addprefix obj/, $(addsuffix .o, $(basename $(notdir $(SOURCE_FILES))))))
DEPEND_FILES := $(sort $(addprefix dep/, $(addsuffix .d, $(basename $(notdir $(SOURCE_FILES))))))

-include $(DEPEND_FILES)

clean:
	rm -fr obj bin debug dep

prepare:
	@mkdir -p bin obj debug dep

.SECONDEXPANSION:

# default is not to display make commands
log = $(if $(strip $(VERBOSE)),$1,@$1)

default: prepare bin/$(PROG)

script.ld: script.ld.template Makefile
	$(call log,python ../common/preprocess.py $(addprefix -D,$(DEFINES)) -d -s script.ld.template > script.ld)
	
bin/$(PROG): $(OBJECT_FILES) script.ld
	@echo "[LINK] 	$@"
	$(call log,$(call link_cmdline,$(OBJECT_FILES) $(LDLIBS),$@))
	$(CCPATH)/$(TARGET)objcopy -O ihex -S bin/$(PROG) bin/$(PROG).hex
	$(CCPATH)/$(TARGET)objdump -d -S bin/$(PROG) > debug/$(PROG).asm
	$(call log,srec_cat -multiple -Output -Intel bin/token.hex -Intel -crop `cat debug/token.map | grep "_bltext" | grep 0x | tr -s ' '| cut -f2 -d' '` 0x20000000 > bin/token-upgrade.hex)

dep/%.d: %.c Makefile
	@echo "[DEP]    $@"
	@mkdir -p dep
	$(call log,$(call dep_cmdline,$(INCLUDES_PATH), $(DEFINES),$<,$@))

obj/%.o: %.c dep/%.d
	@echo "[CC]	$@"
	$(call log,$(call cc_cmdline,$(INCLUDES_PATH), $(DEFINES),$<,$@))


### BEGIN GCC COMPILER RULES

# link_cmdline(objects,dest)		Macro that is used to format arguments for the linker
link_cmdline = $(LD) $(LDFLAGS) -o $(2) $(1)

# dep_cmdline(include,defines,src($<),dest($@))	Macro that is used to format arguments for the dependency creator
dep_cmdline = $(CC) -M $(CFLAGS) $(addprefix -D,$(2)) $(addprefix -I,$(1)) $(3) | sed 's/\($*\)\.o[ :]*/obj\/\1.o: /g' | sed -e 's/[:\t ][^ ]\+\.c//g' > dep/$(basename $(notdir $(4))).d 2>/dev/null

# cc_cmdline(include,defines,src,dest)	Macro that is used to format arguments for the compiler
cc_cmdline = $(CC) -c $(CFLAGS) $(addprefix -D,$(2)) $(addprefix -I,$(1)) -o $(4) $(3)

### END GCC COMPILER RULES
