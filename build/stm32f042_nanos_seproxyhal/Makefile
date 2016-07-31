
################
# Default rule #
################

all: default

# consider every intermediate target as final to avoid deleting intermediate files
.SECONDARY:

.SECONDEXPANSION:


# disable builtin rules that overload the build process (and the debug log !!)
.SUFFIXES:
MAKEFLAGS += -r

SHELL =       /bin/bash
#.ONESHELL:

DATE        := $(shell date "+%d %B %G")

#set this variable with the APP_ADDRESS of the from bootloader address.
# = ALIGN(FLASHPAGE, MAX( lastaddr(old bl) , lastaddr(new bl) ) )
#UPGRADE_BL_LOAD_ADDRESS := 0x08003000

BOOTLOADER  := ../stm32f042_nanos_bootloader
#VERSION     := $(shell date "+%Y%m%d")
VERSION     := 1.0

#extract data from the used bootloader we're using to load this code
LOAD_ADDRESS := $(shell cat $(BOOTLOADER)/debug/token.map | grep "_apptext" | grep 0x | tr -s ' '| cut -f2 -d' ' |head -n1)
BL_BS_NVRAM  :=$(shell cat $(BOOTLOADER)/debug/token.map | grep "_bs_nvram" | grep 0x | tr -s ' '| cut -f2 -d' ' |head -n1)
BOOTADDR = $$((`cat debug/token.map | grep "          Reset_Handler$$" | grep 0x | tr -s ' '| cut -f2 -d' '` | 1))

$(info Building version: $(VERSION))

############
# Platform #
############
PROG     := token

CONFIG_PRODUCTIONS := bin/$(PROG)


SOURCE_PATH   := src/ ../../bagl/src ../../seproxyhal/src
SOURCE_FILES  := $(foreach path, $(SOURCE_PATH),$(shell find $(path) | grep "\.c$$") )
INCLUDES_PATH := include/ include/stm32f0_hal ../../bagl/include ../../seproxyhal/include $(dir $(foreach path, $(SOURCE_PATH),$(shell find $(path) | grep "\.h$$") ))
# for bootsector's configuration
INCLUDES_PATH += $(BOOTLOADER)/src

DEFINES := STM32F042x6 USE_HAL_DRIVER gcc __IO=volatile

DEFINES += FREQUENCY_HZ=48000000
#DEFINES += assert_param\(x\)= 

DEFINES += PRINTF\(...\)

DEFINES += HAVE_SE
#DEFINES += HAVE_SE_PERSO # Remember to undefined HAVE_BAGL_GLYPH* when building HAVE_SE_PERSO
DEFINES += HAVE_SEPROXYHAL
#DEFINES += TESTLONGLABEL
#DEFINES += DEMO_MODE # for shooting sessions, a set of predefined display transitions
#DEFINES += PROBER_MODE # a screen all white to detect if screen are correctly wired

# NOTE: 
#DEFINES += HAVE_32128A 
DEFINES += HAVE_GLO091
DEFINES += HAVE_BAGL BAGL_WIDTH=128 BAGL_HEIGHT=32
DEFINES += HAVE_BAGL_FONT_OPEN_SANS_REGULAR_11PX
DEFINES += HAVE_BAGL_FONT_OPEN_SANS_EXTRABOLD_11PX HAVE_BAGL_FONT_OPEN_SANS_LIGHT_16PX
# Disable GLYPH when building HAVE_SE_PERSO
DEFINES += HAVE_BAGL_GLYPH_ICON_CHECK HAVE_BAGL_GLYPH_ICON_CROSS HAVE_BAGL_GLYPH_ICON_CHECK_BADGE HAVE_BAGL_GLYPH_ICON_LEFT HAVE_BAGL_GLYPH_ICON_RIGHT HAVE_BAGL_GLYPH_ICON_UP HAVE_BAGL_GLYPH_ICON_DOWN HAVE_BAGL_GLYPH_LOGO_LEDGER_MINI HAVE_BAGL_GLYPH_ICON_CROSS_BADGE HAVE_BAGL_GLYPH_ICON_DASHBOARD_BADGE HAVE_BAGL_GLYPH_ICON_PLUS HAVE_BAGL_GLYPH_ICON_LESS HAVE_BAGL_GLYPH_ICON_TOGGLE_ON HAVE_BAGL_GLYPH_ICON_TOGGLE_OFF HAVE_BAGL_GLYPH_ICON_LOADING_BADGE HAVE_BAGL_GLYPH_ICON_COG_BADGE HAVE_BAGL_GLYPH_ICON_WARNING_BADGE HAVE_BAGL_GLYPH_ICON_DOWNLOAD_BADGE HAVE_BAGL_GLYPH_ICON_TRANSACTION_BADGE HAVE_BAGL_GLYPH_ICON_BITCOIN_BADGE HAVE_BAGL_GLYPH_ICON_ETHEREUM_BADGE HAVE_BAGL_GLYPH_ICON_EYE_BADGE HAVE_BAGL_GLYPH_ICON_LOCK_BADGE HAVE_BAGL_GLYPH_ICON_PEOPLE_BADGE

#DEFINES += HAVE_64128M
#DEFINES += HAVE_PT035HV SCREEN_TEST_AT_BOOT
#DEFINES += BLUENRG_NUCLEO_SHIELD 
#DEFINES += NUCLEO_BASE
DEFINES += MAX_USB_BIDIR_ENDPOINTS=4 MAX_USB_ENDPOINT_SIZE=64

#DEFINES += DEBUG_BUTTON_LINK_DEBUG DEBUG_BUTTON_LINK_DEBUG_INITIAL=2
DEFINES += HAVE_UNSEC_CHUNK_READ

#smartcardano, the screen is not folded
#DEFINES += HAVE_REVERSED_BUTTON
#for dev only, as afterward the user emulation disappears
#DEFINES += DEV_RECOVERY_IS_FLASHBACKREQ


DEFINES += VERSION=\"$(VERSION)\"
DEFINES += BL_BS_NVRAM=$(BL_BS_NVRAM)
ifneq ($(UPGRADE_BL_LOAD_ADDRESS),)
	DEFINES += BOOTLOADER_UPGRADE
	DEFINES += LOAD_ADDRESS=$(UPGRADE_BL_LOAD_ADDRESS)
	DEFINES += NVM_PAGE_SIZE_B=1024


# insert in the dependency chain
dep/%.d: src/upgrade_content.bytes

src/upgrade_content.bytes: $(BOOTLOADER)/bin/token-upgrade.hex makefiles-depends
	# generate erase value for data between bootloader and the loaded code
	# order of addresses depends on which one is the bigger/smaller code to patch
	$(call log,srec_cat -Output -Intel $(BOOTLOADER)/bin/token-upgrade.hex -Intel -generate -maximum-addr $(BOOTLOADER)/bin/token-upgrade.hex -Intel $(UPGRADE_BL_LOAD_ADDRESS) -constant 0xFF > bin/temp_bl_upgrade.hex)
	$(call log,python ../common/hex2carray.py --hex bin/temp_bl_upgrade.hex  > src/upgrade_content.bytes)
	
else
	DEFINES += LOAD_ADDRESS=$(LOAD_ADDRESS)
endif

##############
# Compiler #
##############
CCPATH   := $(BOLOS_ENV)/gcc-arm-none-eabi-5_3-2016q1/bin/
TARGET   := arm-none-eabi-
CC       := $(CCPATH)/$(TARGET)gcc
CFLAGS   := -ggdb2 -Werror=int-to-pointer-cast -Wall -Wextra #-save-temps
CFLAGS   += -O3 -Os
#CFLAGS   += -O0
CFLAGS   += -mcpu=cortex-m0 -fno-common -mtune=cortex-m0 -mthumb -nostartfiles -mlittle-endian -finline-limit-0 -fdata-sections -ffunction-sections -funsigned-char -funsigned-bitfields -fshort-enums --std=gnu99
# NOT SUPPORTED BY STM3L152 CFLAGS   += -fpack-struct
#-pg --coverage
LD       := $(CCPATH)/$(TARGET)gcc
LDFLAGS  :=  -ggdb2 -Wall 
LDFLAGS  += -O3 -Os
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


# default is not to display make commands
log = $(if $(strip $(VERBOSE)),$1,@$1)

default: prepare bin/$(PROG)

makefiles-depends: Makefile

script.ld: script.ld.template $(BOOTLOADER)/bin/token.hex makefiles-depends
	$(call log,python ../common/preprocess.py $(addprefix -D,$(DEFINES)) -d -s script.ld.template > script.ld)

bin/$(PROG): $(OBJECT_FILES) script.ld
	@echo "[LINK] 	$@"
	$(call log,$(call link_cmdline,$(OBJECT_FILES) $(LDLIBS),$@))
	$(CCPATH)/$(TARGET)objcopy -O ihex -S bin/$(PROG) bin/$(PROG).hex
	$(CCPATH)/$(TARGET)objdump -d -S bin/$(PROG) > debug/$(PROG).asm

	srec_cat -multiple -Output -Intel \
		bin/token.hex -Intel \
		-fill 0xFF 0x08000000 -minimum-addr bin/token.hex -Intel \
		$(BOOTLOADER)/bin/token.hex -Intel \
		-random-fill -maximum-addr bin/token.hex -Intel 0x08007FB0 \
		-crop 0x08000000 0x20000000 \
		-generate $(BL_BS_NVRAM) $$(($(BL_BS_NVRAM)+4)) --l-e-constant 0x51145E82 4 \
		-generate $$(($(BL_BS_NVRAM)+4)) $$(($(BL_BS_NVRAM)+8)) --l-e-constant $(BOOTADDR) 4 \
		> bin/all_in_one.hex 2>/dev/null
	python ../common/signMCUhex.py --hex bin/all_in_one.hex --signedHex bin/all_in_one.signed.hex --sigAddr 0x08007FB0 --key 4e6f2c20776520646f6e277420707574206b657973206f6e206769746875620a 
	srec_cat -multiple -Output -Intel \
		bin/all_in_one.signed.hex -Intel \
		-crop -minimum-addr bin/token.hex -Intel 0x20000000 \
		> bin/token.loadable.hex

load:
	python -m ledgerblue.loadMCU --targetId 0x01000001 --fileName bin/token.loadable.hex 

dep/%.d: %.c makefiles-depends
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
