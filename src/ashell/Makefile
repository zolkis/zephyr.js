# Copyright (c) 2016-2017, Intel Corporation.

ifdef V
$(info Compiling application)
endif

# ASHELL_IDE_PROTOCOL = 0: console mode is used over WebUSB.
# ASHELL_IDE_PROTOCOL = 1: IDE protocol mode is used over WebUSB.
# "make ide ROM=256 ASHELL_IDE_PROTOCOL=0" builds console IDE + WebUSB + UART
# "make ide ROM=256" builds IDE protocol + WebUSB + UART
ASHELL_IDE_PROTOCOL ?= 1

# Adding path for jerry script APIs
DEPS_BASE ?= $(ZJS_BASE)/deps
ZEPHYRINCLUDE += -I$(JERRY_INCLUDE) -I$(DEPS_BASE)

ifeq ($(CONFIG_USB_CDC_ACM),y)
ASHELL_IDE_PROTOCOL = 0
else
ZEPHYRINCLUDE += -I$(ZEPHYR_BASE)/samples/subsys/usb/webusb/src \
                 -I${ZEPHYR_BASE}/include/drivers/usb \
                 -I${ZEPHYR_BASE}/subsys/usb/class \
                 -I$(ZEPHYR_BASE)/include/usb
obj-y += ../../deps/zephyr/samples/subsys/usb/webusb/src/webusb_serial.o
obj-y += webusb-uart.o
endif

ifeq ($(ASHELL_IDE_PROTOCOL),1)
obj-y += ide-comms.o
obj-y += ide-cmd.o
ccflags-y += -DASHELL_IDE_PROTOCOL=1
endif

obj-y += ashell.o
obj-y += file-utils.o
obj-y += jerry-code.o

obj-y += term-ihex.o
obj-y += term-uart.o
obj-y += term-cmd.o
obj-y += ../../deps/ihex/kk_ihex_read.o

ifneq ($(BOARD), linux)
# Select extended ANSI C/POSIX function set in recent Newlib versions
ccflags-y += -D_XOPEN_SOURCE=700
endif
ccflags-y += -I$(O)/include -I$(JERRY_BASE)/jerry-ext/include
ccflags-y += -DZJS_ASHELL -Wall -Werror
