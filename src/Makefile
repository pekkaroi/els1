PROJECT = els
BUILD_DIR = bin

SHARED_DIR = ../CMSIS/DSP/Include ../CMSIS/Core/Include
CFILES = main.c motor.c trapez.c usart.c motion.c configuration.c

DEVICE=stm32f103cb
OOCD_FILE = /home/pekka/stlink.cfg
# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
