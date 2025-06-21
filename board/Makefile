PROJECT = MPU6050

BUILD_DIR = build

CXXFILES = \
	src/systick.cpp \
	src/uart.cpp \
	src/i2c.cpp \
	src/mpu6050.cpp \
	src/main.cpp

DEVICE=stm32f103rb

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
