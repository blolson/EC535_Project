ifneq ($(KERNELRELEASE),)
	obj-m := led.o
else
	KERNELDIR := $(EC535)/gumstix/oe/linux-2.6.21
	PWD := $(shell pwd)
	ARCH := arm
	CROSS := arm-linux-

CC = $(EC535)/gumstix/oe/cross/bin/arm-linux-gcc
#CC = /ad/eng/courses/ec/ec535/arm-linux/bin/arm-linux-gcc
CFLAGS = -Wall -std=c99 -lm
objects = punchomatic.o i2c.o sensors.o ring_buffer.o ring_buffer_test.o

all: ring_test i2c_test punchomatic modules
i2c_test : CFLAGS = -Wall -lm
i2c_test : i2c_test.c MahonyAHRS.h MahonyAHRS.c
punchomatic : CLAFGS = -Wall -lm
punchomatic : ring_buffer.c ring_buffer.h MahonyAHRS.h MahonyAHRS.c sensors.c sensors.h punchomatic.c i2c.c i2c.h 
ring_test : CC = gcc
ring_test : CFLAG = -Wall -std=c99 -o ring_test -lm
ring_test : ring_buffer.c ring_buffer.h ring_buffer_test.c sensors.c sensors.h
	gcc ring_buffer_test.c ring_buffer.c ring_buffer.h sensors.c sensors.h i2c.h i2c.c MahonyAHRS.h MahonyAHRS.c -Wall -std=c99 -o ring_test -lm
modules : CC = /ad/eng/courses/ec/ec535/arm-linux/bin/arm-linux-gcc
modules :
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) modules	

clean:
	rm i2c_test ring_test punchomatic
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) clean
endif
