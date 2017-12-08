CC = $(EC535)/gumstix/oe/cross/bin/arm-linux-gcc
CFLAGS = -Wall -std=c99
objects = punchomatic.o i2c.o sensors.o ring_buffer.o ring_buffer_test.o

all: ring_test i2c_test
i2c_test : CFLAGS = -Wall -std=c99 -lm
i2c_test : i2c_test.c MahonyAHRS.h MahonyAHRS.c
ring_test : CFLAG = -Wall -std=c99 -o ring_test
ring_test : ring_buffer.c ring_buffer.h ring_buffer_test.c sensors.c sensors.h
	gcc ring_buffer_test.c ring_buffer.c ring_buffer.h sensors.c sensors.h i2c.h i2c.c -Wall -std=c99 -o ring_test

clean:
	rm i2c_test ring_test
