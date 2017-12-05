i2c_test: i2c_test.c
	$(EC535)/gumstix/oe/cross/bin/arm-linux-gcc i2c_test.c MahonyAHRS.h MahonyAHRS.c -o i2c_test -std=c99 -Wall -lm

clean:
	rm i2c_test
