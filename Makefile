project: i2c_test punchomatic

i2c_test: i2c_test.c
	$(EC535)/gumstix/oe/cross/bin/arm-linux-gcc i2c_test.c MahonyAHRS.h MahonyAHRS.c -o i2c_test -std=c99 -Wall -lm

punchomatic: punchomatic.o i2c.o sensors.o
	$(EC535)/gumstix/oe/cross/bin/arm-linux-gcc -o punchomatic -Wall punchomatic.o i2c.o sensors.o

punchomatic.o: punchomatic.c
	$(EC535)/gumstix/oe/cross/bin/arm-linux-gcc -c punchomatic.c -std=c99 -Wall

sensors.o: sensors.c sensors.h
	$(EC535)/gumstix/oe/cross/bin/arm-linux-gcc -c sensors.c -std=c99 -Wall

i2c.o: i2c.c i2c.h
	$(EC535)/gumstix/oe/cross/bin/arm-linux-gcc -c i2c.c -std=c99 -Wall

clean:
	rm i2c_test punchomatic punchomatic.o sensors.o i2c.o
