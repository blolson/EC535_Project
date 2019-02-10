# Punch-O-Matic
## BU EC535 Embedded Systems Project
### Patrick Dillon (pdillon) & Blade Olson (blolson)

Article on the project:
http://circuitcellar.com/cc-blog/gesture-recognition-in-a-boxing-glove/

### Setup
1. `make`
2. Copy punchomatic, led.ko, & i2c_test (optional) to gumstix board
3. On the gumstix:
```
mknod /dev/led c 61 0
insmod led.ko
./punchomatic
```

### For testing sensors
```
./i2c_test accel - print accelerometer data
./i2c_test gyro - print gyrometer data
./i2c_test mag - print magnotemeter data
./i2c_test all - print all`
./i2c_test ADC - print out values coming from the analog-to-digital converter
```
