# Arduino Board Firmware

## Basic Information
Arduino board is used to control car motion:
- it is based on changing signal values from 0 to 255 for each pin
- 255 highest voltage programming value
- car has 4 motors (2 for each side)

## Connection Scheme
Arduino UNO R3 has 0-13 digital pins.
- some pins are used for motors
- other pins are used for RaspberryPi commands
- [pins scheme](https://store-usa.arduino.cc/products/arduino-uno-rev3)

Connection between motors and Arduino is made through an H-Bridge in order to control the left and right motors separately.
- H-Bridge is connected to the power supply, Arduino and motors
- [connection scheme](https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/)
  
```
RaspberryPi Pins        Arduino Pins
[21] stop -------------> [ 0]
[22] forward/backward -> [ 1]
[23] left/right -------> [ 2]
                         [ 3]
                         [ 4]
                         [ 5] -----> engine left
                         [ 6] -----> engine right
                         [ 7] -----> engine left high
                         [ 8] -----> engine right low
                         [ 9] -----> engine right high
                         [10]
                         [11] -----> engine left low
                         [12]
                         [13]
```

## Turn Left and Right
This car has no a steering mechanism, to workaround this limitation we will send reverse voltage values on each side of car motors to turn left and right.
