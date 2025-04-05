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
- [pins scheme](https://docs.arduino.cc/hardware/uno-rev3/)
- [pins scheme](https://robu.in/arduino-pin-configuration/)

Connection between motors and Arduino is made through an H-Bridge in order to control the left and right motors separately.
- H-Bridge is connected to the power supply, Arduino and motors
- [L298N H-Bridge guide](https://www.bluetin.io/guides/l298n-h-bridge-dc-motor-driver-guide/)
- [connection scheme](https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/)

```
RaspberryPi Pins        Arduino Pins                      L298N H-Bridge pins
[31] forward  ---------> [ D0]
[33] backward ---------> [ D1]
[35] left     ---------> [ D2]
[37] right    ---------> [ D3]
                          ...
                         [ D5] -----> engine left pwm      ---> [ENA]
                         [ D6] -----> engine left reverse  ---> [IN1]
                         [ D7] -----> engine left forward  ---> [IN2]
                         [ D8] -----> engine right reverse ---> [IN3]
                         [ D9] -----> engine right forward ---> [IN4]
                         [D10] -----> engine right pwm     ---> [ENB]
                          ...
                         [GND] ------------common wire-------- [GND]  <- -9V battery
           battery 9V -> [PWR]                                 [+12V] <- +9V battery
```

## Turn Left and Right
This car has no a steering mechanism, to workaround this limitation we will send reverse voltage values on each side of car motors to turn left and right.
