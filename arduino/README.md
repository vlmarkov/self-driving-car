# Arduino Board Firmware

## Basic Information
Arduino board is used to control car motion:
- it is based on changing signal values from 0 to 255 for each pin
- 255 highest voltage programming value
- car has 4 motors (2 for each side)

## Requirements
- Aduino UNO rev-3 board
- L298N H-Bridge
- 4 DC motors

## Connection Scheme
Arduino UNO R3 has 0-13 digital pins.
- some pins are used for motors
- other pins are used for RaspberryPi commands
- [official arduino pins scheme](https://docs.arduino.cc/hardware/uno-rev3/)
- [another arduino pins scheme](https://robu.in/arduino-pin-configuration/)

Connection between motors and Arduino is made through an H-Bridge in order to control the left and right motors separately.
- H-Bridge is connected to the power supply, Arduino and motors
- [L298N H-Bridge connection scheme with tutorial v1](https://www.bluetin.io/guides/l298n-h-bridge-dc-motor-driver-guide/)
- [L298N H-Bridge connection scheme with tutorial v2](https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/)
- [L298N H-Bridge connection scheme with tutorial v3](https://www.roboticsinsighto.com/2020/08/how-to-control-motor-driver-with-arduino.html?m=1)

```
RaspberryPi5 Pins       Arduino Uno Pins                     L298N H-Bridge pins
[29] forward  ---------> [ D0]
[31] backward ---------> [ D1]
[33] left     ---------> [ D2]
[35] right    ---------> [ D3]
                          ...
                         [ D5] -----> engine left pwm      ---> [ENA]
                         [ D6] -----> engine left reverse  ---> [IN1]
                         [ D7] -----> engine left forward  ---> [IN2]
                         [ D8] -----> engine right reverse ---> [IN3]
                         [ D9] -----> engine right forward ---> [IN4]
                         [D10] -----> engine right pwm     ---> [ENB]
                          ...
[6] ----common wire----- [GND] ------------common wire-------- [GND]  <- -12V battery
           battery 9V -> [PWR]                                 [+12V] <- +12V battery
```

## How to turn left and right?
This car does not have a steering mechanism, to workaround this limitation send reverse voltage values to the each side of car motors:

a) To turn left: left motors go backward, right motors go forward

b) To turn right: left motors go forward, right motors go backward

c) With fully charged battery on smooth surface you can get ~3 degrees accuracy on turning by sending 150 PWM value for 25ms long
