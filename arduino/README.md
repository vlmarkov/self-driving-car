# Arduino Board Firmware

## Basic Information
Arduino board is used to control car motion:
- it is based on changing signal values from 0 to 255 for each pin
- 255 highest voltage programming value
- car has 4 motors (2 for each side)

## Connection Scheme
Connection between motors and Arduino is made through an H-Bridge in order to control the left and right motors separately.

H-Bridge is connected to the power supply, arduino and motors.

TODO picture

## Pins Descriptions

TODO description

## Turn Left and Right
This car has no a steering mechanism, to workaround this limitation we will send reverse voltage values on each side of car motors to turn left and right.

TODO calibration procedure, sterring algorithm
