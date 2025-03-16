# Manual Control

## Basic Information
This module allows to control remotely self-driving car

## Features
- on/off auto-pilot mode
- command to move forward, left, right, backward directions

## Manual Control App
1. Server must be run on RaspberyPi (or other control board, e.g. self driving car)
> It is done via integration server source code into manual-control ros-module
2. Client must be run on remote linux machine

## How To Run
1. Ros module
```bash
ros2 run manual-control manual_control [IP ADDRES] [PORT]
```
2. Manual control app
```bash
./manual-control-app [IP ADDRES] [PORT]
```

## How to Remotely Control
```bash
# Press 'e' to turn on auto-pilot mode
# Press 'r' to turn off auto-pilot mode
# Press 'w' to increase forward movement
# Press 's' to increase backward movement
# Press 'a' increase left steering angle
# Press 'd' increase right steering angle
# Press 'q' to quit
```
