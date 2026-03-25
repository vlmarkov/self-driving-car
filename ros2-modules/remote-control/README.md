# Remote Control

## Basic Information
This module allows to control self-driving car remotely.

## Features
- build/update self-driving software
- start/stop/restart self-driving software
- turn on/off auto-pilot
- remote control car (forward/left/right/backward)
- .json based protocol communication

## Standalone Remote Control Application
1. Server must be run on RaspberyPi (or other control board, e.g. self driving car)
2. Client must be run on remote Linux machine

## How To Run
1. Ros module
```bash
ros2 run remote-control remote_control [IP ADDRES] [PORT]
```
2. Remote control app
```bash
./remote-control-app [IP ADDRES] [PORT]
```

## How to Remotely Control
```bash
# type `exit` to quit
# type `build_sw` to build software
# type `update_sw` to update software
# type `start_sw` to start software
# type `stop_sw` to stop software
# type `restart_sw to restart software
# type `start_auto_pilot to start auto-pilot
# type `stop_auto_pilot to stop auto-pilot
# type `w` to move forward
# type `s` to move backward
# type `a` to turn left
# type `d` to turn right
# type `test_cmd` to run remote-control command sequence
```
