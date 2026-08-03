# Self-Driving Car

A basic autonomous robot car built on the Raspberry Pi 5. 
It uses ROS2 for communication and OpenCV for computer vision, with features including lane detection to move across circle race track.
This project demonstrates core self-driving concepts in a compact, embedded environment.

# Features
- remote control
- chassis control
- lane detection

# Requirements
- [Rapsberry Pi5 board](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [Raspberry Pi Camera Module](https://www.raspberrypi.com/products/camera-module-v2/)
- [L298N Motor Driver Module](https://components101.com/modules/l293n-motor-driver-module)
- [4WD Robot Car Chassis](https://www.ram-e-shop.com/shop/ro-base-4wd-2floor-4wd-robot-car-chassis-kit-with-speed-encoder-wheels-2-floor-9320?srsltid=AfmBOoqtz2iSJ7eduWpYzpNygmDq9iOMmLDvP2im1m1YjQXuX9EbJaEr)
- [12V 3600mAh Rechargeable Lithium NMC Battery Pack](https://www.indiamart.com/proddetail/wattnine-12v-3600mah-rechargeable-lithium-nmc-battery-pack-2850220132662.html?srsltid=AfmBOopMQlroVSwoj_lOtTL6Bd6ZhxFx8f3iKWj-K_AcWCDwqKWOigmB)
- Wires, pins and connector to connect robot car components (from Arduino kit for example)
- Ubuntu 24.04
- [ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Latest cmake, gcc/clang, gtest
- [Wiringpi](https://github.com/WiringPi/WiringPi/tree/master) to work with Rapsberry Pi5 board pins
  - do not forget to add user to gpio group `sudo chmod g+rw /dev/gpiomem0`
- OpenCV
  - `sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2`
- [LCCV](https://github.com/kbarni/LCCV/tree/main) libcamera bindings for OpenCV it is a small wrapper library that provides access to the Raspberry Pi camera in OpenCV

# Prepare your ROS2 environment
1. According to [official documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files) for every new shell you will need to run this command to have access to the ROS 2 commands:
```
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

# How to build
```
make all
```

# How to test
```
make tests
```

# How to run
```
sudo bash -c "source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 run main-pipeline main_pipeline"
sudo bash -c "source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 run lane-detection lane-detection"
sudo bash -c "chmod g+rw /dev/gpiomem0 && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 run chasiss-control chasiss-control"
```
# How to run with remote control
```
# Server side on robot car
sudo bash -c "source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 run remote-control remote-control 192.168.0.1 8080"
# Client side on laptop
remote-control-app 192.168.0.1 8080
```
