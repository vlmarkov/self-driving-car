all: common main-pipeline line-detection manual-control obstacle-avoidance path-planning motion-calibration

configure-ros2-build:
	source /opt/ros/jazzy/setup.bash
	. install/setup.bash

configure-ros2-run:
	source install/local_setup.bash

common:
	colcon build --packages-select common

main-pipeline:
	colcon build --packages-select main-pipeline

line-detection:
	colcon build --packages-select line-detection

manual-control:
	colcon build --packages-select manual-control

obstacle-avoidance:
	colcon build --packages-select obstacle-avoidance

path-planning:
	colcon build --packages-select path-planning

motion-calibration:
	colcon build --packages-select motion-calibration

clear:
	rm -Rf build
	rm -Rf install
	rm -Rf log
