# Workaround to propagate enviroment varaible for ros2
#IGNORE1 := $(shell bash -c "source /opt/ros/jazzy/setup.bash; env | sed 's/=/:=/' | sed 's/^/export /' > makeenv_1")
#IGNORE2 := $(shell bash -c "source install/local_setup.bash; env | sed 's/=/:=/' | sed 's/^/export /' > makeenv_2")
#include makeenv_1
#include makeenv_2

all: interfaces common line-detection manual-control obstacle-avoidance path-planning motion-calibration main-pipeline

interfaces:
	colcon build --packages-select interfaces

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

tests:
	colcon test --event-handlers console_direct+

clean:
	rm -Rf build
	rm -Rf install
	rm -Rf log
	#rm -f makeenv_1
	#rm -f makeenv_2
