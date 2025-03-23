BUILD_DIR    := build
INSTALL_DIR  := install
LOG_DIR      := log
CXX_FLAGS    := -Wall -Wextra -Wpedantic -Wunused-parameter -std=c++23

all: interfaces common lane-detection manual-control obstacle-avoidance path-planning motion-calibration main-pipeline arduino-code

arduino-code:
	mkdir -p $(BUILD_DIR)
	cd ./$(BUILD_DIR) && cmake -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)" ../arduino/ && make

interfaces:
	colcon build --packages-select interfaces

common:
	colcon build --packages-select common --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

main-pipeline:
	colcon build --packages-select main-pipeline --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

lane-detection:
	colcon build --packages-select lane-detection --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

manual-control:
	colcon build --packages-select manual-control --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

obstacle-avoidance:
	colcon build --packages-select obstacle-avoidance --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

path-planning:
	colcon build --packages-select path-planning --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

motion-calibration:
	colcon build --packages-select motion-calibration --cmake-args -DCMAKE_CXX_FLAGS="$(CXX_FLAGS)"

tests:
	colcon test --packages-select interfaces --event-handlers console_direct+
	colcon test --packages-select common --event-handlers console_direct+
	colcon test --packages-select main-pipeline --event-handlers console_direct+
	colcon test --packages-select lane-detection --event-handlers console_direct+
	colcon test --packages-select manual-control --event-handlers console_direct+
	colcon test --packages-select obstacle-avoidance --event-handlers console_direct+
	colcon test --packages-select path-planning --event-handlers console_direct+
	colcon test --packages-select motion-calibration --event-handlers console_direct+
	cd ./$(BUILD_DIR)/tests && ctest

clean:
	rm -Rf $(BUILD_DIR)
	rm -Rf $(INSTALL_DIR)
	rm -Rf $(LOG_DIR)
