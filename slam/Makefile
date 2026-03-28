BUILD_DIR   := build

# Allows to manually set or disable unit tests
ENABLE_TESTS               := ON
# Allows to manually set or disable compiler warings VALID FOR GCC OR CLANG
ENABLE_COMPILER_OPTIONS    := ON
# Allows to manually set or disable c++23 standard features
ENABLE_LATEST_CPP_STANDARD := ON

all: configure slam

configure:
	mkdir -p $(BUILD_DIR)
	cd ./$(BUILD_DIR) && cmake ..

slam:
	cd ./$(BUILD_DIR) && make all

test:
	cd ./$(BUILD_DIR) && ctest

clean:
	rm -Rf ./$(BUILD_DIR)
