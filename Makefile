CC=gcc
CXX=g++

CFLAGS=-O2 -Wall -Iinclude
CXXFLAGS=-O2 -Wall -Iinclude -Isrc

BUILD_DIR=build

C_SRCS = \
src/vl53l5cx_api.c \
src/vl53l5cx_plugin_xtalk.c \
src/vl53l5cx_plugin_detection_thresholds.c \
src/vl53l5cx_plugin_motion_indicator.c

CPP_SRCS = \
src/platform.cpp

C_OBJS=$(patsubst src/%.c,$(BUILD_DIR)/%.o,$(C_SRCS))
CPP_OBJS=$(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(CPP_SRCS))

LIB_OBJS=$(C_OBJS) $(CPP_OBJS)

MAIN_TARGET=main

all: $(MAIN_TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Library C files
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Library C++ files
$(BUILD_DIR)/%.o: src/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Main executable object
$(BUILD_DIR)/main.o: main.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Build main
$(MAIN_TARGET): $(LIB_OBJS) $(BUILD_DIR)/main.o
	$(CXX) $(BUILD_DIR)/main.o $(LIB_OBJS) \
		-o $(MAIN_TARGET) \
		-lwiringPi -lpthread -lpigpiod_if2 -lrt

clean:
	rm -rf $(BUILD_DIR) $(MAIN_TARGET)

run-main: $(MAIN_TARGET)
	./$(MAIN_TARGET)

.PHONY: all clean run-main run-tof