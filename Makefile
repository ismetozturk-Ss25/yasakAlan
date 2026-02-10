# Forbidden-Area Avoidance Command Shaper — Build System
#
# Usage:
#   make            Build test runner (with debug print)
#   make run        Build and run tests
#   make clean      Remove build artifacts

CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -pedantic -std=c99 -Isrc -DAVD_DEBUG_PRINT
LDFLAGS =

SRC_DIR  = src
TEST_DIR = test
BUILD    = build

# Source files (split architecture: preop + opmode)
CORE_SRC = $(SRC_DIR)/avoidance_preop.c $(SRC_DIR)/avoidance_opmode.c
TEST_SRC = $(TEST_DIR)/runner_main.c

# Output
TEST_BIN = $(BUILD)/test_runner

.PHONY: all run clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(TEST_BIN): $(CORE_SRC) $(TEST_SRC) $(SRC_DIR)/avoidance_simulink.h | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) $(CORE_SRC) $(TEST_SRC) -o $(TEST_BIN) $(LDFLAGS)

run: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -rf $(BUILD)
