CXX ?= g++

# C++11 because that is what the VEX toolchain compiles user programs as.
# Building the host-side binaries under the same standard keeps a feature that
# will not survive the port from quietly reaching the robot.
CXXFLAGS := -std=c++11 -O2 -Wall -Wextra -Iinclude

HEADERS := $(wildcard include/*.h)

.PHONY: all masks clean
all: build/masks_parity build/bridge

build/masks_parity: test/masks_parity.cpp $(HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

# The algorithm, driven by the TypeScript simulator over a pipe. Links exactly
# the translation unit the robot gets; only the IRobot implementation differs.
build/bridge: sim/bridge.cpp src/program.cpp $(HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ sim/bridge.cpp src/program.cpp

# Regenerate include/masks.h from the simulator's footprint geometry. Kept off
# the default build on purpose: the generated header is committed so the robot
# can be built by a toolchain with no bun in it, and a checkout that reorders
# file timestamps must not be able to trigger a rebuild that then fails for
# want of a TypeScript runtime. test/masks.test.ts is what catches staleness.
masks:
	bun tools/gen_tables.ts

clean:
	rm -rf build
