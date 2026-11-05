# Compiles the project assuming it adheres to the following structure:
#
# <root directory>
# |
# +---- <SRC_DIR> ---+---- subdir1/*.cpp, *.hpp
# |                  +---- subdir2/*.cpp, *.hpp                       
# |                  +---- demo00.cpp, demo01.cpp, ...
#
# Build a selected demo (src/demoXX.cpp) as:
#
# make                 # builds default demo (demo00)
# make DEMO=demoXX     # same as source file without the .cpp
# make run DEMO=demoXX # build and run

SHELL    := /bin/bash
CXX      := g++

SRC_DIR  := src
BIN_DIR  := build
OBJ_DIR  := build/obj

# include all directories that contain headers under src/ (plus src itself)
rwildcard = $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(wildcard $d/$2))
INCDIRS  := $(sort $(SRC_DIR) $(dir $(call rwildcard,$(SRC_DIR)/,*.hpp) $(call rwildcard,$(SRC_DIR)/,*.h)))
INCLUDES := $(patsubst %,-I%,$(INCDIRS))

# DEFINES options are used to enable options such as compiling with
# textures if textures have been fetched, e.g.:
# make DEFINES=-DUSE_TEXTURES <target>
DEFINES ?=

CXXFLAGS := $(INCLUDES) -O3 -std=c++17 -Wall -Wextra -MMD -MP\
            $(DEFINES) -Wno-unused-parameter -Wno-unused-function\
			-fopenmp
LDFLAGS  :=
LDLIBS   :=

# demo cmd line variable (DEMO=demoXX) default value
DEMO ?= demo00

IMPL_SRCS := $(call rwildcard,$(SRC_DIR)/,*.cpp)
ALL_SRCS  := $(filter-out $(SRC_DIR)/demo%.cpp ,$(IMPL_SRCS))
ALL_OBJS  := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/$(SRC_DIR)/%.o,$(ALL_SRCS))

# demo bin for src/demoXX.cpp ->  $(BIN_DIR)/demoXX
DEMO_SRC := $(SRC_DIR)/$(DEMO).cpp
DEMO_BIN := $(BIN_DIR)/$(DEMO)

# dependency files: if a header changes, rebuild the object files
DEPS := $(ALL_OBJS:.o=.d)

.PHONY: all run clean rebuild convert

# build selected demo
all: $(DEMO_BIN)
	@echo -e "\n======== Built demo at: $(DEMO_BIN) ========"

# link the selected demo with implementation objects
$(DEMO_BIN): $(DEMO_SRC) $(ALL_OBJS)
	@mkdir -p $(BIN_DIR)
	@echo -e "\n======== Linking $@ ========"
	$(CXX) $(CXXFLAGS) $(ALL_OBJS) $< -o $@ $(LDFLAGS) $(LDLIBS)

# compile all objects
$(OBJ_DIR)/$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo -e "\n======== Compiling $< -> $@ ========"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# dependency info for implementation objects (if headers change)
-include $(DEPS)

run: $(DEMO_BIN)
	$(DEMO_BIN) output_$(DEMO).ppm

# redundant target for convenience; build with USE_TEXTURES
.PHONY: 
use-textures:
	@echo -e "\n======== Building with USE_TEXTURES ========"
	@$(MAKE) --no-print-directory DEMO=$(DEMO) DEFINES="$(DEFINES) -DUSE_TEXTURES"

convert:
	@echo -e "\n======== Converting textures/backgrounds to PPM ========"
	@bash scripts/textures2ppm.sh

clean:
	@echo -e "\n======== Cleaning build artifacts ========"
	@rm -rf build

rebuild: clean all
