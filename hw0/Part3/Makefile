###############################################################################
# CS/CNS 171 — HW0 Part 3
###############################################################################
CXX       := g++
CXXFLAGS  := -O2 -g -std=c++17 -Wall -Wextra -Wno-unused-parameter

EIGEN_DIR := ../
CPPFLAGS  := -isystem $(EIGEN_DIR)

SOURCES   := $(wildcard *.cpp)
EXENAME   := load_transformed_objects

all: $(EXENAME)

$(EXENAME): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $(SOURCES)

clean:
	rm -f $(EXENAME)

print-eigen:
	@echo "Using Eigen from: $(EIGEN_DIR)"

.PHONY: all clean print-eigen

