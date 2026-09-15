CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -fPIC -Wall -Wextra -pedantic
ROOT := ../../..

all: librover_logic.so

librover_logic.so: $(ROOT)/rover_control/rover_logic_core.cpp $(ROOT)/rover_control/rover_logic_c_api.cpp
	$(CXX) $(CXXFLAGS) -shared -o $@ $^

clean:
	rm -f librover_logic.so
