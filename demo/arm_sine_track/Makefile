CXX      ?= g++
CXXFLAGS  = -O2 -std=c++17 -Wall -Wextra -pthread
TARGET    = arm_sine_track
SRC       = arm_sine_track.cpp

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
