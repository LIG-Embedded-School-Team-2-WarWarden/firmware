CXX = arm-linux-gnueabihf-g++
HOST_CXX ?= g++
CPPFLAGS += -D_DEFAULT_SOURCE
CXXFLAGS ?= -O2 -Wall -Wextra
CXXFLAGS += -std=c++11 -pthread
OBJECTS = main.o motor.o laser.o ethernet.o

all: motor-demo

motor-demo: $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(OBJECTS:.o=.d)

controller-test: controller_test.cpp controller.h protocol.h control_fsm.h raster_path.h
	$(HOST_CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ controller_test.cpp

check:
	$(HOST_CXX) $(CPPFLAGS) $(CXXFLAGS) -fsyntax-only main.cpp motor.cpp laser.cpp ethernet.cpp

test: check controller-test
	./controller-test

clean:
	rm -f motor-demo controller-test $(OBJECTS) $(OBJECTS:.o=.d)

.PHONY: all check test clean
