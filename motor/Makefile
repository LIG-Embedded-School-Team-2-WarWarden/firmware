CXX ?= g++
CPPFLAGS += -D_DEFAULT_SOURCE
CXXFLAGS ?= -O2 -Wall -Wextra
CXXFLAGS += -std=c++11
OBJECTS = main.o motor.o laser.o ethernet.o

all: motor-demo

motor-demo: $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(OBJECTS:.o=.d)

clean:
	rm -f motor-demo $(OBJECTS) $(OBJECTS:.o=.d)

.PHONY: all clean
