# Sources
SRC =  test1.cpp

SRC += Trajectory.cpp
SRC += Kinematics.cpp
SRC += ZMP.cpp
SRC += EigenUtils.cpp


# Libs directories
LIBDIR = -L/usr/local/lib/

# Binary
OUT = test1 

# CFLAGS
#CFLAGS= -g -c -Wall -std=c++0x -O3
CFLAGS= -g -c -Wall -std=gnu++0x


#CC = arm-linux-gnueabihf-g++
CC = g++

OBJECTS = $(SRC:.cpp=.o)

.PHONY: all clean

all: $(SRC) $(OUT)

$(OUT) : $(OBJECTS)
	$(CC) $(LIBDIR) $(OBJECTS) $(LIBFLAGS)  -o $@

%.o : %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ 

clean:
	rm -rf $(OBJECTS) $(OUT)

