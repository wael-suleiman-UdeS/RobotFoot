# Sources
SRC =  test1.cpp
OFFSET = ../../

#SRC += ../Utilities/logger.cpp
SRC += Trajectory.cpp
SRC += DenavitHartenberg.cpp
SRC += MotionControl.cpp
SRC += EigenUtils.cpp
#SRC += LegMotion.cpp

# Libs directories
LIBDIR = -L/usr/local/lib/
LIBDIR += -L$(OFFSET)ThirdParty/boost_1_54_0/stage/ubuntu_x86_lib/

## Boost libs
LIBFLAGS =  -lboost_date_time
LIBFLAGS += -lboost_regex
LIBFLAGS += -lboost_system
LIBFLAGS += -lboost_thread
LIBFLAGS += -lpthread
LIBFLAGS += -lboost_wserialization
LIBFLAGS += -lboost_filesystem
LIBFLAGS += -lboost_timer
LIBFLAGS += -lboost_chrono

# Additionnal includes directories
INCLUDES = -I$(OFFSET)ThirdParty/boost_1_54_0/

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

