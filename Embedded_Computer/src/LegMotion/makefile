# Sources
SRC =  test1.cpp

#SRC += ../Utilities/logger.cpp
SRC += Trajectory.cpp
SRC += DenavitHartenberg.cpp
SRC += MotionControl.cpp
SRC += EigenUtils.cpp
#SRC += LegMotion.cpp

# Libs directories
LIBDIR = -L/usr/local/lib/

## Boost libs
#LIBFLAGS =  -lboost_date_time
#LIBFLAGS += -lboost_regex
#LIBFLAGS += -lboost_system
#LIBFLAGS += -lboost_thread
#LIBFLAGS += -lpthread
#LIBFLAGS += -lboost_wserialization
#LIBFLAGS += -lboost_filesystem
#LIBFLAGS += -lboost_timer
#LIBFLAGS += -lboost_chrono


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

