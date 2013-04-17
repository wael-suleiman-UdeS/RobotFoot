# Sources
SRC =  RobotFoot.cpp
SRC += Utilities/SerialInterface.cpp
SRC += Utilities/logger.cpp
SRC += Utilities/XmlParser.cpp
SRC += Utilities/pugixml.cpp
SRC += ImageProcessing/Camera.cpp
SRC += ImageProcessing/ObjectTracker.cpp
SRC += ImageProcessing/ColorFinder.cpp
SRC += ImageProcessing/ImageProcessing.cpp
SRC += Control/STM32F4.cpp

# Librairies
LIBDIR = -L/opt/local/lib
LIBDIR += -L/usr/local/lib

LIBFLAGS =  -lboost_date_time
LIBFLAGS += -lboost_regex
LIBFLAGS += -lboost_system
LIBFLAGS += -lboost_thread
LIBFLAGS += -lpthread
LIBFLAGS += -lboost_wserialization
LIBFLAGS += -lboost_filesystem
LIBFLAGS += $(shell pkg-config --libs opencv)
 
# Includes
INCLUDE = /opt/local/include
OUT = audit_2_demo

CC = arm-linux-gnueabi-gcc

all: $(OUT)

$(OUT) : $(SRC)
	$(CC) -std=c++0x -I$(INCLUDE) $(SRC) -o$(OUT) $(LIBDIR) $(LIBFLAGS)
clean:
	rm -f $(OUT)

