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

# Libs directories
LIBDIR = -LThirdParty/boost_1_53_0/stage/arm_lib/
LIBDIR += -LThirdParty/opencv/build/arm/

# Boost libs
LIBFLAGS =  -lboost_date_time
LIBFLAGS += -lboost_regex
LIBFLAGS += -lboost_system
LIBFLAGS += -lboost_thread
LIBFLAGS += -lpthread
LIBFLAGS += -lboost_wserialization
LIBFLAGS += -lboost_filesystem

# OpenCV libs
LIBFLAGS += -lopencv_calib3d
LIBFLAGS += -lopencv_contrib
LIBFLAGS += -lopencv_core
LIBFLAGS += -lopencv_features2d
LIBFLAGS += -lopencv_flann
LIBFLAGS += -lopencv_gpu
LIBFLAGS += -lopencv_highgui
LIBFLAGS += -lopencv_imgproc
LIBFLAGS += -lopencv_legacy
LIBFLAGS += -lopencv_ml
LIBFLAGS += -lopencv_nonfree
LIBFLAGS += -lopencv_objdetect
LIBFLAGS += -lopencv_photo
LIBFLAGS += -lopencv_stitching
LIBFLAGS += -lopencv_ts
LIBFLAGS += -lopencv_video
LIBFLAGS += -lopencv_videostab  

# OpenCV dependencies
LIBFLAGS += -lz
LIBFLAGS += -ljpeg
LIBFLAGS += -lpng12
LIBFLAGS += -lgtk-x11-2.0
LIBFLAGS += -lgdk-x11-2.0
LIBFLAGS += -lgobject-2.0
LIBFLAGS += -lglib-2.0
LIBFLAGS += -ldc1394
LIBFLAGS += -lpangocairo-1.0
LIBFLAGS += -lX11
LIBFLAGS += -lXfixes
LIBFLAGS += -latk-1.0
LIBFLAGS += -lcairo
LIBFLAGS += -lgdk_pixbuf-2.0
LIBFLAGS += -lgio-2.0
LIBFLAGS += -lpangoft2-1.0
LIBFLAGS += -lpango-1.0
LIBFLAGS += -lfontconfig
LIBFLAGS += -lXext
LIBFLAGS += -lXrender
LIBFLAGS += -lXinerama
LIBFLAGS += -lXi
LIBFLAGS += -lXrandr
LIBFLAGS += -lXcursor
LIBFLAGS += -lXcomposite
LIBFLAGS += -lXdamage
LIBFLAGS += -lffi
LIBFLAGS += -lpcre
LIBFLAGS += -lraw1394
LIBFLAGS += -lusb-1.0
LIBFLAGS += -lfreetype
LIBFLAGS += -lxcb
LIBFLAGS += -lpixman-1
LIBFLAGS += -lxcb-shm
LIBFLAGS += -lgmodule-2.0
LIBFLAGS += -lselinux
LIBFLAGS += -lexpat
LIBFLAGS += -lxcb-render
LIBFLAGS += -lXau
LIBFLAGS += -lXdmcp

# Additionnal includes directories
INCLUDES = -IThirdParty/boost_1_53_0/
INCLUDES += -IThirdParty/opencv/include/

# Binary
OUT = audit_2_demo

# CFLAGS
CFLAGS= -c -Wall -std=c++0x

CC = arm-linux-gnueabihf-g++

OBJECTS = $(SRC:.cpp=.o)

.PHONY: all clean

all: $(SRC) $(OUT)

$(OUT) : $(OBJECTS)
	$(CC) $(LIBDIR) $(OBJECTS) $(LIBFLAGS)  -o $@

%.o : %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ 

clean:
	rm -rf *.o $(OUT)

