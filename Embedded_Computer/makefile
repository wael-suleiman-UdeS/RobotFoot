# Project Path
SRC_PATH =  src/
BUILD_PATH =  bin/

# Sources
SRC += RobotFoot.cpp
SRC += Utilities/SerialInterface.cpp
SRC += Utilities/logger.cpp
SRC += Utilities/XmlParser.cpp
SRC += Utilities/pugixml.cpp
SRC += Utilities/PID.cpp
SRC += Utilities/ThreadManager.cpp
SRC += ImageProcessing/Camera.cpp
SRC += ImageProcessing/ObjectTracker.cpp
SRC += ImageProcessing/ColorFinder.cpp
SRC += ImageProcessing/ImageProcessing.cpp
SRC += ImageProcessing/HeadControlTask.cpp
SRC += Control/STM32F4_2.cpp
SRC += Control/MotorControl_2.cpp
SRC += Control/Protocol.cpp
SRC += Control/Motor.cpp
SRC += Demo/StaticWalking/StaticWalk.cpp
SRC += Demo/StaticWalking/WalkStatus.cpp
SRC += Demo/StaticWalking/matrixpinv.cpp
SRC += LegMotion/Trajectory.cpp
SRC += LegMotion/DenavitHartenberg.cpp
SRC += LegMotion/MotionControl.cpp
SRC += LegMotion/EigenUtils.cpp
SRC += LegMotion/LegMotion.cpp


# Libs directories
LIBDIR_ARM =  -LThirdParty/boost_1_54_0/stage/arm_lib/
LIBDIR_ARM += -LThirdParty/opencv/build/arm/

LIBDIR_X86 =  -LThirdParty/boost_1_54_0/stage/ubuntu_x86_lib/
LIBDIR_X86 += -LThirdParty/opencv/build/x86/ubuntu/
LIBDIR_X86 += -LThirdParty/opencv/build/x86/ubuntu/dependencies/

LIBDIR_X64 =  -LThirdParty/boost_1_54_0/stage/ubuntu_x64_lib/
LIBDIR_X64 += -LThirdParty/opencv/build/x64/ubuntu/
LIBDIR_X64 += -LThirdParty/opencv/build/x64/ubuntu/dependencies/

# Boost libs
LIBFLAGS =  -lboost_date_time
LIBFLAGS += -lboost_regex
LIBFLAGS += -lboost_system
LIBFLAGS += -lboost_thread
LIBFLAGS += -lpthread
LIBFLAGS += -lboost_wserialization
LIBFLAGS += -lboost_filesystem
LIBFLAGS += -lboost_timer
LIBFLAGS += -lboost_chrono

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
INCLUDES = -IThirdParty/boost_1_54_0/
INCLUDES += -IThirdParty/opencv/include/
INCLUDES += -Isrc/

# Architectures
ARM=arm
X86=x86
X64=x64

# Binary out
DEBUG_PREFIX = debug
RELEASE_PREFIX = release
OUT_FILE = RobotFoot

# CFLAGS
CFLAGS = -c -Wall -std=gnu++0x
DEBUG_CFLAG = -g3
RELEASE_CFLAG = -O3

# Compilers
ARM_CC = arm-linux-gnueabihf-g++
INTEL_CC = g++

.PHONY: armd arm x86d x86 x64d x64
ifeq ($(MAKECMDGOALS), arm)	
    BUILD_PATH := $(BUILD_PATH)$(ARM)/$(RELEASE_PREFIX)/
    CFLAGS += $(RELEASE_CFLAG)
    CC = $(ARM_CC)
    LIBDIR := $(LIBDIR_ARM)
else ifeq ($(MAKECMDGOALS), armd)
    BUILD_PATH := $(BUILD_PATH)$(ARM)/$(DEBUG_PREFIX)/
    CFLAGS += $(DEBUG_CFLAG)
    CC = $(ARM_CC)
    LIBDIR := $(LIBDIR_ARM)
else ifeq ($(MAKECMDGOALS), x64d)
    BUILD_PATH := $(BUILD_PATH)$(X64)/$(DEBUG_PREFIX)/
    CFLAGS += $(DEBUG_CFLAG)
    CC = $(INTEL_CC)
    LIBDIR := $(LIBDIR_X64)
else ifeq ($(MAKECMDGOALS), x86d)
    BUILD_PATH := $(BUILD_PATH)$(X86)/$(DEBUG_PREFIX)/
    CFLAGS += $(DEBUG_CFLAG)
    CC = $(INTEL_CC)
    LIBDIR := $(LIBDIR_X86)
else ifeq ($(MAKECMDGOALS), x64)
    BUILD_PATH := $(BUILD_PATH)$(X64)/$(RELEASE_PREFIX)/
    CFLAGS += $(RELEASE_CFLAG)
    CC = $(INTEL_CC)
    LIBDIR := $(LIBDIR_X64)
else
    BUILD_PATH := $(BUILD_PATH)$(X86)/$(RELEASE_PREFIX)/
    CFLAGS += $(RELEASE_CFLAG)
    CC = $(INTEL_CC)
    LIBDIR := $(LIBDIR_X86)
endif

# Add prefix to build variables
OBJECTS  = $(addprefix $(BUILD_PATH), $(SRC:.cpp=.o))
SRC_     = $(addprefix $(SRC_PATH), $(SRC))
OUT      = $(addprefix $(BUILD_PATH), $(OUT_FILE))
OBJECTS_PATH = $(dir $(OBJECTS))

arm armd x86 x86d x64 x64d all: $(SRC_) $(OUT)

# Link
$(OUT) : $(OBJECTS)
	@echo "Linking..."
	$(CC) $(LIBDIR) $(OBJECTS) $(LIBFLAGS) -o $@
	@echo "$(BUILD_PATH)$(OUT_FILE)" > ./run
	@chmod 777 run

# Compile
$(BUILD_PATH)%.o : $(SRC_PATH)%.cpp
	@mkdir -p $(OBJECTS_PATH) 
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.PHONY: install
# Install shared libs in /usr/local/lib
INSTALL_PATH = /usr/local/lib/
ifeq (i686 ,$(findstring i686, $(shell arch)))
    $(info i686 architecture detected.)
    BOOST_SHARED  = ThirdParty/boost_1_54_0/stage/ubuntu_x86_lib/libboost_*
    OPENCV_SHARED = ThirdParty/opencv/build/x86/ubuntu/libopencv_*
else ifeq (arm ,$(findstring arm, $(shell arch)))
    $(info Arm architecture detected.)
    BOOST_SHARED  = ThirdParty/boost_1_54_0/stage/arm_lib/libboost_*
    OPENCV_SHARED = ThirdParty/opencv/build/arm/*
else ifeq (x86_64 ,$(findstring x86_64, $(shell arch)))
    $(info x64 architecture detected.)
    BOOST_SHARED  = ThirdParty/boost_1_54_0/stage/ubuntu_x64_lib/libboost_*
    OPENCV_SHARED = ThirdParty/opencv/build/x64/ubuntu/libopencv_*
else
    $(info Unknow architecture.)
endif

install:
	if [[ -z $(BOOST_SHARED)] && [ -z $(OPENCV_SHARED)]]; then \
	    echo "Could not install shared libs"; \
	else \
	    rsync -av $(BOOST_SHARED) $(INSTALL_PATH); \
	    rsync -av $(OPENCV_SHARED) $(INSTALL_PATH); \
	    echo "/usr/local/lib" > /etc/ld.so.conf.d/robotfoot.conf; \
	    ldconfig; \
	fi

.PHONY: clean debug
# Clear all files of any builds
# TODO Make clean for each build type
clean:
	@echo "Cleanning project..."
	@find bin/ -name "*.o" -exec rm -f '{}' +
	@find bin/ -name $(OUT_FILE) -exec rm -f '{}' +
	@echo "done"

debug:
	@echo $(OBJECTS)
