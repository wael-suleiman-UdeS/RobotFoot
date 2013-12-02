CC       = g++-4.7
OUTPUT   = test
SRC      = main.cpp FIFOalloc.cpp
CXXFLAGS = -std=gnu++11 -O3 -Wall
OBJS     = $(SRC:.cpp=.o)

$(OUTPUT) : $(OBJS)
	$(CC) -o $(OUTPUT) $(OBJS)

%.o : %.cpp
	$(CC) -c $< -o $@ $(CXXFLAGS)

main.o : FIFOalloc.hpp main.cpp
FIFOalloc.o : FIFOalloc.hpp
