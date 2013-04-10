CC=g++
CFLAGS=-c -Wall
LDFLAGS=
SOURCES=main.cpp WalkStatus.cpp matrixpinv.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=App2

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@


