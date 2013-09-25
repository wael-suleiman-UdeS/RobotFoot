CC=g++
CFLAGS=-I/usr/local/include/opencv2
LIBS=$(shell pkg-config --libs opencv)

all : main.o test.o Settings.o
    $(CC) $(CFLAGS) main.o test.o Settings.o -o testCamera $(LIBS)
main.o : main.cpp test.h
    $(CC) $(CFLAGS) -c main.cpp -o main.o $(LIBS)
test.o test.cpp Settings.h
    $(CC) $(CFLAGS) -c test.cpp -o test.o $(LIBS)
Settings.o : Settings.cpp
    $(CC) $(CFLAGS) -c Settings.cpp -o Settings.o $(LIBS)
clean :
    rm -f *.o
