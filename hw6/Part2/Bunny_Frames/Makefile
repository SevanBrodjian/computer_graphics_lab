CC = g++
CFLAGS = -g -std=c++14

SOURCES = $(wildcard *.cpp)
EXENAME = bunny_interpolate

INCLUDE := -I../..

all: $(EXENAME)

$(EXENAME): $(SOURCES)
	$(CC) $(CFLAGS) -o $(EXENAME) $(SOURCES) $(INCLUDE)

clean:
	rm -f $(EXENAME) *.o

.PHONY: all clean
