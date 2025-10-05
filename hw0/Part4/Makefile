###############################################################################
# CS/CNS 171 — HW0 Part 4
###############################################################################
CC = g++
FLAGS = -g
SOURCES = $(wildcard *.cpp)

EXENAME = ppm_test

all: $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(SOURCES)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean

