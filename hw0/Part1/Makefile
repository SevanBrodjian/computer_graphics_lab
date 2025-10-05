###############################################################################
# CS/CNS 171 — HW0 Part 1
###############################################################################
CC = g++
FLAGS = -g
SOURCES = $(wildcard *.cpp)

EXENAME = read_obj

all: $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(SOURCES)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean

