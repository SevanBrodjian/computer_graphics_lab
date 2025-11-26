###############################################################################
# CS/CNS 171 Fall 2015 -- **MODIFIED FOR MAC**
# Template Makefile for OpenGL programs.
###############################################################################
CC     = g++
CFLAGS = -g -std=c++14

SOURCES = $(wildcard *.cpp)
EXENAME = keyframe

UNAME_S := $(shell uname -s)

INCLUDE := -I../..
LIBDIR  :=
LIBS    :=

ifeq ($(UNAME_S),Darwin)  # macOS
    BREW_PREFIX := $(shell brew --prefix)

    INCLUDE += -I$(BREW_PREFIX)/include
    LIBDIR  += -L$(BREW_PREFIX)/lib

    LIBS += -lGLEW -framework OpenGL -framework GLUT
else  # assume Linux
    INCLUDE += -I/usr/X11R6/include -I/usr/include/GL -I/usr/include
    LIBDIR  += -L/usr/X11R6/lib -L/usr/local/lib

    LIBS += -lGLEW -lGL -lGLU -lglut -lm
endif

all: $(EXENAME)

$(EXENAME): $(SOURCES)
	$(CC) $(CFLAGS) -o $(EXENAME) $(SOURCES) $(INCLUDE) $(LIBDIR) $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
