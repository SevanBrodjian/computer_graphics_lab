###############################################################################
# CS/CNS 171 Fall 2015 -- **MODIFIED FOR MAC**
#
# This is a template Makefile for OpenGL programs. Edit it however you find
# convenient.
# 
# The current version of this file should compile OpenGL programs just fine on
# Debian-based Linux operating systems.
#
# If you run Mac OS or other distributions of Linux, then you may have to
# fool around with the 'INCLUDE' and 'LIBDIR' lines below before the Makefile
# can compile the OpenGL parts successfully.
###############################################################################
CC     = g++
CFLAGS = -g -std=c++14

SOURCES = $(wildcard *.cpp)
EXENAME = double_pendulum

UNAME_S := $(shell uname -s)

INCLUDE :=
LIBDIR  :=
LIBS    :=

ifeq ($(UNAME_S),Darwin)  # macOS
    BREW_PREFIX := $(shell brew --prefix)

    INCLUDE += -I$(BREW_PREFIX)/include
    LIBDIR  += -L$(BREW_PREFIX)/lib

    LIBS += -lGLEW -framework OpenGL -framework GLUT
else  # assume Linux, use original makefile
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