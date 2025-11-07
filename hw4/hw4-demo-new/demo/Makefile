UNAME_S := $(shell uname -s)

CC       := clang++
CXXFLAGS := -g -DGL_SILENCE_DEPRECATION
BREW_PREFIX := $(shell brew --prefix 2>/dev/null)
PNG_PREFIX  := $(shell brew --prefix libpng 2>/dev/null)

SOURCES := $(wildcard *.cpp)
EXENAME := glslRenderer

ifeq ($(UNAME_S),Darwin)  # macOS
    INCLUDE := $(if $(PNG_PREFIX),-I$(PNG_PREFIX)/include)
    LIBDIR  := $(if $(PNG_PREFIX),-L$(PNG_PREFIX)/lib)

    # OpenGL & GLUT are frameworks on macOS; no -lGL/-lGLU
    LIBS := -lpng -framework OpenGL -framework GLUT

    # Help the compiler find SDK headers if needed
    SDKROOT := $(shell xcrun --sdk macosx --show-sdk-path)
    CXXFLAGS += -isysroot $(SDKROOT)
else                      # Linux
    CC := g++
    INCLUDE := -I/usr/X11R6/include -I/usr/include/GL -I/usr/include
    LIBDIR  := -L/usr/X11R6/lib -L/usr/local/lib
    LIBS := -lGL -lGLU -lglut -lm -lpng
endif

$(EXENAME): $(SOURCES)
	$(CC) $(CXXFLAGS) -o $@ $(INCLUDE) $(LIBDIR) $(SOURCES) $(LIBS)

.PHONY: run clean all
run:
	./$(EXENAME)

clean:
	rm -f *.o $(EXENAME) core glslRenderer

all: clean $(EXENAME)