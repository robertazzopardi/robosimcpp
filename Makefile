#
# 'make check'        	run cppcheck
# 'make analyse'		run scan-build
#

# define the C compiler to use
# CC = g++
CC = clang++

# define any compile-time flags
CFLAGS	:= -Wall -Wextra -flto -g -O3 -D_THREAD_SAFE `sdl2-config --cflags`
CFLAGS  += -fsanitize=address -fno-omit-frame-pointer

# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
LFLAGS = -lSDL2_image `sdl2-config --libs`

# scan-build
SCAN =		scan-build

# scan flags
SCANFLAGS =	-v -analyze-headers -no-failure-reports -enable-checker deadcode.DeadStores --status-bugs

# define source directory
SRC		:= src/robosim

# define include directory
INCLUDE	:= include/robosim

ifeq ($(OS),Windows_NT)
SOURCEDIRS	:= $(SRC)
INCLUDEDIRS	:= $(INCLUDE)
RM			:= del /q /f
else
SOURCEDIRS	:= $(shell find $(SRC) -type d)
INCLUDEDIRS	:= $(shell find $(INCLUDE) -type d)
RM = rm -f
endif

# define any directories containing header files other than /usr/include
INCLUDES	:= $(patsubst %,-I%, $(INCLUDEDIRS:%/=%))

# define the C source files
SOURCES		:= $(wildcard $(patsubst %,%/*.cpp, $(SOURCEDIRS)))

#
# The following part of the makefile is generic; it can be used to
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

check:
	cppcheck -f --enable=all --inconclusive --check-library --debug-warnings --suppress=missingIncludeSystem --check-config $(INCLUDES) ./$(SRC)

analyse:
# scan-build make
	clear
	for file in $(SOURCES) ; do \
    	$(SCAN) $(SCANFLAGS) $(CC) $(CFLAGS) --analyze -fsanitize=address $(INCLUDES) $$file ; \
	done
	$(RM) *.plist

bloaty:
	bloaty build/librobosim.dylib
