# Build BulletSim
#
# The Bullet physics engine has been previously built and is in the directories IDIR and LDIR

TARGETBASE=libBulletSim

IDIR = ./include
LDIR = ./lib

# the Bullet libraries are linked statically so we don't have to also distribute the shared binaries
BULLETLIBS := $(wildcard $(LDIR)/*.a)
 
#CC = gcc
#CC = /usr/bin/g++
#LD = /usr/bin/g++
CC = /usr/bin/c++
LD = /usr/bin/c++
export UNAME := $(shell uname)
export UNAMEPROCESSOR := $(shell uname -m)

# Version of the bullet engine being statically linked
export BULLETVERSION := $(shell cat $(IDIR)/VERSION)
# Version of BulletSim being built and included
export BULLETSIMVERSION := $(shell cat VERSION)

export BUILDDATE := $(shell date "+%Y%m%d")

# Kludge for building libBulletSim.so with different library dependencies
#    As of 20130424, 64bit Ubuntu needs to wrap memcpy so it doesn't pull in glibc 2.14.
#    The wrap is not needed on Ubuntu 32bit and, in fact, causes crashes.
ifeq ($(UNAMEPROCESSOR), x86_64)
WRAPMEMCPY = -Wl,--wrap=memcpy
else
WRAPMEMCPY =
endif

# Linux build.
ifeq ($(UNAME), Linux)
TARGET = $(TARGETBASE)-$(BULLETSIMVERSION)-$(BULLETVERSION)-$(BUILDDATE)-$(UNAMEPROCESSOR).so
CFLAGS = -I$(IDIR) -fPIC -g -fpermissive
LFLAGS = $(WRAPMEMCPY) -shared -Wl,-soname,$(TARGET) -o $(TARGET)
endif

# OSX build. Builds 32bit dylib on 64bit system. (Need 32bit because Mono is 32bit only).
ifeq ($(UNAME), Darwin)
TARGET = $(TARGETBASE)-$(BULLETSIMVERSION)-$(BULLETVERSION)-$(BUILDDATE)-universal.dylib
# CC = clang
# LD = clang
CFLAGS = -arch arm64 -arch x86_64 -O2 -I$(IDIR) -g 
LFLAGS = -v -dynamiclib -arch arm64 -arch x86_64 -o $(TARGET)
endif

BASEFILES = API2.cpp BulletSim.cpp

SRC = $(BASEFILES)
# SRC = $(wildcard *.cpp)

# This presumes that all .o's in the current directory are the pieces for BulletSim
BIN = $(patsubst %.cpp, %.o, $(SRC))

all: $(TARGET)

# When building target, we always rebuild the .o files (make sure they match the lib/include dirs)
$(TARGET) : clean $(BIN)
	$(LD) $(LFLAGS) $(BIN) $(BULLETLIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $?

BulletSim.cpp : BulletSim.h Util.h

BulletSim.h: ArchStuff.h APIData.h WorldData.h

API2.cpp : BulletSim.h

clean:
	rm -f *.o

cleanlibs:
	rm -f $(wildcard $(TARGETBASE)*)
