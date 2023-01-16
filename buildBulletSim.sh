#! /bin/bash
# Script to build BulletSim Linux binaries.
# This presumes the bins and includes for Bullet are in LIBDIR and INCLUDEDIR

BASE=$(pwd)

LIBDIR=./lib
INCLUDEDIR=./include

TARGETBASE=libBulletSim

# CC=gcc
# CC=/usr/bin/g++
CC=/usr/bin/c++
# LD=/usr/bin/g++
LD=/usr/bin/c++

UNAME=$(uname)
MACH=$(uname -m)

# Version of the Bullet engine that is being statically linked
BULLETVERSION=$(cat "${INCLUDEDIR}/VERSION")
# Version of the BulletSim glue that is being built and included
BULLETSIMVERSION=$(cat "VERSION")

BUILDDATE=$(date "+%Y%m%d")

# Kludge for building libBulletSim.so with different library dependencies
#    As of 20130424, 64bit Ubuntu needs to wrap memcpy so it doesn't pull in glibc 2.14.
#    The wrap is not needed on Ubuntu 32bit and, in fact, causes crashes.
if [[ "$MACH" == "x86_64" ]] ; then
    WRAPMEMCPY=-Wl,--wrap=memcpy
else
    WRAPMEMCPY=
fi

case $UNAME in
    "Linux")
        TARGET=${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-${MACH}.so
        CFLAGS="-I${INCLUDEDIR} -fPIC -g -fpermissive"
        LFLAGS="${WRAPMEMCPY} -shared -Wl,-soname,${TARGET} -o ${TARGET}"
        ;;
    "Darwin")
        TARGET=${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-universal.dylib
        CFLAGS="-arch arm64 -arch x86_64 -O2 -I${INCLUDEDIR} -g"
        LFLAGS="-v -dynamiclib -arch arm64 -arch x86_64 -o ${TARGET}"
        ;;
    *)
        TARGET=${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-${MACH}.so
        CFLAGS="-I${IDIR} -fPIC -g -fpermissive"
        LFLAGS="${WRAPMEMCPY} -shared -Wl,-soname,${TARGET} -o ${TARGET}"
        ;;
esac

# All of the Bullet bin files
BULLETLIBS=$(ls ${LIBDIR}/*.a)

# Just build everything
echo "=== Building target $TARGET from BulletSim ${BULLETSIMVERSION} and Bullet ${BULLETVERSION}"
${CC} ${CFLAGS} -c API2.cpp
${CC} ${CFLAGS} -c BulletSim.cpp
${LD} ${LFLAGS} API2.o BulletSim.o ${BULLETLIBS}

