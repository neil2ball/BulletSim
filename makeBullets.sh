#! /bin/bash
# Script to fetch the Bullet Physics Engine sources, build same
#    using the 'buildBulletSMake.sh' script, and then build BulletSim
#    .so's using 'Makefile'.
# This captures the steps needed and will be replaced by better scripts
#    and Github actions.
#
# This builds two version of Bullet: one of current version and another
#    if Bullet version 2.86 which is the version of Bullet that was
#    used in the BulletSim binaries distributed with OpenSimulator
#    from 2015 to 2022.
# This also applies the BulletSim patches to the Bullet sources.

# Set these values to 'yes' or 'no' to enable/disable fetching and building
FETCHBULLETSOURCES=yes
BUILDBULLET2=yes
BUILDBULLET3=yes

BASE=$(pwd)

if [[ "$FETCHBULLETSOURCES" == "yes" ]] ; then
    cd "$BASE"
    echo "=== Fetching Bullet Physics Engine sources into bullet3/"
    git clone https://github.com/bulletphysics/bullet3.git

    cd "$BASE"
    echo "=== Creating bullet2/ of Bullet version 2.86"
    cp -r bullet3 bullet2
    cd bullet2
    git checkout tags/2.86 -b tag-2.86

    echo "=== Applying BulletSim patches to bullet3"
    cd "$BASE"
    cd bullet3
    for file in ../000* ; do cat $file | patch -p1 ; done

    echo "=== Applying BulletSim patches to bullet3"
    cd "$BASE"
    cd bullet2
    for file in ../2...-00* ; do cat $file | patch -p1 ; done
fi

cd "$BASE"

echo "=== removing libBulletSim-*"
make cleanlibs

if [[ "$BUILDBULLET2" == "yes" ]] ; then
    echo "=== building bullet2"
    BULLETDIR=bullet2 ./buildBulletCMake.sh
    make
fi

if [[ "$BUILDBULLET3" == "yes" ]] ; then
    echo "=== building bullet3"
    BULLETDIR=bullet3 ./buildBulletCMake.sh
    make
fi
