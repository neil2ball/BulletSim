#! /bin/bash
# Script to fetch the Bullet Physics Engine sources, build same
#    using the 'buildBulletSMake.sh' script, and then build BulletSim
#    .so's using 'buildBulletSim.sh'.
# This captures the steps needed and will be replaced by better scripts
#    and Github actions.
#
# This can build two versions of Bullet: one of current version and another
#    of Bullet version 2.86 which is the version of Bullet that was
#    used in the BulletSim binaries distributed with OpenSimulator
#    from 2015 to 2022.
# This also applies the BulletSim patches to the Bullet sources.

# Set these values to 'yes' or 'no' to enable/disable fetching and building
FETCHBULLETSOURCES=${FETCHBULLETSOURCES:-yes}
BUILDBULLET3=${BUILDBULLET3:-yes}
USEOPENCL=${USEOPENCL:-yes}

# Note that Bullet3 sources are build in "bullet3/" and the
#     these are copied into "bullet2/" and checkouted to the version 2 sources.

BASE=$(pwd)

# Check for OpenCL SDK installation if OpenCL is enabled
if [ "$USEOPENCL" = "yes" ]; then
    echo "=== Checking for OpenCL SDK installation"

    # Check if OpenCL SDK is available
    openCLInstalled=false

    # Check environment variables first
    if [ -n "$OPENCL_SDK_ROOT" ] && [ -d "$OPENCL_SDK_ROOT" ]; then
        openCLInstalled=true
        echo "OpenCL SDK found via environment variable: $OPENCL_SDK_ROOT"
    # Check default installation path
    elif [ -d "$HOME/OpenCL-SDK" ]; then
        openCLInstalled=true
        echo "OpenCL SDK found at default location: ~/OpenCL-SDK"
#     # Check for headers and libraries in common locations
#     elif pkg-config --exists OpenCL; then
#         openCLInstalled=true
#         echo "OpenCL found via pkg-config"
#     elif [ -f "/usr/include/CL/cl.h" ] || [ -f "/usr/local/include/CL/cl.h" ]; then
#         openCLInstalled=true
#         echo "OpenCL found in system include paths"
#     fi
    fi

    if [ "$openCLInstalled" = false ]; then
        echo "OpenCL SDK not found. Installing Khronos vendor-neutral OpenCL SDK..."

        # Check if install script exists
        installScript="installOpenCL-SDK.sh"
        if [ -f "$installScript" ]; then
            # Install to a local directory to avoid needing sudo
            ./installOpenCL-SDK.sh --force
            # Source the environment variables for current session
            export OPENCL_SDK_ROOT="$HOME/OpenCL-SDK"
            export OpenCL_INCLUDE_DIR="$OPENCL_SDK_ROOT/include"
            export OpenCL_LIBRARY_DIR="$OPENCL_SDK_ROOT/lib"
            export PATH="$OPENCL_SDK_ROOT/bin:$PATH"
            export LD_LIBRARY_PATH="$OPENCL_SDK_ROOT/lib:$LD_LIBRARY_PATH"
            echo "OpenCL SDK installation completed."
        else
            echo "WARNING: OpenCL SDK install script '$installScript' not found. Continuing without OpenCL support."
            USEOPENCL="no"
        fi
    else
        echo "OpenCL SDK is available for build"
    fi
fi

if [ "$FETCHBULLETSOURCES" = "yes" ]; then
    cd "$BASE"
    rm -rf bullet3

    echo "=== Fetching Bullet Physics Engine sources into bullet3/"
    git clone https://github.com/bulletphysics/bullet3.git

    echo "=== Applying BulletSim patches to bullet3"
    cd "$BASE"
    cd bullet3
    for file in ../000*; do
        filename=$(basename "$file")
        echo "Processing patch: $filename"
		if [ "$filename" = "0002-opencl-bullet3-onlyfloats-windows-openml.patch" ]; then
			continue
		fi

                if ! git apply --ignore-whitespace "$file"; then
                    echo "ERROR: Failed to apply OpenCL patch: $filename"
                    exit 1
                fi
    done
fi

cd "$BASE"

echo "=== Setting environment variables"
export BuildDate=$(date +%Y%m%d)
export BulletSimVersion=$(cat VERSION)
export BulletSimGitVersion=$(git rev-parse HEAD)
export BulletSimGitVersionShort=$(git rev-parse --short HEAD)
cd bullet3
export BulletVersion=$(cat VERSION)
export BulletGitVersion=$(git rev-parse HEAD)
export BulletGitVersionShort=$(git rev-parse --short HEAD)

echo "=== Creating version information file"
cd "$BASE"
rm -f BulletSimVersionInfo
echo "BuildDate=$BuildDate" > BulletSimVersionInfo
echo "BulletSimVersion=$BulletSimVersion" >> BulletSimVersionInfo
echo "BulletSimGitVersion=$BulletSimGitVersion" >> BulletSimVersionInfo
echo "BulletSimGitVersionShort=$BulletSimGitVersionShort" >> BulletSimVersionInfo
echo "BulletVersion=$BulletVersion" >> BulletSimVersionInfo
echo "BulletGitVersion=$BulletGitVersion" >> BulletSimVersionInfo
echo "BulletGitVersionShort=$BulletGitVersionShort" >> BulletSimVersionInfo
echo "USEOPENCL=$USEOPENCL" >> BulletSimVersionInfo
cat BulletSimVersionInfo

echo "=== removing libBulletSim-*"
rm -f libBulletSim-*.so

if [ "$BUILDBULLET3" = "yes" ]; then
    echo "=== building bullet3"
    cd "$BASE"
    # Build the Bullet physics engine
    export USEOPENCL="$USEOPENCL"
    BULLETDIR=bullet3 ./buildBulletCMake.sh
    # Build the BulletSim glue/wrapper statically linked to Bullet
    ./buildBulletSim.sh
fi
