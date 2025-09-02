#! /bin/bash
# Script to build Bullet on a target system.

STARTDIR=$(pwd)

# The UNAME is either "Darwin" or otherwise. Note that env variable overrides
UNAME=${BULLETUNAME:-$(uname)}
# The MACH is either 'x86_64', 'aarch64', or assumed generic. Note env variable overrides.
MACH=${BULLETMACH:-$(uname -m)}
# Note that this sets BULLETDIR unless there is an environment variable of the same name
BULLETDIR=${BULLETDIR:-bullet3}

# Check and set OpenCL environment variables if needed
if [ -z "$OPENCL_SDK_ROOT" ]; then
    OPENCL_SDK_ROOT="$HOME/OpenCL-SDK"
fi
if [ -z "$OpenCL_INCLUDE_DIR" ]; then
    OpenCL_INCLUDE_DIR="$OPENCL_SDK_ROOT/include"
fi
if [ -z "$OpenCL_LIBRARY_DIR" ]; then
    OpenCL_LIBRARY_DIR="$OPENCL_SDK_ROOT/lib"
fi

# Verify OpenCL SDK exists
if [ ! -d "$OpenCL_INCLUDE_DIR" ]; then
    echo "OpenCL include directory not found: $OpenCL_INCLUDE_DIR"
    echo "Please install OpenCL SDK first or set OPENCL_SDK_ROOT environment variable"
    exit 1
fi

BUILDDIR=bullet-build

cd "${BULLETDIR}"
mkdir -p "${BUILDDIR}"
cd "${BUILDDIR}"

valid=false

until $valid; do
    echo "Select the architecture level to build for:"
    echo "1. x86-64-v1 (SSE2)"
    echo "2. x86-64-v2 (AVX)"
    echo "3. x86-64-v3 (AVX2)"
    echo "4. x86-64-v4 (AVX-512)"

    read -p "Enter your choice (1-4): " choice

    case "$choice" in
        1) arch="x86-64-v1"; compiler_flag="-march=x86-64"; valid=true ;;
        2) arch="x86-64-v2"; compiler_flag="-march=x86-64-v2"; valid=true ;;
        3) arch="x86-64-v3"; compiler_flag="-march=x86-64-v3"; valid=true ;;
        4) arch="x86-64-v4"; compiler_flag="-march=x86-64-v4"; valid=true ;;
        *) echo "Invalid selection. Please try again." ;;
    esac
    echo ""
done

echo "Selected architecture: $arch"
echo "GCC compiler flag: $compiler_flag"


echo "=== Building Bullet in dir ${BULLETDIR} for uname ${UNAME} and arch ${MACH} into ${BUILDDIR}"
echo "Using OpenCL SDK at: $OPENCL_SDK_ROOT"

if [[ "$UNAME" == "Darwin" ]] ; then
    echo "=== Running cmake for Darwin"
    cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=ON \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=ON \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_OSX_ARCHITECTURES="arm64; x86_64" \
                -DCMAKE_CXX_FLAGS="-arch arm64 -arch x86_64" \
                -DCMAKE_C_FLAGS="-arch arm64 -arch x86_64 -fPIC -O2" \
                -DCMAKE_EXE_LINKER_FLAGS="-arch arm64 -arch x86_64" \
                -DCMAKE_VERBOSE_MAKEFILE="on" \
                -DCMAKE_BUILD_TYPE=Release
elif [[ "$UNAME" =~ "MINGW64*" ]] ; then
    cmake .. -G "Visual Studio 17 2022" \
            -DBUILD_BULLET3=ON \
            -DBUILD_EXTRAS=ON \
                -DBUILD_INVERSE_DYNAMIC_EXTRA=ON \
                -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                -DBUILD_OBJ2SDF_EXTRA=OFF \
                -DBUILD_SERIALIZE_EXTRA=ON \
                -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                -DBUILD_HACD_EXTRA=ON \
                -DBUILD_GIMPACTUTILS_EXTRA=OFF \
            -DBUILD_CPU_DEMOS=OFF \
            -DBUILD_BULLET2_DEMOS=OFF \
            -DBUILD_ENET=OFF \
            -DBUILD_PYBULLET=OFF \
            -DBUILD_UNIT_TESTS=OFF \
            -DBUILD_SHARED_LIBS=OFF \
            -DINSTALL_EXTRA_LIBS=ON \
            -DINSTALL_LIBS=ON \
            -DCMAKE_CXX_FLAGS="-fPIC" \
            -DCMAKE_BUILD_TYPE=Release
else
    if [[ "$MACH" == "x86_64" ]]
    then
        echo "=== Running cmake for arch $MACH with OpenCL support"
        cmake .. \
            -DBUILD_SHARED_LIBS=OFF \
            -DBUILD_BULLET2_DEMOS=OFF \
            -DBUILD_EXTRAS=ON \
            -DBUILD_BULLET3=ON \
            -DUSE_OPENCL=ON \
	    -DUSE_OPENMP=ON \
            -DOpenCL_INCLUDE_DIR="$OpenCL_INCLUDE_DIR" \
            -DOpenCL_LIBRARY_DIR="$OpenCL_LIBRARY_DIR" \
            -DBUILD_UNIT_TESTS=OFF \
            -DBUILD_BULLET_ROBOTICS_EXTRA=ON \
            -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
            -DBUILD_HACD_EXTRA=ON \
            -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
            -DBUILD_SERIALIZE_EXTRA=ON \
            -DBUILD_INVERSE_DYNAMIC_EXTRA=ON \
            -DCMAKE_C_FLAGS="-fPIC" \
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
            -DOpenGL_GL_PREFERENCE=GLVND \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-fPIC -DBT_XML_SUPPORT -O3 $compiler_flag -ffast-math -fvisibility=default -UBT_USE_DOUBLE_PRECISION"
    elif [[ "$MACH" == "aarch64" ]]
    then
        echo "=== Running cmake for arch $MACH"
        cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=ON \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=ON \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_OSX_ARCHITECTURES="arm64" \
                -DCMAKE_CXX_FLAGS="-arch arm64" \
                -DCMAKE_C_FLAGS="-arch arm64 -fPIC -O2" \
                -DCMAKE_EXE_LINKER_FLAGS="-arch arm64" \
                -DCMAKE_BUILD_TYPE=Release
    else
        echo "=== Running cmake for generic arch"
        cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=ON \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=ON \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_CXX_FLAGS="-fPIC" \
                -DCMAKE_BUILD_TYPE=Release
    fi
fi

# DEBUG DEBUG
echo "=== $(pwd)"
ls -l
echo "=== END"
# END DEBUG DEBUG

if [[ -e Makefile ]] ; then
    echo "=== Building Makefile"
    # Build in specific order to handle dependencies (matching Windows script order)
    make -j4 LinearMath
    make -j4 Bullet3Common
    make -j4 BulletCollision
    make -j4 BulletDynamics
    make -j4 BulletSoftBody
    make -j4 BulletInverseDynamics
    make -j4 HACD
    make -j4 Bullet3OpenCL_clew

    # Build Serialize extra projects
    make -j4 BulletFileLoader
    make -j4 BulletWorldImporter
    make -j4 BulletXmlWorldImporter

    # Build any remaining targets
    make -j4
fi
if [[ -e "BULLET_PHYSICS.sln" ]] ; then
    echo "=== Building BULLET_PHYSICS.sln"
    dotnet build -c Release BULLET_PHYSICS.sln
fi

# Create a lib directory in the build folder to match Windows behavior
mkdir -p lib/Release

# Copy all built libraries to the lib/Release directory first
find . -name "*.a" -exec cp {} lib/Release/ \;
find . -name "*.lib" -exec cp {} lib/Release/ \;

# Now proceed with the original copying logic
echo "=== Cleaning out any existing lib and include directories"
cd "$STARTDIR"
rm -rf lib
rm -rf include

echo "=== Moving .a files into ../lib"
cd "$STARTDIR"
mkdir -p lib
for afile in $(find "${BULLETDIR}/${BUILDDIR}/lib/Release" -name "*.a" -o -name "*.lib") ; do
    cp "$afile" lib
done

echo "=== Copying debug symbols if they exist"
find "${BULLETDIR}/${BUILDDIR}" -name '*.pdb' -o -name '*.dSYM' | while read file; do
    cp "$file" "$STARTDIR/lib/"
done

echo "=== Moving .h and .inl files into ../include"
cd "$STARTDIR"
mkdir -p include
cd "${BULLETDIR}/src"
find . -name '*.h' -o -name '*.inl' | while read file; do
    target_dir="${STARTDIR}/include/$(dirname "$file")"
    mkdir -p "$target_dir"
    cp "$file" "$target_dir"
done

# Move Bullet's VERSION file into lib/ so BulletSim can reference it
echo "=== Moving Bullet's VERSION file into ../lib"
cd "$STARTDIR"
mkdir -p lib
cp "${BULLETDIR}/VERSION" lib/

echo "=== Moving .h and .inl files from Extras into ../include"
cd "$STARTDIR"
cd "${BULLETDIR}/Extras"
find . -name '*.h' -o -name '*.inl' | while read file; do
    target_dir="${STARTDIR}/include/$(dirname "$file")"
    mkdir -p "$target_dir"
    cp "$file" "$target_dir"
done

echo "=== Bullet build completed successfully!"
echo "Libraries are in: $STARTDIR/lib"
echo "Headers are in: $STARTDIR/include"
