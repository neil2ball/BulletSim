#! /bin/bash
# Script to build BulletSim Linux binaries.
# This presumes the bins and includes for Bullet are in BLIBDIR and BINCLUDEDIR

BASE=$(pwd)

BLIBDIR=${BLIBDIR:-./lib}
BINCLUDEDIR=${BINCLUDEDIR:-./include}

# Verify Bullet include directory exists
if [[ ! -d "${BINCLUDEDIR}" ]]; then
    echo "Warning: Bullet include directory ${BINCLUDEDIR} not found"
    # Try to find system Bullet includes
    if [[ -d "/usr/include/bullet" ]]; then
        BINCLUDEDIR="/usr/include/bullet"
        echo "Using system Bullet includes: ${BINCLUDEDIR}"
    elif [[ -d "/usr/local/include/bullet" ]]; then
        BINCLUDEDIR="/usr/local/include/bullet"
        echo "Using local Bullet includes: ${BINCLUDEDIR}"
    else
        echo "Error: No Bullet include directory found!"
        echo "Please install Bullet development packages or set BINCLUDEDIR"
        exit 1
    fi
fi

# Check for system-installed Bullet libraries first
SYSTEM_BULLET_LIBS=(
    "/usr/lib/x86_64-linux-gnu"
    "/usr/lib"
    "/usr/local/lib"
)

# Try to find the best library directory
FOUND_LIBDIR=""
for libpath in "${SYSTEM_BULLET_LIBS[@]}"; do
    if [[ -d "${libpath}" ]]; then
        # Check if this path has Bullet shared libraries (from system packages)
        if [[ -f "${libpath}/libBulletDynamics.so" || -f "${libpath}/libBulletCollision.so" || -f "${libpath}/libBulletXmlWorldImporter.so" ]]; then
            FOUND_LIBDIR="${libpath}"
            echo "Found system Bullet shared libraries in: ${FOUND_LIBDIR}"
            break
        # Check for static libraries as fallback
        elif [[ -f "${libpath}/libBulletDynamics.a" || -f "${libpath}/libBulletCollision.a" || -f "${libpath}/libBulletXmlWorldImporter.a" ]]; then
            FOUND_LIBDIR="${libpath}"
            echo "Found Bullet libraries in: ${FOUND_LIBDIR}"
            break
        fi
    fi
done

if [[ -n "${FOUND_LIBDIR}" ]]; then
    BLIBDIR="${FOUND_LIBDIR}"
    echo "Using library directory: ${BLIBDIR}"
else
    echo "Error: No Bullet library directory found!"
    echo "Please install Bullet development packages or set BLIBDIR"
    exit 1
fi

# Output file is ${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-${ARCH}.so
TARGETBASE=${TARGETBASE:-libBulletSim-3.27}  # Changed to match OpenSim expected name

CC=/usr/bin/c++
LD=/usr/bin/c++

UNAME=${UNAME:-$(uname)}
ARCH=${ARCH:-$(uname -m)}

# Version of the Bullet engine that is being statically linked
# Check if VERSION file exists before trying to read it
if [[ -f "${BLIBDIR}/VERSION" ]]; then
    BULLETVERSION=$(cat "${BLIBDIR}/VERSION")
else
    BULLETVERSION="3.27"  # Default to expected version
    echo "Warning: ${BLIBDIR}/VERSION not found, using '${BULLETVERSION}'"
fi
# Version of the BulletSim glue that is being built and included
BULLETSIMVERSION=$(cat "VERSION")

BUILDDATE=$(date "+%Y%m%d")

# Kludge for building libBulletSim.so with different library dependencies
if [[ "$ARCH" == "x86_64" ]] ; then
    WRAPMEMCPY=-Wl,--wrap=memcpy
else
    WRAPMEMCPY=
fi

# Pass version information into compilations as C++ variables
VERSIONCFLAGS="-D BULLETVERSION=$BULLETVERSION -D BULLETSIMVERSION=$BULLETSIMVERSION"
case $UNAME in
    "Linux")
        TARGET=${TARGETBASE}-${BUILDDATE}-${ARCH}.so  # Removed BULLETVERSION from filename
        CFLAGS="-I${BINCLUDEDIR} -I/usr/include/bullet -I/usr/local/include/bullet -I/usr/include -I/usr/local/include -fPIC -g -fpermissive ${VERSIONCFLAGS}"
        LFLAGS="${WRAPMEMCPY} -shared -Wl,-soname,${TARGET} -L${BLIBDIR} -o ${TARGET}"
        ;;
    "Darwin")
        CC=gcc
        LD=g++
        TARGET=${TARGETBASE}-${BUILDDATE}-universal.dylib
        CFLAGS="-arch arm64 -arch x86_64 -O3 -I${BINCLUDEDIR} -I/usr/local/include/bullet -I/usr/local/include -g ${VERSIONCFLAGS}"
        LFLAGS="-v -dynamiclib -arch arm64 -arch x86_64 -L${BLIBDIR} -o ${TARGET}"
        ;;
    *)
        TARGET=${TARGETBASE}-${BUILDDATE}-${ARCH}.so
        CFLAGS="-I${BINCLUDEDIR} -I/usr/local/include/bullet -I/usr/local/include -fPIC -g -fpermissive ${VERSIONCFLAGS}"
        LFLAGS="${WRAPMEMCPY} -shared -Wl,-soname,${TARGET} -L${BLIBDIR} -o ${TARGET}"
        ;;
esac

# Function to verify a library exists
verify_library() {
    local lib_name=$1
    local lib_path_a="${BLIBDIR}/${lib_name}.a"
    local lib_path_so="${BLIBDIR}/${lib_name}.so"
    
    if [[ ! -f "$lib_path_a" && ! -f "$lib_path_so" ]]; then
        echo "Error: Library ${lib_name} not found in ${BLIBDIR}!"
        echo "Looking for: ${lib_path_a} or ${lib_path_so}"
        return 1
    fi
    return 0
}

# Verify all required Bullet libraries exist
REQUIRED_BULLET_LIBS=(
    "libBulletDynamics"
    "libBulletCollision" 
    "libLinearMath"
    "libBulletXmlWorldImporter"
)

# Optional libraries (warn if missing but continue)
OPTIONAL_BULLET_LIBS=(
    "libHACD"
)

echo "=== Verifying required libraries in ${BLIBDIR}"

for lib in "${REQUIRED_BULLET_LIBS[@]}"; do
    if ! verify_library "$lib"; then
        echo "Please install the missing Bullet library or build from source"
        exit 1
    fi
    echo "✓ Found: ${lib}"
done

for lib in "${OPTIONAL_BULLET_LIBS[@]}"; do
    if verify_library "$lib"; then
        echo "✓ Found: ${lib} (optional)"
    else
        echo "⚠ Warning: ${lib} not found (optional)"
    fi
done

# Use FULL PATH to shared libraries to ensure proper linking
BULLETLIBS=""
for lib in "${REQUIRED_BULLET_LIBS[@]}" "${OPTIONAL_BULLET_LIBS[@]}"; do
    if [[ -f "${BLIBDIR}/${lib}.so" ]]; then
        BULLETLIBS+="${BLIBDIR}/${lib}.so "
        echo "✓ Using shared library: ${BLIBDIR}/${lib}.so"
    elif [[ -f "${BLIBDIR}/${lib}.a" ]]; then
        echo "⚠ Warning: Using static library ${lib}.a - may cause PIC issues"
        BULLETLIBS+="${BLIBDIR}/${lib}.a "
    fi
done

# Verify OpenCL library exists
echo "=== Verifying OpenCL library"
if ! ldconfig -p | grep -q libOpenCL.so; then
    echo "⚠ Warning: OpenCL library not found in system library path"
    echo "OpenCL support may not be available"
fi

# Additional system libraries needed for OpenCL
OPENCL_LIBS="-lOpenCL"

# Verify source files exist
if [[ ! -f "API2.cpp" ]]; then
    echo "Error: API2.cpp not found!"
    exit 1
fi

if [[ ! -f "BulletSim.cpp" ]]; then
    echo "Error: BulletSim.cpp not found!"
    exit 1
fi

# Just build everything
echo "=== Building target $TARGET from BulletSim glue ${BULLETSIMVERSION} and Bullet ${BULLETVERSION}"
echo "Using include directory: ${BINCLUDEDIR}"
echo "Using library directory: ${BLIBDIR}"
echo "Linking with Bullet XML library for WorldData::BSLog symbol"
echo "Libraries to link: ${BULLETLIBS}"

${CC} ${CFLAGS} -c API2.cpp
${CC} ${CFLAGS} -c BulletSim.cpp

# Link with the appropriate libraries using FULL PATHS
${LD} ${LFLAGS} API2.o BulletSim.o ${BULLETLIBS} ${OPENCL_LIBS}

# Check if build was successful
if [[ $? -eq 0 ]]; then
    echo "=== Build successful: ${TARGET}"
    echo "=== Verifying WorldData::BSLog symbol is present..."
    if nm -D ${TARGET} | grep -q "T.*BSLog"; then
        echo "✓ Found defined BSLog symbol"
    elif nm -D ${TARGET} | grep -q "BSLog"; then
        echo "⚠ Warning: BSLog symbol found but undefined (U) - linking issue"
        echo "This may cause runtime errors in OpenSim"
    else
        echo "❌ Error: BSLog symbol not found at all"
    fi
    
    # Verify other important symbols
    echo "=== Verifying other important symbols..."
    nm -D ${TARGET} | grep -E "(_Z|\bBullet|Sim|World)" | head -5
    
else
    echo "=== Build failed!"
    echo "=== Trying alternative linking approach..."
    
    # Try linking with explicit library names instead of paths
    ALT_BULLETLIBS=""
    for lib in "${REQUIRED_BULLET_LIBS[@]}" "${OPTIONAL_BULLET_LIBS[@]}"; do
        ALT_BULLETLIBS+="-l${lib#lib} "
    done
    
    echo "=== Attempting to link with: ${ALT_BULLETLIBS}"
    ${LD} ${LFLAGS} API2.o BulletSim.o ${ALT_BULLETLIBS} ${OPENCL_LIBS} -L${BLIBDIR}
    
    if [[ $? -eq 0 ]]; then
        echo "=== Build successful with alternative linking: ${TARGET}"
    else
        echo "=== Build failed completely!"
        echo "=== Please check that Bullet libraries are properly installed"
        echo "=== You may need to rebuild Bullet with:"
        echo "    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=ON"
        exit 1
    fi
fi
