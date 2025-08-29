#! /bin/bash
# Script to build BulletSim Linux binaries.
# This presumes the bins and includes for Bullet are in BLIBDIR and BINCLUDEDIR

BASE=$(pwd)

BLIBDIR=${BLIBDIR:-./lib}
BINCLUDEDIR=${BINCLUDEDIR:-./include}

# Verify locally built Bullet include directory exists
if [[ ! -d "${BINCLUDEDIR}" ]]; then
    echo "Error: Bullet include directory ${BINCLUDEDIR} not found!"
    echo "Please build Bullet first using buildBulletCMake.sh"
    exit 1
fi

# Verify locally built Bullet library directory exists
if [[ ! -d "${BLIBDIR}" ]]; then
    echo "Error: Bullet library directory ${BLIBDIR} not found!"
    echo "Please build Bullet first using buildBulletCMake.sh"
    exit 1
fi

# Output file is ${TARGETBASE}-${BUILDDATE}-${ARCH}.so
TARGETBASE=${TARGETBASE:-libBulletSim-3.27}
ARCH=$(uname -m)
BUILDDATE=$(date "+%Y%m%d")
TARGET="${TARGETBASE}-${BUILDDATE}-${ARCH}.so"

# Version of the Bullet engine that is being linked
if [[ -f "${BLIBDIR}/VERSION" ]]; then
    BULLETVERSION=$(cat "${BLIBDIR}/VERSION" | tr -d '[:space:]')
else
    BULLETVERSION="3.27"
    echo "Warning: ${BLIBDIR}/VERSION not found, using '${BULLETVERSION}'"
fi

# Version of the BulletSim glue that is being built and included
if [[ ! -f "VERSION" ]]; then
    echo "Error: VERSION file not found in current directory!"
    exit 1
fi
BULLETSIMVERSION=$(cat "VERSION" | tr -d '[:space:]')

# Pass version information into compilations as C++ variables
VERSIONCFLAGS="-DBULLETVERSION='\"${BULLETVERSION}\"' -DBULLETSIMVERSION='\"${BULLETSIMVERSION}\"'"

# Compiler flags
#CFLAGS="-I${BINCLUDEDIR} -I. -fPIC -g -fpermissive ${VERSIONCFLAGS} -D BULLETSIM_EXPORTS"
CFLAGS="-I${BINCLUDEDIR} -I. -fPIC -g -fpermissive ${VERSIONCFLAGS} -DBULLETSIM_EXPORTS -UBT_USE_DOUBLE_PRECISION -O3 -mavx -ffast-math"

# Check for Extras include directory
EXTRAS_INCLUDE_PATH="${BINCLUDEDIR}/Extras"
if [[ -d "${EXTRAS_INCLUDE_PATH}" ]]; then
    CFLAGS="-I${EXTRAS_INCLUDE_PATH} ${CFLAGS}"
    echo "✓ Added Extras include path: ${EXTRAS_INCLUDE_PATH}"
fi

# Function to verify a library exists
verify_library() {
    local lib_name=$1
    local lib_path="${BLIBDIR}/lib${lib_name}.a"

    if [[ ! -f "$lib_path" ]]; then
        echo "Error: Library lib${lib_name}.a not found in ${BLIBDIR}!"
        return 1
    fi
    return 0
}

# Verify all required Bullet libraries exist (from local build)
REQUIRED_BULLET_LIBS=(
    "BulletDynamics"
    "BulletCollision"
    "LinearMath"
    "BulletXmlWorldImporter"
)

# Optional libraries (warn if missing but continue)
OPTIONAL_BULLET_LIBS=(
    "HACD"
    "Bullet3Common"
    "BulletInverseDynamicsUtils"
)

echo "=== Verifying locally built Bullet libraries in ${BLIBDIR}"

for lib in "${REQUIRED_BULLET_LIBS[@]}"; do
    if ! verify_library "$lib"; then
        echo "Required library lib${lib}.a missing! Rebuild Bullet with buildBulletCMake.sh"
        exit 1
    fi
    echo "✓ Found: lib${lib}.a"
done

for lib in "${OPTIONAL_BULLET_LIBS[@]}"; do
    if verify_library "$lib"; then
        echo "✓ Found: lib${lib}.a (optional)"
    else
        echo "⚠ Warning: lib${lib}.a not found (optional)"
    fi
done

# Build library list for linking
BULLETLIBS=""
for lib in "${REQUIRED_BULLET_LIBS[@]}" "${OPTIONAL_BULLET_LIBS[@]}"; do
    if [[ -f "${BLIBDIR}/lib${lib}.a" ]]; then
        BULLETLIBS+="${BLIBDIR}/lib${lib}.a "
        echo "✓ Using library: lib${lib}.a"
    fi
done

# OpenCL support discovery
echo "=== Checking for OpenCL support"
OPENCL_CFLAGS=""
OPENCL_LIBS=""

# Check for OpenCL headers
OPENCL_PATHS=(
    "$HOME/OpenCL-SDK/include"
    "/usr/include"
    "/usr/local/include"
    "/usr/local/cuda/include"
    "/opt/AMDAPP/include"
)

for path in "${OPENCL_PATHS[@]}"; do
    if [[ -f "${path}/CL/cl.h" ]]; then
        OPENCL_CFLAGS="-I${path} -D BT_USE_OPENCL"
        echo "✓ Found OpenCL headers in: ${path}"
        break
    elif [[ -f "${path}/cl.h" ]]; then
        OPENCL_CFLAGS="-I${path} -D BT_USE_OPENCL"
        echo "✓ Found OpenCL headers in: ${path}"
        break
    fi
done

if [[ -z "${OPENCL_CFLAGS}" ]]; then
    echo "⚠ Warning: OpenCL headers not found. Disabling OpenCL support."
    OPENCL_CFLAGS="-D NO_OPENCL"
fi

# Check for OpenCL library
if ldconfig -p | grep -q libOpenCL.so; then
    OPENCL_LIBS="-lOpenCL"
    echo "✓ Found OpenCL library"
else
    echo "⚠ Warning: OpenCL library not found"
fi

CFLAGS="${CFLAGS} ${OPENCL_CFLAGS}"

# Verify source files exist
if [[ ! -f "API2.cpp" ]]; then
    echo "Error: API2.cpp not found!"
    exit 1
fi

if [[ ! -f "BulletSim.cpp" ]]; then
    echo "Error: BulletSim.cpp not found!"
    exit 1
fi

# Compile source files
echo "=== Compiling source files"
echo "Using include directory: ${BINCLUDEDIR}"
echo "Using library directory: ${BLIBDIR}"

g++ ${CFLAGS} -c API2.cpp
if [[ $? -ne 0 ]]; then
    echo "Failed to compile API2.cpp"
    exit 1
fi

g++ ${CFLAGS} -c BulletSim.cpp
if [[ $? -ne 0 ]]; then
    echo "Failed to compile BulletSim.cpp"
    exit 1
fi

# Link the shared library
echo "=== Building target ${TARGET}"
echo "Linking with locally built Bullet libraries"

g++ -shared -o "${TARGET}" API2.o BulletSim.o ${BULLETLIBS} ${OPENCL_LIBS} -L${BLIBDIR}

# Check if build was successful
if [[ $? -eq 0 ]]; then
    echo "=== Build successful: ${TARGET}"

    # Verify the shared library was created
    if [[ -f "${TARGET}" ]]; then
        echo "✓ Shared library created successfully"

        # Verify BSLog symbol using nm
        echo "=== Verifying WorldData::BSLog symbol is present..."
        if nm -D "${TARGET}" | grep -q "BSLog"; then
            echo "✓ Found BSLog symbol"
        else
            echo "⚠ Warning: BSLog symbol not found in exports - may cause runtime errors"
        fi

    else
        echo "Error: Shared library was not created despite successful link"
        exit 1
    fi
else
    echo "=== Build failed!"
    echo "=== Common issues:"
    echo "    1. Bullet not built properly - run buildBulletCMake.sh again"
    echo "    2. Missing development tools (g++, make, etc.)"
    echo "    3. Missing source files in current directory"
    exit 1
fi

# Clean up object files
rm -f API2.o BulletSim.o

echo "=== BulletSim build completed successfully!"
echo "Output: $(readlink -f ${TARGET})"
