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

# Determine target platform and set appropriate flags
if [[ "$TARGET_OS" == "windows" ]]; then
    echo "=== Building for Windows (MinGW)"
    CC=x86_64-w64-mingw32-gcc
    CXX=x86_64-w64-mingw32-g++
    EXT=".dll"
    # Windows-specific flags for proper DLL linking
    CFLAGS="$CFLAGS -D_WINDOWS -D_USRDLL -D_CRT_SECURE_NO_WARNINGS"
    # Windows linking flags - add essential Windows libraries
    LDFLAGS="-static-libstdc++ -static-libgcc -lgomp -lwinpthread -lole32 -loleaut32 -luuid -Wl,--enable-auto-import"
    
    # Windows-specific OpenCL paths
    OPENCL_PATHS=(
        "$HOME/OpenCL-SDK/include"
        "/usr/x86_64-w64-mingw32/include"
        "/mingw64/include"
    )
else
    echo "=== Building for Linux"
    CC=gcc
    CXX=g++
    EXT=".so"
    # Follow Linux best practices: static link only OpenMP, dynamic link system libraries
    LDFLAGS="-lgomp"
    
    # Linux-specific OpenCL paths
    OPENCL_PATHS=(
        "$HOME/OpenCL-SDK/include"
        "/usr/include"
        "/usr/local/include"
        "/usr/local/cuda/include"
        "/opt/AMDAPP/include"
    )
fi

# Update target name with appropriate extension
TARGET="${TARGETBASE}-${BUILDDATE}-${ARCH}${EXT}"

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

# Pass version information into compilations as C++ variables
VERSIONCFLAGS="-DBULLETVERSION='\"${BULLETVERSION}\"' -DBULLETSIMVERSION='\"${BULLETSIMVERSION}\"'"

# Compiler flags
CFLAGS="-I${BINCLUDEDIR} -I. -fPIC -g -fpermissive ${VERSIONCFLAGS} -DBULLETSIM_EXPORTS -UBT_USE_DOUBLE_PRECISION -O3 $compiler_flag -ffast-math -fopenmp"

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

# OpenCL support discovery - platform-specific
echo "=== Checking for OpenCL support"
OPENCL_CFLAGS=""
OPENCL_LIBS=""

# Check for OpenCL SDK installation
if [[ -d "$HOME/OpenCL-SDK" ]]; then
    OPENCL_CFLAGS="-I$HOME/OpenCL-SDK/include -D BT_USE_OPENCL"
    echo "✓ Using OpenCL SDK at: $HOME/OpenCL-SDK"
    
    if [[ "$TARGET_OS" == "windows" ]]; then
        # For Windows, use the specific OpenCL import library
        OPENCL_LIBS="-L$HOME/OpenCL-SDK/lib -l:libOpenCL.dll.a"
        echo "✓ Using Windows OpenCL library: libOpenCL.dll.a"
    else
        # For Linux, use standard OpenCL linking
        OPENCL_LIBS="-L$HOME/OpenCL-SDK/lib -lOpenCL"
        echo "✓ Using Linux OpenCL library"
    fi
else
    echo "⚠ Warning: OpenCL SDK not found at $HOME/OpenCL-SDK. Disabling OpenCL support."
    OPENCL_CFLAGS="-D NO_OPENCL"
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
echo "=== Compiling source files with $CXX"
echo "Using include directory: ${BINCLUDEDIR}"
echo "Using library directory: ${BLIBDIR}"

$CXX ${CFLAGS} -c API2.cpp
if [[ $? -ne 0 ]]; then
    echo "Failed to compile API2.cpp"
    exit 1
fi

$CXX ${CFLAGS} -c BulletSim.cpp
if [[ $? -ne 0 ]]; then
    echo "Failed to compile BulletSim.cpp"
    exit 1
fi

# Link the shared library
echo "=== Building target ${TARGET}"
echo "Linking with locally built Bullet libraries"

# For Windows builds, ensure we link against required system libraries
if [[ "$TARGET_OS" == "windows" ]]; then
    # Add ALL necessary Windows system libraries
    WINDOWS_LIBS="-lwinpthread -lole32 -loleaut32 -luuid -ladvapi32 -lkernel32 -luser32 -lgdi32 -lsetupapi -lversion -lshell32 -lcomdlg32"
    
    # Use the correct OpenCL library from your SDK
    OPENCL_LIBS="-L$HOME/OpenCL-SDK/lib -l:libOpenCL.dll.a"
    
    LINK_COMMAND="$CXX -shared -o \"${TARGET}\" API2.o BulletSim.o ${BULLETLIBS} ${OPENCL_LIBS} -L${BLIBDIR} ${LDFLAGS} ${WINDOWS_LIBS} -Wl,--enable-auto-import -Wl,--image-base,0x10000000"
else
    LINK_COMMAND="$CXX -shared -o \"${TARGET}\" API2.o BulletSim.o ${BULLETLIBS} ${OPENCL_LIBS} -L${BLIBDIR} ${LDFLAGS}"
fi


# Execute the link command
echo "Running: $LINK_COMMAND"
eval $LINK_COMMAND

# Check if build was successful
if [[ $? -eq 0 ]]; then
    echo "=== Build successful: ${TARGET}"

    # Verify the shared library was created
    if [[ -f "${TARGET}" ]]; then
        echo "✓ Shared library created successfully"

        # Platform-specific symbol verification
        if [[ "$TARGET_OS" == "windows" ]]; then
            echo "=== Verifying DLL exports using objdump..."
            
            # Check for export table
            if x86_64-w64-mingw32-objdump -p "${TARGET}" | grep -q "Export Address Table"; then
                echo "✓ DLL has an export table"
                
                # Try to find BSLog (might be name-mangled)
                if x86_64-w64-mingw32-objdump -p "${TARGET}" | grep -i "BSLog"; then
                    echo "✓ Found BSLog symbol in export table"
                else
                    echo "⚠ Note: BSLog symbol not found via objdump (may be name-mangled)"
                    echo "   Trying strings method..."
                    if strings "${TARGET}" | grep -i "BSLog"; then
                        echo "✓ Found BSLog string in binary"
                    else
                        echo "⚠ Warning: BSLog not found via strings either"
                    fi
                fi
            else
                echo "⚠ Warning: No export table found in DLL"
            fi
            
            # Check for OpenMP (statically linked, so symbols won't be exported)
            echo "=== OpenMP is statically linked, symbols won't appear in export table"
            
        else
            # Linux verification with nm
            echo "=== Verifying WorldData::BSLog symbol is present..."
            if nm -D "${TARGET}" | grep -q "BSLog"; then
                echo "✓ Found BSLog symbol"
            else
                echo "⚠ Warning: BSLog symbol not found in exports - may cause runtime errors"
            fi

            echo "=== Verifying OpenMP support..."
            if nm -D "${TARGET}" | grep -q "omp_get_num_threads"; then
                echo "✓ Found OpenMP symbol: omp_get_num_threads"
            else
                echo "⚠ Warning: OpenMP symbol omp_get_num_threads not found in exports - OpenMP may not be linked correctly"
            fi
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