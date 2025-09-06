#! /bin/bash
# Script to build Bullet on a target system.

STARTDIR=$(pwd)

# The UNAME is either "Darwin" or otherwise. Note that env variable overrides
UNAME=${BULLETUNAME:-$(uname)}
# The MACH is either 'x86_64', 'aarch64', or assumed generic. Note env variable overrides.
MACH=${BULLETMACH:-$(uname -m)}
# Note that this sets BULLETDIR unless there is an environment variable of the same name
BULLETDIR=${BULLETDIR:-bullet3}

# Default TARGET_OS if not set
TARGET_OS=${TARGET_OS:-linux}

BUILDDIR=bullet-build

cd "${BULLETDIR}"
mkdir -p "${BUILDDIR}"
cd "${BUILDDIR}"

valid=false

# Only show architecture selection for x86_64 builds
if [[ "$MACH" == "x86_64" ]]; then
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
else
    # Default for non-x86_64
    compiler_flag=""
    arch="generic"
fi

echo "=== Building Bullet in dir ${BULLETDIR} for uname ${UNAME} and arch ${MACH} into ${BUILDDIR}"

# Use GNU C++ standard which includes M_PI definition
CXX_STANDARD="-std=gnu++17"

# Check if we're building for Windows
if [[ "$TARGET_OS" == "windows" ]]; then
    echo "=== Building for Windows"
    
    # Check if we have MinGW available
    if command -v x86_64-w64-mingw32-g++ >/dev/null 2>&1; then
        # Use MinGW cross-compilation
        echo "Using MinGW cross-compilation"
        CMAKE_GENERATOR="Unix Makefiles"
        CMAKE_EXTRA_FLAGS=(
            "-DCMAKE_SYSTEM_NAME=Windows"
            "-DCMAKE_C_COMPILER=x86_64-w64-mingw32-gcc"
            "-DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++"
            "-DCMAKE_RC_COMPILER=x86_64-w64-mingw32-windres"
        )
    elif command -v mingw32-make >/dev/null 2>&1; then
        # Use native MinGW
        echo "Using native MinGW"
        CMAKE_GENERATOR="MinGW Makefiles"
        CMAKE_EXTRA_FLAGS=()
    else
        echo "Error: No MinGW compiler found."
        echo "To install MinGW-w64 for cross-compilation on Ubuntu/Debian, run:"
        echo "  sudo apt install mingw-w64"
        echo ""
        echo "Or if you want to build for Linux instead, set TARGET_OS=linux"
        exit 1
    fi
    
	# Common Windows flags for ALL builds
	WINDOWS_FLAGS=(
		"-DBUILD_SHARED_LIBS=OFF"
		"-DBUILD_BULLET2_DEMOS=OFF"
		"-DBUILD_EXTRAS=ON"
		"-DBUILD_BULLET3=ON"
		"-DUSE_OPENCL=ON"
		"-DUSE_OPENMP=ON"
		"-DBULLET2_MULTITHREADING=ON"
		"-DBULLET2_USE_OPEN_MP_MULTITHREADING=ON"
		"-DBT_THREADSAFE=ON"
		"-DBT_USE_OPENMP=ON"
		"-DBUILD_UNIT_TESTS=OFF"
		"-DBUILD_BULLET_ROBOTICS_EXTRA=ON"
		"-DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF"
		"-DBUILD_HACD_EXTRA=ON"
		"-DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON"
		"-DBUILD_SERIALIZE_EXTRA=ON"
		"-DBUILD_INVERSE_DYNAMIC_EXTRA=ON"
		"-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
		"-DCMAKE_BUILD_TYPE=Release"
		"-DCMAKE_CXX_STANDARD=17"
		"-DCMAKE_CXX_EXTENSIONS=ON"
	)

	# Set appropriate flags based on compiler
	if command -v x86_64-w64-mingw32-g++ >/dev/null 2>&1; then
		# MinGW cross-compilation flags - apply Windows definitions to ALL compilations
		WINDOWS_FLAGS+=(
			"-DCMAKE_C_FLAGS=-fPIC $compiler_flag -fopenmp -static -static-libgcc -static-libstdc++ -D_WINDOWS -D_USRDLL -D_CRT_SECURE_NO_WARNINGS"
			"-DCMAKE_CXX_FLAGS=-fPIC -DBT_XML_SUPPORT -O3 $compiler_flag -ffast-math -fvisibility=hidden -UBT_USE_DOUBLE_PRECISION -fopenmp -static -static-libgcc -static-libstdc++ -D_WINDOWS -D_USRDLL -D_CRT_SECURE_NO_WARNINGS $CXX_STANDARD"
			"-DCMAKE_EXE_LINKER_FLAGS=-static -fopenmp -lgomp -lwinpthread -Wl,--enable-auto-import"
		)
	else
		# Native MinGW flags - apply Windows definitions to ALL compilations
		WINDOWS_FLAGS+=(
			"-DCMAKE_C_FLAGS=-fPIC -fopenmp -D_WINDOWS -D_USRDLL -D_CRT_SECURE_NO_WARNINGS"
			"-DCMAKE_CXX_FLAGS=-fPIC -DBT_XML_SUPPORT -O3 -ffast-math -fvisibility=hidden -UBT_USE_DOUBLE_PRECISION -fopenmp -D_WINDOWS -D_USRDLL -D_CRT_SECURE_NO_WARNINGS $CXX_STANDARD"
			"-DCMAKE_EXE_LINKER_FLAGS=-fopenmp -static -static-libgcc -static-libstdc++ -Wl,-Bstatic -lgomp -lwinpthread -Wl,-Bdynamic -Wl,--enable-auto-import"
		)
	fi

	# Add Windows system libraries to ensure proper linking
	WINDOWS_FLAGS+=(
		"-DCMAKE_FIND_LIBRARY_SUFFIXES=.a;.lib"
		"-DCMAKE_FIND_LIBRARY_PREFIXES=lib;"
	)

	# Run CMake with Windows configuration
	cmake .. -G "$CMAKE_GENERATOR" "${CMAKE_EXTRA_FLAGS[@]}" "${WINDOWS_FLAGS[@]}"
    
elif [[ "$UNAME" == "Darwin" ]] ; then
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
                -DCMAKE_CXX_FLAGS="-arch arm64 -arch x86_64 $CXX_STANDARD" \
                -DCMAKE_C_FLAGS="-arch arm64 -arch x86_64 -fPIC -O2" \
                -DCMAKE_EXE_LINKER_FLAGS="-arch arm64 -arch x86_64" \
                -DCMAKE_VERBOSE_MAKEFILE="on" \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CXX_STANDARD=17 \
                -DCMAKE_CXX_EXTENSIONS=ON
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
			-DBULLET2_MULTITHREADING=ON \
			-DBULLET2_USE_OPEN_MP_MULTITHREADING=ON \
			-DBT_THREADSAFE=ON \
			-DBT_USE_OPENMP=ON \
            -DBUILD_UNIT_TESTS=OFF \
            -DBUILD_BULLET_ROBOTICS_EXTRA=ON \
            -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
            -DBUILD_HACD_EXTRA=ON \
            -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
            -DBUILD_SERIALIZE_EXTRA=ON \
            -DBUILD_INVERSE_DYNAMIC_EXTRA=ON \
            -DCMAKE_C_FLAGS="-fPIC -fopenmp $compiler_flag" \
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-fPIC -DBT_XML_SUPPORT -O3 $compiler_flag -ffast-math -fvisibility=default -UBT_USE_DOUBLE_PRECISION -fopenmp $CXX_STANDARD" \
            -DCMAKE_EXE_LINKER_FLAGS="-lgomp -Wl,-Bstatic -lgomp -Wl,-Bdynamic" \
            -DCMAKE_CXX_STANDARD=17 \
            -DCMAKE_CXX_EXTENSIONS=ON
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
                -DCMAKE_CXX_FLAGS="-arch arm64 $CXX_STANDARD" \
                -DCMAKE_C_FLAGS="-arch arm64 -fPIC -O2" \
                -DCMAKE_EXE_LINKER_FLAGS="-arch arm64" \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CXX_STANDARD=17 \
                -DCMAKE_CXX_EXTENSIONS=ON
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
                -DBUILD_UNIT_TESTESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_CXX_FLAGS="-fPIC $CXX_STANDARD" \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_CXX_STANDARD=17 \
                -DCMAKE_CXX_EXTENSIONS=ON
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
find . -name "*.a" -exec cp {} lib/Release/ \; 2>/dev/null || true
find . -name "*.lib" -exec cp {} lib/Release/ \; 2>/dev/null || true

# Now proceed with the original copying logic
echo "=== Cleaning out any existing lib and include directories"
cd "$STARTDIR"
rm -rf lib
rm -rf include

echo "=== Moving .a files into ../lib"
cd "$STARTDIR"
mkdir -p lib
for afile in $(find "${BULLETDIR}/${BUILDDIR}/lib/Release" -name "*.a" -o -name "*.lib" 2>/dev/null || true) ; do
    cp "$afile" lib
done

echo "=== Copying debug symbols if they exist"
find "${BULLETDIR}/${BUILDDIR}" -name '*.pdb' -o -name '*.dSYM' 2>/dev/null | while read file; do
    cp "$file" "$STARTDIR/lib/"
done || true

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

echo "=== Checking for undefined symbols in built libraries"

# Check if nm command is available and lib directory exists
if command -v nm >/dev/null 2>&1 && [[ -d "$STARTDIR/lib" ]]; then
    echo "Using nm to check for undefined symbols..."
    
    # Check each library for undefined symbols
    cd "$STARTDIR"
    for lib in $(find lib -name "*.a" -o -name "*.lib" 2>/dev/null); do
        echo "Checking $lib for undefined symbols..."
        
        # Get undefined symbols (marked with 'U')
        undefined_symbols=$(nm -g "$lib" | grep " U " | head -20)
        
        if [[ -n "$undefined_symbols" ]]; then
            echo "⚠️  WARNING: Found undefined symbols in $lib:"
            echo "$undefined_symbols"
            
            # Check specifically for __ImageBase which causes relocation issues
            if echo "$undefined_symbols" | grep -q "__ImageBase"; then
                echo "❌ CRITICAL: Found __ImageBase symbol which causes relocation errors on Windows!"
                echo "   This typically indicates missing Windows system library linking."
            fi
        else
            echo "✓ No undefined symbols found in $lib"
        fi
        echo ""
    done
else
    if [[ ! -d "$STARTDIR/lib" ]]; then
        echo "⚠️  lib directory not found - skipping symbol check"
    else
        echo "⚠️  nm command not available - skipping symbol check"
    fi
fi

# Additional check specifically for Windows builds
if [[ "$TARGET_OS" == "windows" ]] && [[ -d "$STARTDIR/lib" ]]; then
    echo "=== Performing Windows-specific symbol checks"
    
    # Check for common Windows symbols that should be resolved
    cd "$STARTDIR"
    for lib in $(find lib -name "*.a" -o -name "*.lib" 2>/dev/null); do
        echo "Checking Windows symbols in $lib..."
        
        # Check for common Windows API functions that might be undefined
        windows_symbols=$(nm -g "$lib" | grep -E " U (CreateWindow|RegisterClass|GetModuleHandle|GetProcAddress|LoadLibrary)" || true)
        
        if [[ -n "$windows_symbols" ]]; then
            echo "⚠️  WARNING: Found potentially problematic Windows symbols in $lib:"
            echo "$windows_symbols"
            echo "   These should be resolved by linking against Windows system libraries."
        fi
    done
fi

echo "=== Bullet build completed successfully!"
echo "Libraries are in: $STARTDIR/lib"
echo "Headers are in: $STARTDIR/include"