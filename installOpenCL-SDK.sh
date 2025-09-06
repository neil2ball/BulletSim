#!/bin/bash
# Script to download and build Khronos vendor-neutral OpenCL SDK from source

set -euo pipefail

INSTALL_PATH="${INSTALL_PATH:-$HOME/OpenCL-SDK}"
FORCE=false
BUILD_THREADS="${BUILD_THREADS:-$(nproc)}"
TARGET_OS="${TARGET_OS:-linux}"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--force)
            FORCE=true
            shift
            ;;
        -j|--jobs)
            BUILD_THREADS="$2"
            shift 2
            ;;
        --target-os)
            TARGET_OS="$2"
            shift 2
            ;;
        *)
            if [[ -z "${INSTALL_PATH_SET:-}" ]]; then
                INSTALL_PATH="$1"
                INSTALL_PATH_SET=true
            else
                echo "Unknown option: $1"
                exit 1
            fi
            shift
            ;;
    esac
done

echo "=== Khronos OpenCL SDK Builder ==="
echo "Target OS: $TARGET_OS"
echo "Install path: $INSTALL_PATH"

# Check if already installed
if [[ -d "$INSTALL_PATH" ]]; then
    if [[ "$FORCE" == false ]]; then
        echo "OpenCL SDK already exists at $INSTALL_PATH. Use -f or --force to rebuild."
        exit 0
    else
        echo "Removing existing installation..."
        rm -rf "$INSTALL_PATH"
    fi
fi

# Create installation directory
mkdir -p "$INSTALL_PATH"

# Check for build dependencies
echo "Checking for build dependencies..."
if [[ "$TARGET_OS" == "windows" ]]; then
    # Windows dependencies
    for dep in git cmake; do
        if ! command -v $dep &> /dev/null; then
            echo "Error: $dep is required but not installed. Please install it first."
            exit 1
        fi
    done
    
    # Check if we have a compatible compiler
    if ! command -v cl &> /dev/null && ! command -v x86_64-w64-mingw32-g++ &> /dev/null; then
        echo "Error: No compatible compiler found. Please install Visual Studio or MinGW-w64."
        exit 1
    fi
else
    # Linux dependencies
    for dep in git cmake g++ make pkg-config; do
        if ! command -v $dep &> /dev/null; then
            echo "Error: $dep is required but not installed. Please install it first."
            exit 1
        fi
    done

    # Check for system dependencies needed by OpenCL-ICD-Loader
    if ! pkg-config --exists libudev; then
        echo "Error: libudev-dev is required but not installed. Please install it first."
        exit 1
    fi
fi

# Clone the repository with submodules
REPO_URL="https://github.com/KhronosGroup/OpenCL-SDK.git"
TEMP_CLONE_DIR=$(mktemp -d)
echo "Cloning OpenCL-SDK repository with submodules..."
git clone --recursive "$REPO_URL" "$TEMP_CLONE_DIR"

# Create build directory
BUILD_DIR=$(mktemp -d)
echo "Building OpenCL SDK in: $BUILD_DIR"

# Configure and build
cd "$BUILD_DIR"

# Set up CMake options based on target OS
CMAKE_OPTIONS=(
    "-DCMAKE_INSTALL_PREFIX=$INSTALL_PATH"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DBUILD_DOCS=OFF"
    "-DBUILD_EXAMPLES=OFF"
    "-DBUILD_TESTING=OFF"
	"-DOPENCL_SDK_BUILD_CLINFO=OFF"
	"-DOPENCL_SDK_BUILD_SAMPLES=OFF"
)

if [[ "$TARGET_OS" == "windows" ]]; then
    # Windows-specific options
    if command -v x86_64-w64-mingw32-g++ &> /dev/null; then
        # Use MinGW cross-compilation
        CMAKE_OPTIONS+=(
            "-DCMAKE_SYSTEM_NAME=Windows"
            "-DCMAKE_C_COMPILER=x86_64-w64-mingw32-gcc"
            "-DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++"
            "-DCMAKE_RC_COMPILER=x86_64-w64-mingw32-windres"
        )
    fi
    # For Visual Studio, let CMake auto-detect
fi

cmake "${CMAKE_OPTIONS[@]}" "$TEMP_CLONE_DIR"

echo "Building with $BUILD_THREADS threads..."
cmake --build . --config Release --parallel "$BUILD_THREADS"

echo "Installing to $INSTALL_PATH..."
cmake --install . --config Release

# Clean up temporary directories
rm -rf "$TEMP_CLONE_DIR" "$BUILD_DIR"

# Set environment variables
echo "Setting environment variables..."

# Determine shell profile based on OS and shell
if [[ "$TARGET_OS" == "windows" ]]; then
    # Windows environment variables (for Git Bash, MSYS2, etc.)
    SHELL_PROFILE="${HOME}/.bash_profile"
    [[ -f "${HOME}/.bashrc" ]] && SHELL_PROFILE="${HOME}/.bashrc"
    
    # Add to PATH for Windows
    cat << EOF >> "$SHELL_PROFILE"

# OpenCL SDK configuration for Windows
export OPENCL_SDK_ROOT="$INSTALL_PATH"
export OpenCL_INCLUDE_DIR="\$OPENCL_SDK_ROOT/include"
export OpenCL_LIBRARY_DIR="\$OPENCL_SDK_ROOT/lib"
export PATH="\$OPENCL_SDK_ROOT/bin:\$OPENCL_SDK_ROOT/lib:\$PATH"
EOF
else
    # Linux environment variables
    SHELL_PROFILE="${HOME}/.bashrc"
    [[ -f "${HOME}/.zshrc" ]] && SHELL_PROFILE="${HOME}/.zshrc"
    
    # Handle case where LD_LIBRARY_PATH might not be set
    LD_LIBRARY_PATH_FALLBACK=""
    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        LD_LIBRARY_PATH_FALLBACK=":\$LD_LIBRARY_PATH"
    fi

    cat << EOF >> "$SHELL_PROFILE"

# OpenCL SDK configuration for Linux
export OPENCL_SDK_ROOT="$INSTALL_PATH"
export OpenCL_INCLUDE_DIR="\$OPENCL_SDK_ROOT/include"
export OpenCL_LIBRARY_DIR="\$OPENCL_SDK_ROOT/lib"
export PATH="\$OPENCL_SDK_ROOT/bin:\$PATH"
export LD_LIBRARY_PATH="\$OPENCL_SDK_ROOT/lib${LD_LIBRARY_PATH_FALLBACK}"
EOF
fi

# Set current session variables
export OPENCL_SDK_ROOT="$INSTALL_PATH"
export OpenCL_INCLUDE_DIR="$INSTALL_PATH/include"
export OpenCL_LIBRARY_DIR="$INSTALL_PATH/lib"
export PATH="$INSTALL_PATH/bin:$PATH"

if [[ "$TARGET_OS" != "windows" ]]; then
    # Handle LD_LIBRARY_PATH for current session (Linux only)
    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        export LD_LIBRARY_PATH="$INSTALL_PATH/lib:$LD_LIBRARY_PATH"
    else
        export LD_LIBRARY_PATH="$INSTALL_PATH/lib"
    fi
fi

# Verify installation
echo -e "\n=== Verification ==="
checks=(
    "SDK Root:$INSTALL_PATH:directory"
    "Include Directory:$INSTALL_PATH/include:directory"
    "Lib Directory:$INSTALL_PATH/lib:directory"
    "Bin Directory:$INSTALL_PATH/bin:directory"
)

all_good=true
for check in "${checks[@]}"; do
    IFS=':' read -r name path type <<< "$check"
    if [[ -e "$path" ]]; then
        echo "[OK] $name: $path"
    else
        echo "[FAIL] $name: $path"
        all_good=false
    fi
done

if $all_good; then
    echo -e "\n[SUCCESS] OpenCL SDK built and installed successfully!"
    echo "Environment variables added to $SHELL_PROFILE"
    echo "Permanent variables will be available in new shells."

    echo -e "\n=== Current Session Variables ==="
    echo "OPENCL_SDK_ROOT: $OPENCL_SDK_ROOT"
    echo "OpenCL_INCLUDE_DIR: $OpenCL_INCLUDE_DIR"
    echo "OpenCL_LIBRARY_DIR: $OpenCL_LIBRARY_DIR"
    if [[ "$TARGET_OS" != "windows" ]]; then
        echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    fi
else
    echo -e "\n[WARNING] Installation completed with some missing components."
fi

echo -e "\n=== Usage ==="
echo "To use in CMake, add these flags:"
echo "  -DUSE_OPENCL=ON"
echo "  -DOpenCL_INCLUDE_DIR=\"$OpenCL_INCLUDE_DIR\""
echo "  -DOpenCL_LIBRARY=\"$OpenCL_LIBRARY_DIR\""