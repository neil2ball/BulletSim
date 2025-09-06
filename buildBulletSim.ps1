#! /usr/bin/env pwsh
# Script to build BulletSim Windows binaries using locally built Bullet libraries
param(
    [string]$BLIBDIR = "./lib",
    [string]$BINCLUDEDIR = "./include",
    [string]$TARGETBASE = "libBulletSim-3.27"
)

do {
    Write-Host "Select the architecture level to build for:"
    Write-Host "1. x86-64-v1 (SSE2)"
    Write-Host "2. x86-64-v2 (AVX)"
    Write-Host "3. x86-64-v3 (AVX2)"
    Write-Host "4. x86-64-v4 (AVX-512)"

    $choice = Read-Host "Enter your choice (1-4)"
    switch ($choice) {
        "1" { $arch = "x86-64-v1"; $compilerFlag = "/arch:SSE2"; $valid = $true }
        "2" { $arch = "x86-64-v2"; $compilerFlag = "/arch:AVX";  $valid = $true }
        "3" { $arch = "x86-64-v3"; $compilerFlag = "/arch:AVX2"; $valid = $true }
        "4" { $arch = "x86-64-v4"; $compilerFlag = "/arch:AVX512"; $valid = $true }
        default {
            Write-Host "Invalid selection. Please try again."
            $valid = $false
        }
    }
} until ($valid)

Write-Host "Selected architecture: $arch"
Write-Host "MSVC compiler flag: $compilerFlag"


$BASE = Get-Location

# Verify locally built Bullet include directory exists
if (-not (Test-Path $BINCLUDEDIR -PathType Container)) {
    Write-Error "Bullet include directory $BINCLUDEDIR not found!"
    Write-Error "Please build Bullet first using buildBulletCMake.ps1"
    exit 1
}

# Verify locally built Bullet library directory exists
if (-not (Test-Path $BLIBDIR -PathType Container)) {
    Write-Error "Bullet library directory $BLIBDIR not found!"
    Write-Error "Please build Bullet first using buildBulletCMake.ps1"
    exit 1
}

# Output file is ${TARGETBASE}-${BUILDDATE}-${ARCH}.dll
$ARCH = "x64"
$BUILDDATE = Get-Date -Format "yyyyMMdd"
$TARGET = "${TARGETBase}-${BUILDDATE}-${ARCH}.dll"

# Version of the Bullet engine that is being linked
if (Test-Path "$BLIBDIR\VERSION") {
    $BULLETVERSION = Get-Content "$BLIBDIR\VERSION" -Raw
    $BULLETVERSION = $BULLETVERSION.Trim()
} else {
    $BULLETVERSION = "3.27"
    Write-Warning "$BLIBDIR\VERSION not found, using '$BULLETVERSION'"
}

# Version of the BulletSim glue that is being built and included
if (Test-Path "VERSION") {
    $BULLETSIMVERSION = Get-Content "VERSION" -Raw
    $BULLETSIMVERSION = $BULLETSIMVERSION.Trim()
} else {
    Write-Error "VERSION file not found in current directory!"
    exit 1
}

# Pass version information into compilations as C++ variables
$VERSIONCFLAGS = "/D BULLETVERSION=`"$BULLETVERSION`" /D BULLETSIMVERSION=`"$BULLETSIMVERSION`""

# Compiler flags
$CFLAGS = "/I`"$BINCLUDEDIR`" /I. /Zi /EHsc /W3 /nologo /LD /MT /O2 $compilerFlag /fp:fast /openmp:static /Gd $VERSIONCFLAGS /D BULLETSIM_EXPORTS /U BT_USE_DOUBLE_PRECISION /D BT_USE_OPENCL /D B3_USE_CLEW /D BT_USE_PROFILE /D USE_OPENMP"

$EXTRAS_INCLUDE_PATH = "$BINCLUDEDIR\Extras"
if (Test-Path $EXTRAS_INCLUDE_PATH -PathType Container) {
    $CFLAGS = "/I`"$EXTRAS_INCLUDE_PATH`" $CFLAGS"
    Write-Host "? Added Extras include path: $EXTRAS_INCLUDE_PATH"
}

# Linker flags - ADD OPENMP LINKING HERE
$LFLAGS = "/DLL /OUT:`"$TARGET`" /NOLOGO /DEBUG /LIBPATH:`"$BLIBDIR`" /DEFAULTLIB:vcomp.lib"

# Function to verify a library exists
function Verify-Library {
    param($lib_name)
    
    $lib_path_lib = "$BLIBDIR\$lib_name.lib"
    
    if (-not (Test-Path $lib_path_lib)) {
        Write-Error "Library $lib_name.lib not found in $BLIBDIR!"
        return $false
    }
    return $true
}

# Verify all required Bullet libraries exist (from local build)
$REQUIRED_BULLET_LIBS = @(
    "BulletDynamics",
    "BulletCollision", 
    "LinearMath",
    "BulletXmlWorldImporter",
    "Bullet3OpenCL_clew",      
    "Bullet3Dynamics",         
    "Bullet3Collision",          
    "Bullet3Geometry",         
    "Bullet3Common",
    "BulletSoftBody",
	"BulletFileLoader",
    "BulletWorldImporter"
)

# Optional libraries (warn if missing but continue)
$OPTIONAL_BULLET_LIBS = @(
    "HACD"
    "Bullet3Common"
    "BulletInverseDynamicsUtils"
)

Write-Host "=== Verifying locally built Bullet libraries in $BLIBDIR"

foreach ($lib in $REQUIRED_BULLET_LIBS) {
    if (-not (Verify-Library $lib)) {
        Write-Error "Required library $lib.lib missing! Rebuild Bullet with buildBulletCMake.ps1"
        exit 1
    }
    Write-Host "? Found: $lib.lib"
}

foreach ($lib in $OPTIONAL_BULLET_LIBS) {
    if (Verify-Library $lib) {
        Write-Host "? Found: $lib.lib (optional)"
    } else {
        Write-Warning "$lib.lib not found (optional)"
    }
}

# Build library list for linking
$BULLETLIBS = @()
foreach ($lib in $REQUIRED_BULLET_LIBS + $OPTIONAL_BULLET_LIBS) {
    $lib_path = "$lib.lib"
    if (Test-Path "$BLIBDIR\$lib_path") {
		# Put Bullet3OpenCL_clew at the end of the list
        if ($lib -eq "Bullet3OpenCL_clew") {
            continue
        }
        $BULLETLIBS += "$lib_path"
        Write-Host "? Using library: $lib_path"
    }
}

if (Test-Path "$BLIBDIR\Bullet3OpenCL_clew.lib") {
    $BULLETLIBS += "Bullet3OpenCL_clew.lib"
    Write-Host "? Using library: Bullet3OpenCL_clew.lib"
}

# Verify OpenCL library exists (optional)
Write-Host "=== Checking for OpenCL support"
$openclLib = "OpenCL.lib"
$openclFound = $false

# Check common OpenCL locations
$openclPaths = @(
    "C:\OpenCL-SDK\lib\$openclLib",
    "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v*\lib\x64\$openclLib",
    "C:\Program Files (x86)\AMD APP SDK\*\lib\x86_64\$openclLib"
)

foreach ($path in $openclPaths) {
    if (Test-Path $path) {
        $OPENCL_LIB = $path
        $openclFound = $true
        Write-Host "? Found OpenCL library: $OPENCL_LIB"
        break
    }
}

# Replace the OpenCL include section with this:
$openclIncludePath = "C:\OpenCL-SDK\include"
if (Test-Path "$openclIncludePath\CL\cl.h") {
    $CFLAGS = "/I`"$openclIncludePath`" $CFLAGS"
    $CFLAGS += " /D BT_USE_OPENCL"
    Write-Host "? Added OpenCL include path: $openclIncludePath"
} elseif (Test-Path "$openclIncludePath\cl.h") {
    # Some OpenCL SDKs put cl.h directly in include/
    $CFLAGS = "/I`"$openclIncludePath`" $CFLAGS"
    $CFLAGS += " /D BT_USE_OPENCL"
    Write-Host "? Added OpenCL include path: $openclIncludePath"
} else {
    # Try to discover the actual OpenCL include path
    $discoveredPath = $null
    $possiblePaths = @(
        "C:\OpenCL-SDK\include",
        "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v*\include",
        "C:\Program Files (x86)\AMD APP SDK\*\include",
        "${env:ProgramFiles}\NVIDIA Corporation\OpenCL\include",
        "${env:ProgramFiles(x86)}\AMD APP SDK\*\include"
    )
    
    foreach ($path in $possiblePaths) {
        $resolvedPath = Resolve-Path $path -ErrorAction SilentlyContinue
        if ($resolvedPath) {
            if (Test-Path "$resolvedPath\CL\cl.h") {
                $discoveredPath = $resolvedPath
                break
            } elseif (Test-Path "$resolvedPath\cl.h") {
                $discoveredPath = $resolvedPath
                break
            }
        }
    }
    
    if ($discoveredPath) {
        $CFLAGS = "/I`"$discoveredPath`" $CFLAGS"
        $CFLAGS += " /D BT_USE_OPENCL"
        Write-Host "? Discovered OpenCL include path: $discoveredPath"
    } else {
        Write-Warning "OpenCL headers not found. Disabling OpenCL support."
        $CFLAGS += " /D NO_OPENCL"
    }
}

# Verify source files exist
if (-not (Test-Path "API2.cpp")) {
    Write-Error "API2.cpp not found!"
    exit 1
}

if (-not (Test-Path "BulletSim.cpp")) {
    Write-Error "BulletSim.cpp not found!"
    exit 1
}

# Compile source files
Write-Host "=== Compiling source files"
Write-Host "Using include directory: $BINCLUDEDIR"
Write-Host "Using library directory: $BLIBDIR"

& cl @($CFLAGS -split ' ') /c API2.cpp
if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to compile API2.cpp"
    exit 1
}

& cl @($CFLAGS -split ' ') /c BulletSim.cpp
if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to compile BulletSim.cpp"
    exit 1
}

# Link the DLL
Write-Host "=== Building target $TARGET"
Write-Host "Linking with locally built Bullet libraries"

# Build the link arguments properly as an array
$linkArgs = @(
    "/DLL",
    "/OUT:$TARGET",
    "/NOLOGO",
    "/DEBUG",
    "/LIBPATH:$BLIBDIR",
    "/DEFAULTLIB:vcomp.lib",  # ADD OPENMP LINKING
    "API2.obj",
    "BulletSim.obj"
) + $BULLETLIBS

# Add OpenMP library explicitly
$linkArgs += "vcomp.lib"

if ($openclFound) {
    $linkArgs += $OPENCL_LIB
}

# Use proper argument passing with splatting
& link @linkArgs

# Check if build was successful
if ($LASTEXITCODE -eq 0) {
    Write-Host "=== Build successful: $TARGET"
    
    # Verify the DLL was created
    if (Test-Path $TARGET) {
        Write-Host "? DLL created successfully"
        
        # Verify BSLog symbol using dumpbin
        Write-Host "=== Verifying WorldData::BSLog symbol is present..."
        $symbols = dumpbin /exports $TARGET | Select-String "BSLog"
        if ($symbols) {
            Write-Host "? Found BSLog symbol: $($symbols[0])"
        } else {
            Write-Warning "BSLog symbol not found in exports - may cause runtime errors"
        }
		
		# Check for OpenMP static linking: we should not import OpenMP functions from a DLL
		Write-Host "=== Verifying OpenMP static linking..."
		$imports = dumpbin /imports $TARGET
		$openmpImports = $imports | Select-String "vcomp"
		if ($openmpImports) {
			Write-Warning "OpenMP dynamic linking detected - imports: $openmpImports"
		} else {
			Write-Host "? OpenMP is statically linked (no imports of vcomp.dll)"
		}
        
    } else {
        Write-Error "DLL was not created despite successful link"
        exit 1
    }
} else {
    Write-Error "=== Build failed with exit code $LASTEXITCODE!"
    Write-Error "=== Common issues:"
    Write-Error "    1. Bullet not built properly - run buildBulletCMake.ps1 again"
    Write-Error "    2. Visual Studio build tools not properly installed"
    Write-Error "    3. Missing source files in current directory"
    exit 1
}

# Clean up object files
Remove-Item "API2.obj" -ErrorAction SilentlyContinue
Remove-Item "BulletSim.obj" -ErrorAction SilentlyContinue

Write-Host "=== BulletSim build completed successfully!"
Write-Host "Output: $(Resolve-Path $TARGET)"