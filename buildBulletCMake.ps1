# Script to build Bullet on a Windows system with OpenCL support

# Store the original directory
$originalDirectory = Get-Location

try {
    # Check if OpenCL environment variables are set, if not set default values
    if (-not $env:OPENCL_SDK_ROOT) {
        $env:OPENCL_SDK_ROOT = "C:\OpenCL-SDK"
    }
    if (-not $env:OpenCL_INCLUDE_DIR) {
        $env:OpenCL_INCLUDE_DIR = "$env:OPENCL_SDK_ROOT\include\CL"
    }
    if (-not $env:OpenCL_LIBRARY_DIR) {
        $env:OpenCL_LIBRARY_DIR = "$env:OPENCL_SDK_ROOT\lib"
    }

    # Verify OpenCL SDK exists
    if (-not (Test-Path $env:OpenCL_INCLUDE_DIR)) {
        Write-Error "OpenCL include directory not found: $env:OpenCL_INCLUDE_DIR"
        Write-Error "Please install OpenCL SDK first or set OPENCL_SDK_ROOT environment variable"
        exit 1
    }

    $MACH="x64"
    $BULLETDIR="bullet3"
    $BUILDDIR="bullet-build"

    # Get absolute paths
    $bulletAbsPath = Join-Path $originalDirectory $BULLETDIR
    $buildAbsPath = Join-Path $bulletAbsPath $BUILDDIR

    # Clean previous build if it exists
    if (Test-Path $buildAbsPath) {
        Write-Host "=== Cleaning previous build directory"
        Remove-Item $buildAbsPath -Recurse -Force
    }

    # Create build directory
    New-Item -ItemType Directory -Path $buildAbsPath -Force

    # Verify CMake is available
    try {
        cmake --version | Out-Null
    } catch {
        Write-Error "CMake is not installed or not in PATH. Please install CMake first."
        exit 1
    }
	
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


    Write-Host "=== Building Bullet in dir $bulletAbsPath for arch $MACH into $buildAbsPath with OpenCL support"
    Write-Host "Using OpenCL SDK at: $env:OPENCL_SDK_ROOT"

    # Run CMake from the build directory with proper source path
    Set-Location $buildAbsPath
	
	# Modify your CMake command to use these settings instead:
	$cmakeArgs = @(
		"-G", "Visual Studio 17 2022",
		"-A", $MACH,
		"-DBUILD_SHARED_LIBS=OFF",
		"-DBUILD_BULLET2_DEMOS=OFF",
		"-DBUILD_EXTRas=ON",
		"-DBUILD_BULLET3=ON",
		"-DUSE_OPENMP=ON",
		"-DUSE_OPENCL=ON",
		"-DBT_USE_PROFILE=ON",
		"-DBULLET2_MULTITHREADING=ON",
		"-DBULLET2_USE_OPEN_MP_MULTITHREADING=ON",
		"-DBT_THREADSAFE=ON",
		"-DBT_USE_OPENMP=ON",
		"-DBUILD_OPENMP=ON ",
		"-DBUILD_UNIT_TESTS=OFF",
		"-DBUILD_BULLET_ROBOTICS_EXTRA=ON",
		"-DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF",
		"-DBUILD_HACD_EXTRA=ON",
		"-DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON",
		"-DBUILD_SERIALIZE_EXTRA=ON",
		"-DCMAKE_BUILD_TYPE=Release",
		"-DBUILD_INVERSE_DYNAMIC_EXTRA=ON",
		"-DBUILD_OBJ2SDF_EXTRA=OFF",
		"-DBUILD_GIMPACTUTILS_EXTRA=OFF",
		"-DBUILD_CPU_DEMOS=OFF",
		"-DBUILD_ENET=OFF",
		"-DBUILD_PYBULLET=OFF",
		"-DINSTALL_EXTRA_LIBS=ON",
		"-DINSTALL_LIBS=ON",
		"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
		"-DCMAKE_CXX_FLAGS=/EHsc /DBT_XML_SUPPORT /O2 $compilerFlag /fp:fast /Gd -UBT_USE_DOUBLE_PRECISION /openmp:static",
		"-DCMAKE_C_FLAGS=/openmp:static",
		".."
	)

    # Run CMake with properly formatted arguments
    & cmake $cmakeArgs

    # Check if CMake succeeded
    if ($LASTEXITCODE -ne 0) {
        Write-Error "CMake configuration failed with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }

    # Find the solution file that CMake actually generated
    $solutionFile = Get-ChildItem -Path $buildAbsPath -Filter "*.sln" | Select-Object -First 1
    
    if (-not $solutionFile) {
        Write-Error "No solution file found in build directory: $buildAbsPath"
        exit 1
    }

    Write-Host "=== Found solution file: $($solutionFile.Name)"

    # Create the lib/Release directory that the linker expects
    $libReleaseDir = Join-Path $buildAbsPath "lib\Release"
    New-Item -ItemType Directory -Path $libReleaseDir -Force | Out-Null

    # Build all projects at once using the solution file
    Write-Host "=== Building all projects using the solution file"
    msbuild -p:Configuration=Release -p:Platform=x64 -maxCpuCount $solutionFile.FullName
    
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Failed to build solution with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }

    # Return to original directory for file copying
    Set-Location $originalDirectory

    # Create target directories
    $libAbsPath = Join-Path $originalDirectory "lib"
    $includeAbsPath = Join-Path $originalDirectory "include"
    
    New-Item -ItemType Directory -Path $libAbsPath -Force
    New-Item -ItemType Directory -Path $includeAbsPath -Force

    # Copy the .lib files into the target lib directory
    Write-Host "=== Copy .lib files into the lib dir"
    Get-ChildItem -Path $buildAbsPath -Include *.lib -Recurse | ForEach-Object {
        Write-Host "----- copying $_ to $libAbsPath"
        Copy-Item $_ -Destination $libAbsPath -Force
    }

    # Also copy any PDB files for debugging
    Write-Host "=== Copy .pdb files into the lib dir"
    Get-ChildItem -Path $buildAbsPath -Include *.pdb -Recurse | ForEach-Object {
        Write-Host "----- copying $_ to $libAbsPath"
        Copy-Item $_ -Destination $libAbsPath -Force
    }

	# Copy the .h and .inl files into the target include directory
	Write-Host "=== Copy .h files into the include dir"
	Get-ChildItem -Path "$bulletAbsPath\src" -Include *.h,*.inl -Recurse | ForEach-Object {
		# Remove the "src\" part from the path
		$relativePath = $_.FullName.Substring($bulletAbsPath.Length + 5)  # +5 to skip "src\"
		$destPath = Join-Path $includeAbsPath $relativePath
		$destDir = Split-Path -Parent $destPath
		
		if (-not (Test-Path $destDir -PathType Container)) {
			Write-Host "----- creating directory $destDir"
			New-Item -ItemType Directory -Path $destDir -Force | Out-Null
		}
		
		Write-Host "----- copying $_ to $destPath"
		Copy-Item $_ -Destination $destPath -Force
	}

    # Copy the .h files from Extras into the target include directory
    Write-Host "=== Copy Extras .h files into the include dir"
    Get-ChildItem -Path "$bulletAbsPath\Extras" -Include *.h,*.inl -Recurse | ForEach-Object {
        $relativePath = $_.FullName.Substring($bulletAbsPath.Length + 1)
        $destPath = Join-Path $includeAbsPath $relativePath
        $destDir = Split-Path -Parent $destPath
        
        if (-not (Test-Path $destDir -PathType Container)) {
            Write-Host "----- creating Extras directory $destDir"
            New-Item -ItemType Directory -Path $destDir -Force | Out-Null
        }
        
        Write-Host "----- copying Extras $_ to $destPath"
        Copy-Item $_ -Destination $destPath -Force
    }

    # Copy Bullet's VERSION file into lib/ so BulletSim can reference it
    $versionFile = Join-Path $bulletAbsPath "VERSION"
    if (Test-Path $versionFile) {
        Copy-Item -Path $versionFile -Destination $libAbsPath -Force
        Write-Host "=== Copied VERSION file to $libAbsPath"
    } else {
        Write-Warning "VERSION file not found in $bulletAbsPath directory"
    }

	# Check for OpenMP support in the built libraries
	Write-Host "=== Verifying OpenMP support in built libraries..."
	$librariesToCheck = @("LinearMath.lib", "BulletCollision.lib", "BulletDynamics.lib")

	foreach ($libName in $librariesToCheck) {
		$libPath = Join-Path $libAbsPath $libName
		
		if (Test-Path $libPath) {
			Write-Host "? Checking $libName for OpenMP support..."
			
			# Check for omp_ symbols
			$ompSymbols = dumpbin /symbols $libPath | Select-String "omp_"
			if ($ompSymbols) {
				Write-Host "  Found OpenMP symbols in ${libName}:"
				$ompSymbols | Select-Object -First 3 | ForEach-Object { Write-Host "    - $_" }
				if ($ompSymbols.Count -gt 3) {
					Write-Host "    ... and $($ompSymbols.Count - 3) more OpenMP symbols"
				}
			} else {
				Write-Warning "  No OpenMP symbols found in $libName"
			}
			
			# Check for vcomp symbols specifically
			$vcompSymbols = dumpbin /symbols $libPath | Select-String "_vcomp"
			if ($vcompSymbols) {
				Write-Host "  Found vcomp symbols in ${libName}:"
				$vcompSymbols | Select-Object -First 3 | ForEach-Object { Write-Host "    - $_" }
				if ($vcompSymbols.Count -gt 3) {
					Write-Host "    ... and $($vcompSymbols.Count - 3) more vcomp symbols"
				}
			} else {
				Write-Warning "  No vcomp symbols found in $libName"
			}
			
			# Check if symbols are undefined (which is expected for static libraries)
			$undefinedSymbols = dumpbin /symbols $libPath | Select-String "UNDEF"
			$undefinedOmpSymbols = $undefinedSymbols | Select-String "omp_"
			if ($undefinedOmpSymbols) {
				Write-Host "  Note: OpenMP symbols are undefined (expected for static libraries):"
				$undefinedOmpSymbols | Select-Object -First 3 | ForEach-Object { Write-Host "    - $_" }
				if ($undefinedOmpSymbols.Count -gt 3) {
					Write-Host "    ... and $($undefinedOmpSymbols.Count - 3) more undefined OpenMP symbols"
				}
			}
		} else {
			Write-Warning "$libName not found - cannot verify OpenMP support"
		}
		Write-Host ""
	}

    Write-Host "=== Bullet build completed successfully!"
    Write-Host "Libraries are in: $libAbsPath"
    Write-Host "Headers are in: $includeAbsPath"
    Write-Host ""
    Write-Host "NOTE: The OpenMP symbols will show as undefined in the static libraries."
    Write-Host "This is normal behavior for static libraries. When you link your application"
    Write-Host "with these libraries, you need to also link with vcomp.lib (OpenMP library)."
    Write-Host ""
    Write-Host "Add this to your application's linker options: /DEFAULTLIB:vcomp.lib"
}
finally {
    # Always return to the original directory, regardless of success or failure
    Set-Location $originalDirectory
}