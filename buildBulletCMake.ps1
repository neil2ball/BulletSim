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
		"-DBUILD_EXTRAS=ON",
		"-DBUILD_BULLET3=ON",
		"-DUSE_OPENMP=ON",
		"-DUSE_OPENCL=ON",
		"-DBT_USE_PROFILE=ON",
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
		"-DBUILD_BULLET3_COLLISION=ON",
		"-DBUILD_BULLET3_DYNAMICS=ON",
		"-DBUILD_BULLET3_GEOMETRY=ON",
		"-DCMAKE_CXX_FLAGS=/EHsc /DBT_XML_SUPPORT /O2 $compilerFlag /fp:fast /Gd -UBT_USE_DOUBLE_PRECISION /openmp:experimental",
		"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=`"$buildAbsPath/lib/Release`"",
		"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE=`"$buildAbsPath/lib/Release`"",
		"-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE=`"$buildAbsPath/lib/Release`"",
		"-DCMAKE_EXE_LINKER_FLAGS_RELEASE=`"/INCREMENTAL:NO`"",
		"-DCMAKE_SHARED_LINKER_FLAGS_RELEASE=`"/INCREMENTAL:NO`"",
		"-DCMAKE_MODULE_LINKER_FLAGS_RELEASE=`"/INCREMENTAL:NO`"",
		".."
	)

    # Run CMake with properly formatted arguments
    & cmake $cmakeArgs

    # Check if CMake succeeded
    if ($LASTEXITCODE -ne 0) {
        Write-Error "CMake configuration failed with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }

    # Create the lib/Release directory that the linker expects
    $libReleaseDir = Join-Path $buildAbsPath "lib\Release"
    New-Item -ItemType Directory -Path $libReleaseDir -Force | Out-Null

	# Build projects in correct order to resolve dependencies
	$buildOrder = @(
		@{Name="LinearMath"; Path="src\LinearMath\LinearMath.vcxproj"},
		@{Name="Bullet3Common"; Path="src\Bullet3Common\Bullet3Common.vcxproj"}, 
		@{Name="BulletCollision"; Path="src\BulletCollision\BulletCollision.vcxproj"},
		@{Name="BulletDynamics"; Path="src\BulletDynamics\BulletDynamics.vcxproj"},
		@{Name="BulletSoftBody"; Path="src\BulletSoftBody\BulletSoftBody.vcxproj"},
		@{Name="BulletInverseDynamics"; Path="Extras\InverseDynamics\BulletInverseDynamicsUtils.vcxproj"},
		@{Name="HACD"; Path="Extras\HACD\HACD.vcxproj"},
		@{Name="Bullet3OpenCL_clew"; Path="src\Bullet3OpenCL\Bullet3OpenCL_clew.vcxproj"},
		@{Name="Bullet3Dynamics"; Path="src\Bullet3Dynamics\Bullet3Dynamics.vcxproj"},
		@{Name="Bullet3Collision"; Path="src\Bullet3Collision\Bullet3Collision.vcxproj"},
		@{Name="Bullet3Geometry"; Path="src\Bullet3Geometry\Bullet3Geometry.vcxproj"},
		@{Name="BulletSoftBody"; Path="src\BulletSoftBody\BulletSoftBody.vcxproj"},
		@{Name="BulletFileLoader"; Path="Extras\Serialize\BulletFileLoader\BulletFileLoader.vcxproj"},
		@{Name="BulletWorldImporter"; Path="Extras\Serialize\BulletWorldImporter\BulletWorldImporter.vcxproj"},
		@{Name="BulletXmlWorldImporter"; Path="Extras\Serialize\BulletXmlWorldImporter\BulletXmlWorldImporter.vcxproj"}
	)

	foreach ($item in $buildOrder) {
		if ($item -is [hashtable]) {
			# Handle hashtable items
			$projectName = $item.Name
			$projectFullPath = Join-Path $buildAbsPath $item.Path
		} else {
			# Handle string items (original logic)
			$projectName = $item
			$projectPath = "src\$item\$item.vcxproj"
			$projectFullPath = Join-Path $buildAbsPath $projectPath
		}
		
		if (Test-Path $projectFullPath) {
			Write-Host "=== Building $projectName"
			
			# Build the project
			msbuild -p:Configuration=Release -p:LinkIncremental=false -maxCpuCount:1 $projectFullPath
			
			if ($LASTEXITCODE -ne 0) {
				Write-Error "Failed to build $projectName with exit code $LASTEXITCODE"
				exit $LASTEXITCODE
			}
			
			# Find and copy the built library
			$libName = "$projectName.lib"
			if ($item -is [hashtable]) {
				# For hashtable projects, use their specific directory structure
				$baseDir = Split-Path $item.Path -Parent
				$sourcePaths = @(
					"$baseDir\Release\$libName",
					"$baseDir\x64\Release\$libName", 
					"Release\$libName",
					"x64\Release\$libName"
				)
			} else {
				# For standard projects, use original logic
				$sourcePaths = @(
					"src\$projectName\Release\$libName",
					"src\$projectName\x64\Release\$libName", 
					"Release\$libName",
					"x64\Release\$libName"
				)
			}
			
			foreach ($sourceRelPath in $sourcePaths) {
				$sourceFullPath = Join-Path $buildAbsPath $sourceRelPath
				if (Test-Path $sourceFullPath) {
					Write-Host "=== Copying $libName to $libReleaseDir"
					Copy-Item $sourceFullPath -Destination $libReleaseDir -Force
					break
				}
			}
		} else {
			Write-Warning "Project not found: $projectFullPath"
		}
	}

    # Build Serialize extra projects
    $serializeProjects = @(
        "BulletFileLoader",
        "BulletWorldImporter",
        "BulletXmlWorldImporter"
    )

    foreach ($project in $serializeProjects) {
        $projectPath = "Extras\Serialize\$project\$project.vcxproj"
        $projectFullPath = Join-Path $buildAbsPath $projectPath
        
        if (Test-Path $projectFullPath) {
            Write-Host "=== Building $project"
            
            # Build the project with incremental linking disabled
            msbuild -p:Configuration=Release -p:LinkIncremental=false -maxCpuCount:1 $projectFullPath
            
            if ($LASTEXITCODE -ne 0) {
                Write-Error "Failed to build $project with exit code $LASTEXITCODE"
                exit $LASTEXITCODE
            }
            
            # Find and copy the built library to where the linker expects it
            $libName = "$project.lib"
            $sourcePaths = @(
                "Extras\Serialize\$project\Release\$libName",
                "Extras\Serialize\$project\x64\Release\$libName", 
                "Release\$libName",
                "x64\Release\$libName"
            )
            
            foreach ($sourceRelPath in $sourcePaths) {
                $sourceFullPath = Join-Path $buildAbsPath $sourceRelPath
                if (Test-Path $sourceFullPath) {
                    Write-Host "=== Copying $libName to $libReleaseDir"
                    Copy-Item $sourceFullPath -Destination $libReleaseDir -Force
                    break
                }
            }
        } else {
            Write-Warning "Project not found: $projectFullPath"
        }
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
    $libReleaseDir = Join-Path $buildAbsPath "lib\Release"
    if (Test-Path $libReleaseDir) {
        Get-ChildItem -Path $libReleaseDir -Include *.lib -Recurse | ForEach-Object {
            Write-Host "----- copying $_ to $libAbsPath"
            Copy-Item $_ -Destination $libAbsPath -Force
        }
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

    Write-Host "=== Bullet build completed successfully!"
    Write-Host "Libraries are in: $libAbsPath"
    Write-Host "Headers are in: $includeAbsPath"
}
finally {
    # Always return to the original directory, regardless of success or failure
    Set-Location $originalDirectory
}