#! /usr/bin/env pwsh
# Script to fetch the Bullet Physics Engine sources, build same
#    using the 'buildBulletSMake.ps1' script, and then build BulletSim
#    .dll's using 'buildBulletSim.ps1'.
# This captures the steps needed and will be replaced by better scripts
#    and Github actions.

# PROPERLY set up x64 Visual Studio environment
$vcvarsPath = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

if (Test-Path $vcvarsPath) {
    # Create a temporary script to capture the environment
    $tempScript = [System.IO.Path]::GetTempFileName() + ".cmd"
    "@echo off`n" + 
    "call `"$vcvarsPath`"`n" +
    "set" | Out-File -Encoding ascii $tempScript
    
    # Capture the environment from the x64 context
    $envVars = cmd /c "`"$tempScript`""
    Remove-Item $tempScript
    
    # Apply ONLY the Visual Studio build variables (not everything)
    $envVars | ForEach-Object {
        if ($_ -match "^(.*?)=(.*)$") {
            $name = $matches[1]
            $value = $matches[2]
            # Only set build-related variables, avoid overwriting PowerShell internals
            if ($name -like "*VS*" -or $name -like "*VC*" -or $name -like "*Windows*" -or 
                $name -like "*Path*" -or $name -like "*LIB*" -or $name -like "*INCLUDE*" -or
                $name -like "*Framework*" -or $name -like "*NET*" -or $name -like "*MS*") {
                [Environment]::SetEnvironmentVariable($name, $value, "Process")
            }
        }
    }
    
    Write-Host "? x64 Visual Studio environment configured"
    Write-Host "Architecture: $env:PROCESSOR_ARCHITECTURE"
    Write-Host "Visual Studio version: $(if (Test-Path env:VSCMD_VER) { $env:VSCMD_VER } else { 'unknown' })"
    
} else {
    Write-Error "vcvars64.bat not found at: $vcvarsPath"
    exit 1
}
# Set these values to 'yes' or 'no' to enable/disable fetching and building
$FETCHBULLETSOURCES = if ($env:FETCHBULLETSOURCES) { $env:FETCHBULLETSOURCES } else { "yes" }
$BUILDBULLET3 = if ($env:BUILDBULLET3) { $env:BUILDBULLET3 } else { "yes" }
$USEOPENCL = if ($env:USEOPENCL) { $env:USEOPENCL } else { "yes" }

# Note that Bullet3 sources are build in "bullet3/" and the
#     these are copied into "bullet2/" and checkouted to the version 2 sources.

$BASE = Get-Location

# Check for OpenCL SDK installation if OpenCL is enabled
if ($USEOPENCL -eq "yes") {
    Write-Host "=== Checking for OpenCL SDK installation"
    
    # Check if OpenCL SDK is available
    $openCLInstalled = $false
    
    # Check environment variables first
    if ($env:OPENCL_SDK_ROOT -and (Test-Path $env:OPENCL_SDK_ROOT)) {
        $openCLInstalled = $true
        Write-Host "OpenCL SDK found via environment variable: $env:OPENCL_SDK_ROOT"
    }
    # Check default installation path
    elseif (Test-Path "C:\OpenCL-SDK") {
        $openCLInstalled = $true
        Write-Host "OpenCL SDK found at default location: C:\OpenCL-SDK"
    }
    # Check if OpenCL headers are available in system paths
    #elseif (Test-Path "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v*\include\CL\cl.h") {
    #    $openCLInstalled = $true
    #    Write-Host "OpenCL found via NVIDIA CUDA installation"
    #}
    #elseif (Test-Path "C:\Program Files (x86)\AMD APP SDK\*\include\CL\cl.h") {
    #    $openCLInstalled = $true
    #    Write-Host "OpenCL found via AMD APP SDK installation"
    #}
    
    if (-not $openCLInstalled) {
        Write-Host "OpenCL SDK not found. Installing Khronos vendor-neutral OpenCL SDK..."
        
        # Check if install script exists
        $installScript = "installOpenCL-SDK.ps1"
        if (Test-Path $installScript) {
            & .\$installScript
            # The install script sets permanent environment variables
            # We don't need to set them here as they'll be available after restart
            Write-Host "OpenCL SDK installation completed. Please restart this script if build fails."
        } else {
            Write-Warning "OpenCL SDK install script '$installScript' not found. Continuing without OpenCL support."
            $USEOPENCL = "no"
        }
    } else {
        Write-Host "OpenCL SDK is available for build"
    }
}

if ($FETCHBULLETSOURCES -eq "yes") {
    Set-Location $BASE
    if (Test-Path "bullet3") {
        Remove-Item -Recurse -Force bullet3
    }

    Write-Host "=== Fetching Bullet Physics Engine sources into bullet3/"
    git clone https://github.com/bulletphysics/bullet3.git

    Write-Host "=== Applying BulletSim patches to bullet3"
    Set-Location $BASE
    Set-Location bullet3
	foreach ($file in (Get-Item "../000*")) {
		$filename = $file.Name
		$filePath = $file.FullName
		
		if ($filename -eq "0002-opencl-bullet3-onlyfloats.patch") {
			continue
		}
		
		Write-Host "Processing patch: $filename"
		git apply --ignore-whitespace $filePath
		if ($LASTEXITCODE -eq 0) {
			Write-Host "Applied patch: $filename"
		} else {
			Write-Host "Warning: Failed to apply patch: $filename" -ForegroundColor Yellow
		}
	}
	
	# APPLY THE OPENCL CONFIG DIRECTLY TO THE CMakeLists.txt
	$cmakeFile = "CMakeLists.txt"
	$content = Get-Content $cmakeFile -Raw

	# ADD THE OPENCL CONFIG RIGHT AFTER THE FIRST LINE
	$newContent = $content -replace '(cmake_minimum_required\(VERSION 3\.5\))', "`$1`n`nset(OPENCL_FOUND TRUE)`nset(OPENCL_INCLUDE_DIR `"C:/OpenCL-SDK/include/CL`")`nset(OPENCL_LIBRARY `"C:/OpenCL-SDK/lib/OpenCL.lib`")`nset(USE_DOUBLE_PRECISION OFF CACHE BOOL `"Use double precision`" FORCE)`nset(USE_OPENCL ON CACHE BOOL `"`" FORCE)`nset(BUILD_BULLET3 ON CACHE BOOL `"`" FORCE)`n"

	Set-Content -Path $cmakeFile -Value $newContent -NoNewline
	Write-Host "SUCCESS: Directly applied OpenCL config to CMakeLists.txt" -ForegroundColor Green
}

Set-Location $BASE

Write-Host "=== Setting version information variables"
$BuildDate = Get-Date -Format "yyyyMMdd"
$BulletSimVersion = Get-Content VERSION
$BulletSimGitVersion = git rev-parse HEAD
$BulletSimGitVersionShort = git rev-parse --short HEAD
Set-Location bullet3
$BulletVersion = Get-Content VERSION
$BulletGitVersion = git rev-parse HEAD
$BulletGitVersionShort = git rev-parse --short HEAD

Write-Host "=== Creating version information file"
Set-Location $BASE
if (Test-Path "BulletSimVersionInfo") {
    Remove-Item BulletSimVersionInfo
}
"BuildDate=$BuildDate" | Out-File BulletSimVersionInfo
"BulletSimVersion=$BulletSimVersion" | Out-File -Append BulletSimVersionInfo
"BulletSimGitVersion=$BulletSimGitVersion" | Out-File -Append BulletSimVersionInfo
"BulletSimGitVersionShort=$BulletSimGitVersionShort" | Out-File -Append BulletSimVersionInfo
"BulletVersion=$BulletVersion" | Out-File -Append BulletSimVersionInfo
"BulletGitVersion=$BulletGitVersion" | Out-File -Append BulletSimVersionInfo
"BulletGitVersionShort=$BulletGitVersionShort" | Out-File -Append BulletSimVersionInfo
"USEOPENCL=$USEOPENCL" | Out-File -Append BulletSimVersionInfo
Get-Content BulletSimVersionInfo

Write-Host "=== removing libBulletSim-*"
Remove-Item libBulletSim-*.dll -ErrorAction SilentlyContinue

if ($BUILDBULLET3 -eq "yes") {
    Write-Host "=== building bullet3"
    Set-Location $BASE
    # Build the Bullet physics engine
    $env:BULLETDIR = "bullet3"
    # Pass OpenCL preference to build script
    $env:USEOPENCL = $USEOPENCL
    ./buildBulletCMake.ps1
    # Build the BulletSim glue/wrapper statically linked to Bullet
    ./buildBulletSim.ps1
}