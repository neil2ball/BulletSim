#! /usr/bin/env pwsh
# Script to download and install Khronos vendor-neutral OpenCL SDK

param(
    [string]$InstallPath = "C:\OpenCL-SDK",
    [switch]$Force = $false
)

Write-Host "=== Khronos OpenCL SDK Installer ==="

# Check if already installed
if (Test-Path $InstallPath -PathType Container) {
    if (-not $Force) {
        Write-Host "OpenCL SDK already exists at $InstallPath. Use -Force to reinstall."
        exit 0
    } else {
        Write-Host "Removing existing installation..."
        Remove-Item -Recurse -Force $InstallPath -ErrorAction SilentlyContinue
    }
}

# Create installation directory
New-Item -ItemType Directory -Path $InstallPath -Force | Out-Null

# GitHub API URL for latest release
$apiUrl = "https://api.github.com/repos/KhronosGroup/OpenCL-SDK/releases/latest"

Write-Host "Fetching latest release information..."
try {
    $releaseInfo = Invoke-RestMethod -Uri $apiUrl -Headers @{
        'Accept' = 'application/vnd.github.v3+json'
    }
} catch {
    Write-Error "Failed to fetch release information: $($_.Exception.Message)"
    exit 1
}

# Find the Windows zip download - MORE FLEXIBLE FILTERING
$windowsAsset = $releaseInfo.assets | Where-Object { 
    $_.name -match '\.zip$' -and (
        ($_.name -like "*windows*x64*") -or 
        ($_.name -like "*win64*") -or 
        ($_.name -like "*Windows*x64*") -or
        ($_.name -like "*Windows*AMD64*") -or
        ($_.name -match "(?i)windows.*x64") -or
        ($_.name -match "(?i)win.*64")
    )
}

# If no match found, try to find any zip file that might be Windows-related
if (-not $windowsAsset) {
    Write-Warning "No exact Windows x64 zip match found. Looking for any Windows-related zip..."
    $windowsAsset = $releaseInfo.assets | Where-Object { 
        $_.name -match '\.zip$' -and (
            $_.name -match "(?i)windows" -or 
            $_.name -match "(?i)win[^a-z]" -or
            $_.name -match "x64" -or
            $_.name -match "amd64"
        )
    }
}

# If still no match, show available assets and exit
if (-not $windowsAsset) {
    Write-Error "No Windows x64 zip found in latest release. Available assets:"
    $releaseInfo.assets | ForEach-Object { Write-Host "  - $($_.name)" }
    exit 1
}

# If multiple matches found, take the first one
if ($windowsAsset -is [array]) {
    Write-Warning "Multiple Windows assets found. Selecting the first one: $($windowsAsset[0].name)"
    $windowsAsset = $windowsAsset[0]
}

$downloadUrl = $windowsAsset.browser_download_url
$zipFileName = $windowsAsset.name
$zipPath = Join-Path $env:TEMP $zipFileName

Write-Host "Downloading OpenCL SDK: $zipFileName"
Write-Host "From: $downloadUrl"

try {
    # Download the file
    Invoke-WebRequest -Uri $downloadUrl -OutFile $zipPath
} catch {
    Write-Error "Failed to download OpenCL SDK: $($_.Exception.Message)"
    exit 1
}

# Extract to temporary directory first to examine structure
$tempExtractPath = Join-Path $env:TEMP "OpenCL-SDK-Temp"
if (Test-Path $tempExtractPath) {
    Remove-Item -Recurse -Force $tempExtractPath -ErrorAction SilentlyContinue
}
New-Item -ItemType Directory -Path $tempExtractPath -Force | Out-Null

Write-Host "Extracting to temporary location: $tempExtractPath"
try {
    Expand-Archive -Path $zipPath -DestinationPath $tempExtractPath -Force
} catch {
    Write-Error "Failed to extract OpenCL SDK: $($_.Exception.Message)"
    exit 1
}

# Check if the extracted content has a single top-level directory
$extractedItems = Get-ChildItem -Path $tempExtractPath
if ($extractedItems.Count -eq 1 -and $extractedItems[0].PSIsContainer) {
    Write-Host "Found nested directory structure. Moving contents to target location..."
    $nestedDir = $extractedItems[0].FullName
    
    # Copy all contents from the nested directory to the install path
    Get-ChildItem -Path $nestedDir | ForEach-Object {
        $destination = Join-Path $InstallPath $_.Name
        if ($_.PSIsContainer) {
            Copy-Item -Path $_.FullName -Destination $destination -Recurse -Force
        } else {
            Copy-Item -Path $_.FullName -Destination $destination -Force
        }
    }
} else {
    # Direct extraction - copy all items to install path
    Write-Host "Copying extracted files to target location..."
    Get-ChildItem -Path $tempExtractPath | ForEach-Object {
        $destination = Join-Path $InstallPath $_.Name
        if ($_.PSIsContainer) {
            Copy-Item -Path $_.FullName -Destination $destination -Recurse -Force
        } else {
            Copy-Item -Path $_.FullName -Destination $destination -Force
        }
    }
}

# Clean up temp files
Remove-Item $tempExtractPath -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item $zipPath -ErrorAction SilentlyContinue

# Set environment variables (FOLDER PATHS, not specific files)
Write-Host "Setting environment variables..."

# Set permanent system environment variables
try {
    [Environment]::SetEnvironmentVariable("OPENCL_SDK_ROOT", $InstallPath, "Machine")
    [Environment]::SetEnvironmentVariable("OpenCL_INCLUDE_DIR", "$InstallPath\include", "Machine")
    [Environment]::SetEnvironmentVariable("OpenCL_LIBRARY_DIR", "$InstallPath\lib", "Machine")
    Write-Host "Set OpenCL system environment variables"
} catch {
    Write-Warning "Could not set system environment variables (admin rights required). Setting user variables instead."
    [Environment]::SetEnvironmentVariable("OPENCL_SDK_ROOT", $InstallPath, "User")
    [Environment]::SetEnvironmentVariable("OpenCL_INCLUDE_DIR", "$InstallPath\include", "User")
    [Environment]::SetEnvironmentVariable("OpenCL_LIBRARY_DIR", "$InstallPath\lib", "User")
}

# Add to PATH (bin folder for DLLs)
try {
    $currentPath = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $binPath = Join-Path $InstallPath "bin"
    
    if (-not $currentPath.Contains($binPath)) {
        $newPath = $currentPath + ";" + $binPath
        [Environment]::SetEnvironmentVariable("Path", $newPath, "Machine")
        Write-Host "Added OpenCL SDK bin to system PATH"
    }
} catch {
    Write-Warning "Could not update system PATH (admin rights required). Updating user PATH instead."
    $currentPath = [Environment]::GetEnvironmentVariable("Path", "User")
    $binPath = Join-Path $InstallPath "bin"
    
    if (-not $currentPath.Contains($binPath)) {
        $newPath = $currentPath + ";" + $binPath
        [Environment]::SetEnvironmentVariable("Path", $newPath, "User")
    }
}

# Set session variables for immediate use (FOLDER PATHS)
$env:OPENCL_SDK_ROOT = $InstallPath
$env:OpenCL_INCLUDE_DIR = "$InstallPath\include"
$env:OpenCL_LIBRARY_DIR = "$InstallPath\lib"
$env:Path += ";" + (Join-Path $InstallPath "bin")

# Verify installation
Write-Host "`n=== Verification ==="
$checks = @(
    @{Name="SDK Root"; Path="$InstallPath"; Type="Directory"},
    @{Name="Include Directory"; Path="$InstallPath\include"; Type="Directory"},
    @{Name="Lib Directory"; Path="$InstallPath\lib"; Type="Directory"},
    @{Name="Bin Directory"; Path="$InstallPath\bin"; Type="Directory"}
)

$allGood = $true
foreach ($check in $checks) {
    $exists = Test-Path $check.Path -PathType Container
    
    $status = if ($exists) { "[OK]" } else { "[FAIL]" }
    Write-Host "$status $($check.Name): $($check.Path)"
    
    if (-not $exists) {
        $allGood = $false
    }
}

if ($allGood) {
    Write-Host "`n[SUCCESS] OpenCL SDK installed successfully!"
    Write-Host "Environment variables set for current session."
    Write-Host "Permanent variables will be available after restarting PowerShell."
    
    Write-Host "`n=== Current Session Variables ==="
    Write-Host "OPENCL_SDK_ROOT: $env:OPENCL_SDK_ROOT"
    Write-Host "OpenCL_INCLUDE_DIR: $env:OpenCL_INCLUDE_DIR"
    Write-Host "OpenCL_LIBRARY_DIR: $env:OpenCL_LIBRARY_DIR"
} else {
    Write-Host "`n[WARNING] Installation completed with some missing components."
}

Write-Host "`n=== Usage ==="
Write-Host "To use in CMake, add these flags:"
Write-Host "  -DUSE_OPENCL=ON"
Write-Host "  -DOpenCL_INCLUDE_DIR=`"$env:OpenCL_INCLUDE_DIR`""
Write-Host "  -DOpenCL_LIBRARY=`"$env:OpenCL_LIBRARY_DIR`""