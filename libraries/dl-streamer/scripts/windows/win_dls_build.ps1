# New-NetFirewallRule -DisplayName "Windows Remote Management (HTTP-In, Public Profile)" -Direction Inbound -LocalPort 5985 -Protocol TCP -Action Allow -Profile Public -Program System
# Record start time
$StartTime = Get-Date

# ==============================================================================
# PARAMETERS & CONFIGURATION
# ==============================================================================
# Network & Proxy Settings
$USE_INTERNAL_PROXY     = $true
$HTTP_PROXY_URL         = "http://proxy-dmz.intel.com:911"
$HTTPS_PROXY_URL        = "http://proxy-dmz.intel.com:912"
$NO_PROXY_LIST          = "localhost,127.0.0.1,.intel.com"

# Version Definitions
$GSTREAMER_VERSION_VAR      = "1.26.6"
$OPENVINO_VERSION_VAR       = "2025.3"
$LIBVA_DRIVER_VERSION_VAR   = "1.0.2"

# Installation Path Definitions
$VCPKG_ROOT_VAR             = "C:\vcpkg"
$OPENVINO_DEST_FOLDER_VAR   = "C:\openvino"
$GSTREAMER_DEST_FOLDER_VAR  = "C:\gstreamer"
$DLSTREAMER_TMP_VAR         = "C:\dlstreamer_tmp"
$LIBVA_DEST_FOLDER_VAR      = "C:\libva"
$GIT_INSTALL_PATH_VAR       = "C:\Program Files\Git"
$BUILD_TOOLS_PATH_VAR       = "C:\BuildTools"

# Build Settings
$BUILD_CONFIG        = "Release"
$BUILD_TARGET        = "ALL_BUILD"
# Automatically detect logical CPU cores
$CPU_CORES           = $env:NUMBER_OF_PROCESSORS 
# Optional: Use only 80% of cores to keep system responsive
$PARALLEL_JOBS       = [math]::Max(1, [int]($CPU_CORES * 0.8))

# Source Location
$DLSTREAMER_SRC_LOCATION_VAR = (Get-Item $PSScriptRoot).Parent.Parent.FullName
Write-Host "Detected DL-Streamer Source Location: $DLSTREAMER_SRC_LOCATION_VAR" -ForegroundColor Cyan

# ==============================================================================
# PROXY SETTINGS
# ==============================================================================
if ($USE_INTERNAL_PROXY) {
    $env:HTTP_PROXY  = $HTTP_PROXY_URL
    $env:HTTPS_PROXY = $HTTPS_PROXY_URL
    $env:NO_PROXY    = $NO_PROXY_LIST
    
    Write-Host "###################### Proxy Configured ######################" -ForegroundColor Cyan
    Write-Host "- HTTP_PROXY  = $env:HTTP_PROXY"
    Write-Host "- HTTPS_PROXY = $env:HTTPS_PROXY"
    Write-Host "- NO_PROXY    = $env:NO_PROXY"

    # Apply to Winget if available
    if (Get-Command winget -ErrorAction SilentlyContinue) {
        winget settings --proxy $env:HTTP_PROXY | Out-Null
        Write-Host "- Winget proxy also configured."
    }
    
    # Apply to Git globally (Critical for VCPKG clone)
    if (Get-Command git -ErrorAction SilentlyContinue) {
        git config --global http.proxy $env:HTTP_PROXY
        git config --global https.proxy $env:HTTPS_PROXY
        Write-Host "- Git global proxy configured."
    }
    Write-Host "##############################################################"
}

# ==============================================================================
# DSC CONFIGURATION EXECUTION
# ==============================================================================
Write-Host "Loading win_dls_config.psm1..."
Import-Module .\win_dls_config.psm1 -Force

Write-Host "Compiling DSC MOF file..."
DLStreamerPrerequisites -GStreamerVersion $GSTREAMER_VERSION_VAR `
                        -OpenVINOVersion $OPENVINO_VERSION_VAR `
                        -OpenVINODestination $OPENVINO_DEST_FOLDER_VAR `
                        -GStreamerDestination $GSTREAMER_DEST_FOLDER_VAR `
                        -TempFolder $DLSTREAMER_TMP_VAR `
                        -VCPKGRoot $VCPKG_ROOT_VAR `
                        -GitInstallPath $GIT_INSTALL_PATH_VAR `
                        -BuildToolsInstallPath $BUILD_TOOLS_PATH_VAR `
                        -LibVADestination $LIBVA_DEST_FOLDER_VAR `
                        -LibVADriverVersion $LIBVA_DRIVER_VERSION_VAR `
                        -SourceLocation $DLSTREAMER_SRC_LOCATION_VAR `
                        -OutputPath ".\DLS_Prereqs_MOF"

Write-Host "Applying DSC Configuration (this may take a while)..."
Start-DscConfiguration -Path ".\DLS_Prereqs_MOF" -Wait -Verbose -Force
# Start-DscConfiguration -Path ".\DLS_Prereqs_MOF" -Wait -Force

# ==============================================================================
# ENVIRONMENT AUDIT SUMMARY
# ==============================================================================
Write-Host "`n###################### Environment Audit Summary ######################" -ForegroundColor Cyan

function Get-ComponentStatus {
    process {
        $gitPath = try { (Get-Command git -ErrorAction SilentlyContinue).Source } catch { "Not Found" }
        $gitVer  = try { (git --version) -replace 'git version ', '' } catch { "N/A" }
        
        $vcpkgExe = "$VCPKG_ROOT_VAR\vcpkg.exe"
        $vcpkgVer = if (Test-Path $vcpkgExe) { 
            $raw = (& $vcpkgExe version | Select-Object -First 1)
            $raw.Substring($raw.IndexOf("version ") + 8).Split("-")[0]
        } else { "N/A" }

        $cmakePath = try { (Get-Command cmake -ErrorAction SilentlyContinue).Source } catch { "Not Found" }
        $cmakeVer  = try { (cmake --version | Select-Object -First 1) -replace 'cmake version ', '' } catch { "N/A" }

        $ovVer = if (Test-Path "$OPENVINO_DEST_FOLDER_VAR\runtime\version.txt") { (Get-Content "$OPENVINO_DEST_FOLDER_VAR\runtime\version.txt" | Select-Object -First 1).Trim() } else { "N/A" }

        $gsBin = "$GSTREAMER_DEST_FOLDER_VAR\1.0\msvc_x86_64\bin\gst-inspect-1.0.exe"
        $gsVer = if (Test-Path $gsBin) { (& $gsBin --version | Select-Object -First 1) -replace 'gst-inspect-1.0 version ', '' } else { "N/A" }

        $pyPath = try { (Get-Command python -ErrorAction SilentlyContinue).Source } catch { "Not Found" }
        $pyVer  = try { (python --version) -replace 'Python ', '' } catch { "N/A" }

        $sdkPath = "${env:ProgramFiles(x86)}\Windows Kits\10"
        $sdkStatus = if (Test-Path $sdkPath) { "Installed" } else { "Missing" }

        return @(
            [PSCustomObject]@{ Component = "Git"; Version = $gitVer; Status = if($gitVer -ne "N/A"){"OK"}else{"FAIL"}; Path = $gitPath },
            [PSCustomObject]@{ Component = "VCPKG"; Version = $vcpkgVer; Status = if($vcpkgVer -ne "N/A"){"OK"}else{"FAIL"}; Path = $VCPKG_ROOT_VAR },
            [PSCustomObject]@{ Component = "CMake"; Version = $cmakeVer; Status = if($cmakeVer -ne "N/A"){"OK"}else{"FAIL"}; Path = $cmakePath },
            [PSCustomObject]@{ Component = "OpenVINO"; Version = $ovVer; Status = if($ovVer -ne "N/A"){"OK"}else{"FAIL"}; Path = $OPENVINO_DEST_FOLDER_VAR },
            [PSCustomObject]@{ Component = "GStreamer"; Version = $gsVer; Status = if($gsVer -ne "N/A"){"OK"}else{"FAIL"}; Path = $GSTREAMER_DEST_FOLDER_VAR },
            [PSCustomObject]@{ Component = "Python"; Version = $pyVer; Status = if($pyVer -ne "N/A"){"OK"}else{"FAIL"}; Path = $pyPath },
            [PSCustomObject]@{ Component = "WinSDK"; Version = "10.0+"; Status = if($sdkStatus -eq "Installed"){"OK"}else{"FAIL"}; Path = $sdkPath }
        )
    }
}

Get-ComponentStatus | Format-Table -Property Component, Status, Version, Path -AutoSize
Write-Host "#######################################################################"

# ==============================================================================
# POST-INSTALLATION & BUILD
# ==============================================================================
Write-Host "###################### Configuring VCPKG dependencies #################"
if (-not (Test-Path "${DLSTREAMER_TMP_VAR}\build")) {
    New-Item -Path "${DLSTREAMER_TMP_VAR}\build" -ItemType Directory -Force
}
Set-Location -Path "${DLSTREAMER_TMP_VAR}\build"

& "$VCPKG_ROOT_VAR\vcpkg.exe" integrate install
& "$VCPKG_ROOT_VAR\vcpkg.exe" install --triplet=x64-windows --vcpkg-root=$VCPKG_ROOT_VAR --x-manifest-root=$DLSTREAMER_SRC_LOCATION_VAR
Stop-Process -Name "msbuild" -ErrorAction SilentlyContinue

Write-Host "###################### Initializing OpenVINO Environment ##############"
if (Test-Path "$OPENVINO_DEST_FOLDER_VAR\setupvars.ps1") {
    & "$OPENVINO_DEST_FOLDER_VAR\setupvars.ps1"
}

Write-Host "###################### Injecting Internal CMake to Path ###############"
$CMAKE_TOOLS_ROOT = "$VCPKG_ROOT_VAR\downloads\tools"
$CMAKE_EXE = Get-ChildItem -Path $CMAKE_TOOLS_ROOT -Filter "cmake.exe" -Recurse -ErrorAction SilentlyContinue | 
             Where-Object { $_.FullName -like "*\bin\cmake.exe" } | Select-Object -First 1

if ($CMAKE_EXE) {
    $env:PATH = "$($CMAKE_EXE.DirectoryName);$env:PATH"
    Write-Host "Using CMake from: $($CMAKE_EXE.FullName)" -ForegroundColor Green
}

Write-Host "###################### Running CMAKE Configuration ####################"
# Use direct call to capture exit code easily via $LASTEXITCODE
& cmake "-DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT_VAR\scripts\buildsystems\vcpkg.cmake" "$DLSTREAMER_SRC_LOCATION_VAR"
if ($LASTEXITCODE -ne 0) { Write-Error "CMake configuration failed"; exit $LASTEXITCODE }

Write-Host "###################### Building DL Streamer ###########################"
Write-Host "Using $PARALLEL_JOBS parallel jobs (Total Cores: $CPU_CORES)" -ForegroundColor Cyan

& cmake --build . -v --parallel $PARALLEL_JOBS --target $BUILD_TARGET --config $BUILD_CONFIG
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; exit $LASTEXITCODE }

# ==============================================================================
# EXECUTION TIME CALCULATION
# ==============================================================================
$EndTime = Get-Date
$Duration = $EndTime - $StartTime

Write-Host "`n#######################################################################" -ForegroundColor Green
Write-Host "  Script Completed Successfully!" -ForegroundColor Green
Write-Host "  Total Execution Time: $($Duration.Minutes) Minutes, $($Duration.Seconds) Seconds" -ForegroundColor Green
Write-Host "#######################################################################"