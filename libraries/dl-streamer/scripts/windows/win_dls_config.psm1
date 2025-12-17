Write-Host "--- INFO: Begin DLStreamerPrerequisites ---" -ForegroundColor Green

Configuration DLStreamerPrerequisites {
    
    param(
        [Parameter(Mandatory)]
        [string]$GStreamerVersion,
        
        [Parameter(Mandatory)]
        [string]$OpenVINOVersion,
        
        [Parameter(Mandatory)]
        [string]$SourceLocation, 
        
        [string]$OpenVINODestination,
        [string]$GStreamerDestination,
        [string]$TempFolder,
        [string]$VCPKGRoot,
        [string]$GitInstallPath,
        [string]$BuildToolsInstallPath,
        [string]$LibVADestination,
        [string]$LibVADriverVersion
    )

    Import-DscResource -ModuleName PSDesiredStateConfiguration

    Node localhost {
        
        # 1. Temporary Directory Creation
        File DlStreamerTmpDir {
            Ensure = "Present"
            Type   = "Directory"
            DestinationPath = $TempFolder 
        }
        
        # 2. Git Installation Check
        Script InstallGit {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = Test-Path $using:GitInstallPath } }
            TestScript = { 
                if (-not (Test-Path $using:GitInstallPath)) { return $false }
                try {
                    # Check if longpaths is enabled in Git configuration
                    $val = git config --system core.longpaths
                    return $val -eq "true"
                } catch { return $false }
            }
            SetScript = {
                Write-Host "Action required: Please install Git to $($using:GitInstallPath) manually."
                Write-Host "Ensure 'git config --system core.longpaths true' is executed for large repositories."
            }
        }

        # 3. VCPKG Root Environment Variable
        Environment VCPKGRootEnv {
            Ensure    = "Present"
            Name      = "VCPKG_ROOT"
            Value     = $VCPKGRoot
            DependsOn = "[Script]InstallGit"
        }
        
        # 4. VCPKG Bootstrapping and Tool Fetching
        Script InstallVCPKG {
            TestScript = {
                # Check if vcpkg.exe exists
                $vExe = Join-Path $using:VCPKGRoot "vcpkg.exe"
                if (-Not (Test-Path $vExe)) { return $false }
                
                # Check if internal CMake tool exists within vcpkg tools directory
                $vRoot = $using:VCPKGRoot
                $CMAKE_TOOLS_ROOT = "$vRoot\downloads\tools"
                
                $CMAKE_EXE = Get-ChildItem -Path $CMAKE_TOOLS_ROOT -Filter "cmake.exe" -Recurse -ErrorAction SilentlyContinue | 
                            Where-Object { $_.FullName -like "*\bin\cmake.exe" } | 
                            Select-Object -First 1

                return [bool]$CMAKE_EXE
            }

            SetScript = {
                $vRoot = $using:VCPKGRoot
                Write-Host "####################################### Installing VCPKG #######################################"
                
                # Proxy Configuration for Git within DSC session
                if ($env:HTTP_PROXY) {
                    Write-Host "Configuring Git Proxy: $env:HTTP_PROXY"
                    git config --global http.proxy $env:HTTP_PROXY
                    git config --global https.proxy $env:HTTPS_PROXY
                }

                # Clean up incomplete directory if it exists
                if (Test-Path $vRoot) { 
                    Remove-Item -Path $vRoot -Recurse -Force -ErrorAction SilentlyContinue 
                }

                # Clone the official VCPKG repository
                git clone --recursive https://github.com/microsoft/vcpkg.git $vRoot
                
                # Run Bootstrap
                Set-Location -Path $vRoot
                .\scripts\bootstrap-vcpkg.bat -disableMetrics
                
                # Force fetch required tools to avoid runtime download failures
                & "$vRoot\vcpkg.exe" fetch cmake
                & "$vRoot\vcpkg.exe" fetch ninja

                # Configure Default Triplet for Release builds
                if (Test-Path "$vRoot\triplets\x64-windows.cmake") {
                    $content = Get-Content "$vRoot\triplets\x64-windows.cmake"
                    if ($content -notcontains "set(VCPKG_BUILD_TYPE release)") {
                        Add-Content -Path "$vRoot\triplets\x64-windows.cmake" -Value "`r`nset(VCPKG_BUILD_TYPE release)"
                    }
                }
                
                Write-Host "############################################ Done ##############################################"
            }

            GetScript = { return @{ Result = "VCPKG" } }
        }

        # 5. Visual Studio Build Tools Installation
        Script InstallVSBuildTools {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = Test-Path $using:BuildToolsInstallPath } }
            TestScript = { 
                $clPath = "$($using:BuildToolsInstallPath)\VC\Tools\MSVC"
                return (Test-Path $clPath)
            }
            SetScript = {
                $installerPath = Join-Path -Path $using:TempFolder -ChildPath "vs_buildtools.exe"
                if (-Not (Test-Path $installerPath)) {
                    Write-Host "Downloading VS Build Tools installer..."
                    Invoke-WebRequest -OutFile $installerPath -Uri "https://aka.ms/vs/17/release/vs_buildtools.exe"
                }
                Write-Host "Installing VS Build Tools components (Native Desktop and MSVC)..."
                Start-Process -Wait -FilePath $installerPath -ArgumentList "--quiet", "--wait", "--norestart", "--nocache", "--installPath", $using:BuildToolsInstallPath, "--add", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "--add", "Microsoft.VisualStudio.ComponentGroup.NativeDesktop.Core"
            }
        }
        
        # 6. Windows SDK Verification
        Script InstallWindowsSDK {
            DependsOn = "[Script]InstallVSBuildTools"
            GetScript = { return @{ Result = Test-Path "${env:ProgramFiles(x86)}\Windows Kits" } }
            TestScript = { return (Test-Path "${env:ProgramFiles(x86)}\Windows Kits") }
            SetScript = { Write-Host "Manual Action: Please ensure Windows SDK 10.0 is installed." }
        }

        # 7. GStreamer Installation Verification
        Script InstallGStreamer {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = Test-Path $using:GStreamerDestination } }
            TestScript = {
                $pcFile = Join-Path -Path $using:GStreamerDestination -ChildPath "1.0\msvc_x86_64\lib\pkgconfig\gstreamer-1.0.pc"
                return (Test-Path $pcFile)
            }
            SetScript = {
                Write-Host "GStreamer version $($using:GStreamerVersion) not found in $($using:GStreamerDestination)."
                Write-Host "NOTE: Please install GStreamer MSI (Development and Runtime) manually."
            }
        }

        # 8. OpenVINO Installation Verification
        Script InstallOpenVINO {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = Test-Path $using:OpenVINODestination } }
            TestScript = {
                $vFile = Join-Path -Path $using:OpenVINODestination -ChildPath "runtime\version.txt"
                return (Test-Path $vFile)
            }
            SetScript = {
                Write-Host "OpenVINO not found. Please install OpenVINO $($using:OpenVINOVersion) manually to $($using:OpenVINODestination)."
            }
        }

        # 9. OpenVINO GenAI Installation and Verification
        Script InstallOpenVINO-GENAI {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = "OpenVINO GenAI Installation State" } }
            TestScript = {
                $dest = $using:OpenVINODestination
                $requiredVer = $using:OpenVINOVersion
                
                # Check for setupvars.ps1 as an indicator of a valid installation
                if (-Not (Test-Path (Join-Path $dest "setupvars.ps1"))) {
                    Write-Host "OpenVINO not found in $dest - installation required."
                    return $false
                }

                # Verify Version Compatibility
                $versionFile = Join-Path $dest "runtime\version.txt"
                if (Test-Path $versionFile) {
                    $content = Get-Content $versionFile -First 1
                    if ($content -and $content.StartsWith($requiredVer)) {
                        Write-Host "OpenVINO version verified: $content"
                        return $true
                    } else {
                        Write-Host "Version mismatch: Installed version does not start with $requiredVer"
                        return $false
                    }
                }
                
                # If file missing but setupvars exists, assume OK but warn
                Write-Host "Warning: version.txt missing, but setupvars.ps1 exists. Assuming installed."
                return $true
            }
            SetScript = {
                Write-Host "Install openvino GENAI."
                $dest = $using:OpenVINODestination
                $tmp = $using:TempFolder
                $ver = $using:OpenVINOVersion
                $installerName = "openvino_genai_windows_${ver}.0.0_x86_64.zip"
                $installerPath = Join-Path $tmp $installerName
                $downloadUrl = "https://storage.openvinotoolkit.org/repositories/openvino_genai/packages/${ver}/windows/$installerName"

                Write-Host "###################### Installing OpenVINO GenAI $ver ######################"

                # 1. Cleanup existing folder if it's the wrong version
                if (Test-Path $dest) {
                    Write-Host "Removing existing/outdated OpenVINO installation at $dest..."
                    Remove-Item -Path $dest -Recurse -Force
                }

                # 2. Download ZIP package
                if (-Not (Test-Path $installerPath)) {
                    Write-Host "Downloading OpenVINO GenAI from $downloadUrl..."
                    Invoke-WebRequest -Uri $downloadUrl -OutFile $installerPath
                } else {
                    Write-Host "Using existing installer: $installerPath"
                }

                # 3. Extraction Logic
                Write-Host "Extracting archive to C:\..."
                Expand-Archive -Path $installerPath -DestinationPath "C:\" -Force
                
                $extractedFolder = "C:\openvino_genai_windows_${ver}.0.0_x86_64"
                
                # 4. Finalizing path (Rename extracted folder to destination name)
                if (Test-Path $extractedFolder) {
                    $destName = Split-Path $dest -Leaf
                    Write-Host "Finalizing installation path to $dest..."
                    Rename-Item -Path $extractedFolder -NewName $destName
                }

                Write-Host "############################### Done #######################################"
            }
        }
        # 10. LIBVA Compatibility Pack via NuGet
        Script InstallLIBVA {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = Test-Path $using:LibVADestination } }
            TestScript = { 
                $items = Get-ChildItem -Path $using:LibVADestination -Filter "Microsoft.Direct3D.VideoAccelerationCompatibilityPack*" -ErrorAction SilentlyContinue
                return [bool]$items
            }
            SetScript = {
                Write-Host "Installing LIBVA nuget package for Windows..."
                if (-not (Test-Path $using:LibVADestination)) { New-Item -ItemType Directory -Path $using:LibVADestination -Force }
                # Note: Nuget.exe or dotnet restore logic should be handled here if automation is required.
            }
        }

        # 11. PKG_CONFIG_PATH Configuration
        Environment PkgConfigPath {
            Ensure    = "Present"
            Name      = "PKG_CONFIG_PATH"
            DependsOn = @("[Script]InstallGStreamer", "[Script]InstallLIBVA")
            Value     = "$($GStreamerDestination)\1.0\msvc_x86_64\lib\pkgconfig;$($LibVADestination)\Microsoft.Direct3D.VideoAccelerationCompatibilityPack.$($LibVADriverVersion)\build\native\x64\lib\pkgconfig"
        }
        
        # 12. LIBVA Runtime Environment Variables
        Environment LibVaDriverName {
            Ensure    = "Present"
            Name      = "LIBVA_DRIVER_NAME"
            Value     = "vaon12"
            DependsOn = "[Script]InstallLIBVA"
        }
        Environment LibVaDriversPath {
            Ensure    = "Present"
            Name      = "LIBVA_DRIVERS_PATH"
            Value     = "$($LibVADestination)\Microsoft.Direct3D.VideoAccelerationCompatibilityPack.$($LibVADriverVersion)\build\native\x64\bin"
            DependsOn = "[Script]InstallLIBVA"
        }

        # 13. Python Runtime Verification
        Script InstallPython {
            DependsOn = "[File]DlStreamerTmpDir"
            GetScript = { return @{ Result = [bool](Get-Command python -ErrorAction SilentlyContinue) } }
            TestScript = { return [bool](Get-Command python -ErrorAction SilentlyContinue) }
            SetScript = { Write-Host "Please install Python 3.12 and ensure it is in the system PATH." }
        }

        # 14. Smart System PATH Management
        Script SetupPathSmart {
            DependsOn = @("[Script]InstallGit", "[Script]InstallVCPKG", "[Script]InstallPython")
            GetScript = { return @{ TestScriptResult = $true } }
            TestScript = {
                $requiredPaths = @(
                    "$($using:GitInstallPath)\bin", 
                    $using:VCPKGRoot
                )
                $currentPathRaw = (Get-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name Path).Path
                $currentParts = $currentPathRaw -split ';' | ForEach-Object { $_.Trim().TrimEnd('\').ToLower() }
                
                foreach ($req in $requiredPaths) {
                    $reqNorm = $req.ToLower().TrimEnd('\')
                    if ($currentParts -notcontains $reqNorm) { return $false }
                }
                return $true
            }
            SetScript = {
                $regKey = 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment'
                $oldPath = (Get-ItemProperty -Path $regKey -Name Path).Path
                
                $pathsToAdd = @(
                    "$($using:GitInstallPath)\bin",
                    $using:VCPKGRoot
                )

                $pathParts = [System.Collections.Generic.List[string]]::new()
                $seen = [System.Collections.Generic.HashSet[string]]::new()
                
                # Step 1: Filter and clean existing paths
                foreach ($part in ($oldPath -split ';')) {
                    $clean = $part.Trim()
                    if (-not [string]::IsNullOrWhiteSpace($clean)) {
                        $key = $clean.ToLower().TrimEnd('\')
                        if (-not $seen.Contains($key)) {
                            $seen.Add($key) | Out-Null
                            $pathParts.Add($clean)
                        }
                    }
                }
                
                # Step 2: Add missing required paths
                foreach ($newP in $pathsToAdd) {
                    $key = $newP.ToLower().TrimEnd('\')
                    if (-not $seen.Contains($key)) {
                        $pathParts.Add($newP)
                        $seen.Add($key) | Out-Null
                        Write-Host "Appending to System PATH: $newP"
                    }
                }
                
                $finalPath = $pathParts -join ';'
                Set-ItemProperty -Path $regKey -Name Path -Value $finalPath
                Write-Host "System Environment Variable 'PATH' has been updated and deduplicated."
            }
        }
    }
}