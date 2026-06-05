@echo off
setlocal enabledelayedexpansion

set "BAZEL_CACHE_DIR=%USERPROFILE%\.cache\bazel"

:: 1. Feature: BAZEL_OVERRIDE Support
if defined BAZEL_OVERRIDE (
    echo Actually calling "%BAZEL_OVERRIDE%"
    set "_SAVE_TARGET=%BAZEL_OVERRIDE%"
) else (
    :: 2. Ensure Bazelisk Integration
    if not defined BAZEL_REAL (
        echo Error: This script must be run via Bazelisk on Windows. >&2
        exit /b 1
    )
    set "_SAVE_TARGET=%BAZEL_REAL%"
)


:bootstrap_git
:: --- AUTOMATED HERMETIC GIT BOOTSTRAPPING WITH SHA256 VALIDATION ---
set "GIT_CACHE_DIR=%BAZEL_CACHE_DIR%\mingw_git"
set "GIT_EXE_PATH=%GIT_CACHE_DIR%\cmd\git.exe"

if not exist "%GIT_EXE_PATH%" (
    echo [Wrapper] Git not detected in runtime cache. Fetching isolated MinGit... >&2
    
    set "GIT_VERSION=v2.44.0.windows.1"
    set "GIT_ZIP_NAME=MinGit-2.44.0-64-bit.zip"
    set "GIT_URL=https://github.com/git-for-windows/git/releases/download/!GIT_VERSION!/!GIT_ZIP_NAME!"
    set "EXPECTED_SHA256=ed4e74e171c59c9c9d418743c7109aa595e0cc0d1c80cac574d69ed5e571ae59"
    
    set "TEMP_DOWNLOAD_DIR=%BAZEL_CACHE_DIR%\git_tmp"
    if exist "!TEMP_DOWNLOAD_DIR!" rmdir /s /q "!TEMP_DOWNLOAD_DIR!"
    mkdir "!TEMP_DOWNLOAD_DIR!"
    
    echo [Wrapper] Downloading from !GIT_URL! ... >&2
    
    set "NOISINESS=--silent"
    sessionfile-check >nul 2>&1
    if %errorlevel% equ 0 set "NOISINESS="

    curl -fLk !NOISINESS! --show-error --output "!TEMP_DOWNLOAD_DIR!\git.zip" "!GIT_URL!"
    if errorlevel 1 (
        echo Error: Failed to download hermetic Git toolchain >&2
        exit /b 1
    )
    
    echo [Wrapper] Validating cryptographic payload checksum... >&2
    set "COMPUTED_SHA256="
    for /f "skip=1 delims=" %%A in ('certutil -hashfile "!TEMP_DOWNLOAD_DIR!\git.zip" SHA256 ^| findstr /v "CertUtil"') do (
        set "LINE=%%A"
        set "LINE=!LINE: =!"
        set "COMPUTED_SHA256=!LINE!"
    )
    
    if /i not "!COMPUTED_SHA256!"=="!EXPECTED_SHA256!" (
        echo. >&2
        echo =============================================================== >&2
        echo SECURITY ERROR: Cryptographic checksum mismatch detected. >&2
        echo Expected: !EXPECTED_SHA256! >&2
        echo Received: !COMPUTED_SHA256! >&2
        echo =============================================================== >&2
        rmdir /s /q "!TEMP_DOWNLOAD_DIR!"
        exit /b 1
    )
    echo [Wrapper] Integrity verification successful. SHA256 matches. >&2
    
    echo [Wrapper] Extracting archive package... >&2
    mkdir "%GIT_CACHE_DIR%" 2>nul
    tar -xf "!TEMP_DOWNLOAD_DIR!\git.zip" -C "%GIT_CACHE_DIR%"
    
    rmdir /s /q "!TEMP_DOWNLOAD_DIR!"
    
    echo [Wrapper] Isolated Git runtime setup completed successfully. >&2
)

:: Configure the global settings every time to ensure any pre-existing caches are correctly updated
"%GIT_EXE_PATH%" config --global http.sslBackend openssl
"%GIT_EXE_PATH%" config --global http.sslVerify true

:autodetect
:: 3. Feature: Use our bootstrapped Git to find the absolute root directory cleanly
set "TOP_LEVEL_DIR="
for /f "tokens=*" %%i in ('"%GIT_EXE_PATH%" rev-parse --show-toplevel 2^>nul') do set "TOP_LEVEL_DIR=%%i"

if "%TOP_LEVEL_DIR%"=="" (
    set "TOP_LEVEL_DIR=%CD%"
)

:: --- URL ENCODING FOR BZLMOD REGISTRY ---
set "URI_PATH=%TOP_LEVEL_DIR:\=/%"
set "ENC_SPACE=%%20"
set "URI_PATH=!URI_PATH: =%ENC_SPACE%!"

set "TMP_BAZELRC=%TOP_LEVEL_DIR%\.bazelrc_autodetection.tmp"
(
    echo # Set the default host platform to Windows x86_64
    echo build --host_platform=//tools/platforms:windows_x86_64
    echo build --extra_execution_platforms=//tools/platforms:windows_x86_64
    echo.
    echo # Inject the locally encoded Bzlmod registry path safely bypassing workspace expansion spaces.
    echo # The local registry must come before bcr (see the registry comment in .bazelrc).
    echo common --registry=file:///!URI_PATH!/registry
    echo common --registry=https://bcr.bazel.build
) > "%TMP_BAZELRC%"
move /y "%TMP_BAZELRC%" "%TOP_LEVEL_DIR%\.bazelrc_autodetection" >nul


:: 4. Feature: Environment Sandboxing (Windows equivalent of "env -i")
set "_SAVE_SYSTEMROOT=%SystemRoot%"
set "_SAVE_SYSTEMDRIVE=%SystemDrive%"
set "_SAVE_COMSPEC=%ComSpec%"
set "_SAVE_PATH=%PATH%"
set "_SAVE_TEMP=%TEMP%"
set "_SAVE_TMP=%TMP%"
set "_SAVE_USERPROFILE=%USERPROFILE%"
set "_SAVE_USERNAME=%USERNAME%"
set "_SAVE_CARGO_REPIN=%CARGO_BAZEL_REPIN%"
:: CRITICAL CRADLE: Back up our bootstrapped Git path and git configurations so the environment purge loop ignores them
set "_SAVE_GIT_CACHE_DIR=%GIT_CACHE_DIR%"
set "_SAVE_GIT_CONFIG_PARAMETERS=http.sslBackend=openssl http.sslVerify=true"

for /f "tokens=1 delims==" %%a in ('set') do (
    set "VAR_NAME=%%a"
    if not "!VAR_NAME:~0,6!"=="_SAVE_" (
        set "%%a="
    )
)

set "SystemRoot=%_SAVE_SYSTEMROOT%"
set "SystemDrive=%_SAVE_SYSTEMDRIVE%"
set "ComSpec=%_SAVE_COMSPEC%"
set "TEMP=%_SAVE_TEMP%"
set "TMP=%_SAVE_TMP%"
set "USERPROFILE=%_SAVE_USERPROFILE%"
set "HOME=%_SAVE_USERPROFILE%"
set "USERNAME=%_SAVE_USERNAME%"
set "USER=%_SAVE_USERNAME%"
set "GIT_CONFIG_PARAMETERS=%_SAVE_GIT_CONFIG_PARAMETERS%"

:: Rehydrate the clean sandbox PATH variable incorporating the protected Git path
set "PATH=%_SAVE_SYSTEMROOT%\system32;%_SAVE_SYSTEMROOT%;%_SAVE_SYSTEMROOT%\System32\Wbem;%_SAVE_GIT_CACHE_DIR%\cmd;%_SAVE_PATH%"

:: CRITICAL EXPLICIT BINDING: Force Bazel's repository rules to bypass PATH lookups entirely
set "BAZEL_GIT=%_SAVE_GIT_CACHE_DIR%\cmd\git.exe"
set "GIT_BIN_PATH=%_SAVE_GIT_CACHE_DIR%\cmd\git.exe"
set "BAZEL_SH=%_SAVE_GIT_CACHE_DIR%\bin\bash.exe"

set "HOSTNAME=%COMPUTERNAME%"
set "TERM=dumb"
set "LANG=C"
set "BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN=0"

if defined _SAVE_CARGO_REPIN (
    set "CARGO_BAZEL_REPIN=%_SAVE_CARGO_REPIN%"
)

:: 5. Execute the isolated Bazel process
"%_SAVE_TARGET%" %*
endlocal
