@echo off

rem Enable color support using PowerShell
setlocal EnableDelayedExpansion

rem Define color functions
set "ECHO_RED=powershell -Command "Write-Host '%*' -ForegroundColor Red""
set "ECHO_GREEN=powershell -Command "Write-Host '%*' -ForegroundColor Green""
set "ECHO_YELLOW=powershell -Command "Write-Host '%*' -ForegroundColor Yellow""
set "ECHO_BLUE=powershell -Command "Write-Host '%*' -ForegroundColor Cyan""

powershell -Command "Write-Host '[Build] Starting build process...' -ForegroundColor Cyan"

rem Project directories setup
set PROJECT_DIR=%~dp0..
set BUILD_DIR=%PROJECT_DIR%\build

powershell -Command "Write-Host '[Build] Project directory: %PROJECT_DIR%' -ForegroundColor Cyan"
powershell -Command "Write-Host '[Build] Build directory: %BUILD_DIR%' -ForegroundColor Cyan"

if not exist "%BUILD_DIR%" (
    powershell -Command "Write-Host '[Build] Creating build directory...' -ForegroundColor Yellow"
    mkdir "%BUILD_DIR%"
)

pushd "%BUILD_DIR%"

powershell -Command "Write-Host '[Build] Running CMake...' -ForegroundColor Cyan"
cmake -G "Ninja" "%PROJECT_DIR%"

powershell -Command "Write-Host '[Build] Running Ninja...' -ForegroundColor Cyan"
ninja > ninja_output.txt 2>&1
set NINJA_EXIT_CODE=%ERRORLEVEL%

rem Display colored output
powershell -Command "Get-Content ninja_output.txt | ForEach-Object { if ($_ -match 'FAILED:') { Write-Host $_ -ForegroundColor Red } elseif ($_ -match ':\\d+:\\d+:.*error') { Write-Host $_ -ForegroundColor Red } elseif ($_ -match 'error:|Error:|ERROR:|undeclared|implicit declaration') { Write-Host $_ -ForegroundColor Red } elseif ($_ -match ':\\d+:\\d+:.*warning') { Write-Host $_ -ForegroundColor Yellow } elseif ($_ -match 'warning:|Warning:|WARNING:|unused') { Write-Host $_ -ForegroundColor Yellow } elseif ($_ -match 'note:') { Write-Host $_ -ForegroundColor Cyan } elseif ($_ -match '^\\s*\\d+\\s*\\|') { Write-Host $_ -ForegroundColor Magenta } elseif ($_ -match '^\\s*\\^') { Write-Host $_ -ForegroundColor Red } else { Write-Host $_ } }"

rem Check build status
if %NINJA_EXIT_CODE% NEQ 0 (
    powershell -Command "Write-Host '[Build] ERROR: Compilation failed! Please check the errors above.' -ForegroundColor Red"
    set BUILD_STATUS=FAILED
) else (
    powershell -Command "Write-Host '[Build] SUCCESS: Compilation completed successfully!' -ForegroundColor Green"
    set BUILD_STATUS=SUCCESS
)

rem Check if ELF file exists
if exist usb_f_h.elf (
    powershell -Command "Write-Host '[Build] Found ELF file, generating binary and hex files...' -ForegroundColor Green"
    
    rem Generate BIN file
    arm-none-eabi-objcopy -O binary usb_f_h.elf usb_app.bin
    powershell -Command "Write-Host '[Build] Created: usb_app.bin' -ForegroundColor Green"
    
    rem Generate HEX file
    arm-none-eabi-objcopy -O ihex usb_f_h.elf usb_app.hex
    powershell -Command "Write-Host '[Build] Created: usb_app.hex' -ForegroundColor Green"
) else (
    powershell -Command "Write-Host '[Build] ERROR: usb_f_h.elf not found!' -ForegroundColor Red"
    powershell -Command "Write-Host '[Build] Checking for other ELF files...' -ForegroundColor Yellow"
    dir *.elf
)

popd

rem Display final build status message
if "%BUILD_STATUS%"=="SUCCESS" (
    echo [Build] ================================================================
    echo [Build] BUILD SUCCESSFUL! The firmware has been compiled successfully.
    echo [Build] ================================================================
) else (
    echo [Build] ================================================================
    echo [Build] BUILD FAILED! Please fix the errors and try again.
    echo [Build] ================================================================
)

echo [Build] Build process completed!
