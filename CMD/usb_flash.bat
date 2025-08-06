@echo off
echo [Flash] Start flashing...
python flash_jlink.py
if %ERRORLEVEL% EQU 0 (
  echo [Flash] Flash successful!
) else (
  echo [Flash] Flash failed!
)
