@echo off
echo ================================
echo Building control_demos (Release)
echo ================================

cmake --build build --config Release
if errorlevel 1 goto :error

echo.
echo ================================
echo Installing control_demos
echo ================================

cmake --install build --config Release
if errorlevel 1 goto :error

echo.
echo ================================
echo Done.
echo ================================
exit /b 0

:error
echo.
echo ================================
echo Build or install failed.
echo ================================
exit /b 1
