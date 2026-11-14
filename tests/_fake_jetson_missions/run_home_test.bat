@echo off
cd /d %~dp0
echo Checking your setup...
python preflight.py
if errorlevel 1 (
  echo.
  echo Fix the FAIL lines above, then run this file again.
  pause
  exit /b 1
)
echo.
echo Starting virtual radio, fake Jetson and the laptop GCS...
start "Virtual radio" cmd /k python virtual_radio.py
timeout /t 2 /nobreak >nul
start "Fake Jetson" cmd /k python fake_jetson.py
timeout /t 2 /nobreak >nul
start "Laptop GCS" cmd /k "cd /d %~dp0..\laptop&& set GCS_RADIO_PORT=socket://127.0.0.1:9001&& python api_proxy.py"
python preflight.py --wait 5000 40
if errorlevel 1 (
  echo.
  echo The "Laptop GCS" window shows why the web page did not start.
  pause
  exit /b 1
)
start http://127.0.0.1:5000
