@echo off
cd /d %~dp0
set GCS_RADIO_PORT=COM12
set GCS_WEB_HOST=127.0.0.1
python api_proxy.py
pause
