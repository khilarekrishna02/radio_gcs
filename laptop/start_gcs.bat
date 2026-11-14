@echo off
cd /d %~dp0
:: Optional port overrides; leave unset or empty for automatic port detection
:: set GCS_RADIO_PORT=COM13
:: set GCS_RADIO2_PORT=COM12
set GCS_WEB_HOST=127.0.0.1
python api_proxy.py
pause

