cd "$(dirname "$0")"

export GCS_RADIO_PORT=/dev/ttyUSB0
export GCS_WEB_HOST=127.0.0.1

python3 api_proxy.py

read -p "Press Enter to exit..."