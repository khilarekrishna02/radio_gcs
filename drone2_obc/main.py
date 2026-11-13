
import threading
import time

from pixhawk import PixhawkController
from radio_bridge import RadioBridge


def main():

    print("===================================")
    print("       DRONE 2 ONBOARD SYSTEM")
    print("===================================")

    running = threading.Event()
    running.set()

    # 1. Initialize Pixhawk controller
    pixhawk = PixhawkController()

    # 2. Start Pixhawk telemetry in background
    telemetry_thread = threading.Thread(
        target=pixhawk.telemetry_loop,
        args=(running,),
        daemon=True,
        name="drone2-pixhawk-telemetry"
    )

    telemetry_thread.start()

    # 3. Start radio independently
    radio = RadioBridge(pixhawk)

    try:
        radio.start()

        print("Drone 2 SiK radio started.")
        print("Waiting for laptop GCS commands...")
        print("Pixhawk connection runs independently.")

        # 4. Keep system alive
        while running.is_set():
            if pixhawk.connected:
                pass

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutdown requested.")

    except Exception as exc:
        print(f"Drone 2 system error: {exc}")

    finally:
        running.clear()
        radio.stop()

        if pixhawk.master:
            pixhawk.master.close()

        print("Drone 2 onboard system stopped.")


if __name__ == "__main__":
    main()
