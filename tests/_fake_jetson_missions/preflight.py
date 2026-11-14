#!/usr/bin/env python3
"""Setup check for the home test. run_home_test.bat runs this before starting anything.

    python preflight.py                 check packages, files and that the code imports
    python preflight.py --wait 5000 40  wait up to 40 s for something to listen on port 5000
"""
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
LAPTOP = os.path.join(ROOT, "laptop")
JETSON = next((os.path.join(ROOT, d) for d in ("jetson", "rpi") if os.path.isdir(os.path.join(ROOT, d))),
              os.path.join(ROOT, "jetson"))
fails = 0


def ok(msg):
    print(f"  OK    {msg}")


def bad(msg):
    global fails
    fails += 1
    print(f"  FAIL  {msg}")


def port_in_use(port):
    s = socket.socket()
    s.settimeout(0.5)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def import_in(folder, module, env_extra):
    env = dict(os.environ, **env_extra)
    r = subprocess.run([sys.executable, "-c", f"import {module}"], cwd=folder, env=env,
                       capture_output=True, text=True, timeout=60)
    if r.returncode == 0:
        ok(f"{os.path.basename(folder)}/{module} imports")
    else:
        last = [ln for ln in r.stderr.strip().splitlines() if ln.strip()][-1:] or ["unknown error"]
        bad(f"{os.path.basename(folder)}/{module} cannot start: {last[0]}")


def check():
    print("Python packages")
    for mod, pip in (("flask", "flask"), ("serial", "pyserial"), ("shapely", "shapely")):
        try:
            __import__(mod)
            ok(mod)
        except Exception:
            bad(f"{mod} is not installed -> pip install {pip}")

    print("Files")
    need = {
        LAPTOP: ["api_proxy.py", "radio_bridge.py", "config.py", "protocol.py", "request_manager.py", "mission_planner.py"],
        JETSON: ["radio_bridge.py", "config.py", "protocol.py"],
        HERE: ["virtual_radio.py", "fake_jetson.py"],
    }
    for folder, names in need.items():
        for n in names:
            p = os.path.join(folder, n)
            if os.path.isfile(p):
                ok(f"{os.path.basename(folder)}/{n}")
            else:
                hint = ""
                if n == "protocol.py":
                    hint = "  (each machine needs its own protocol.py next to its scripts; copy the full one from jetson/)"
                bad(f"missing {os.path.basename(folder)}/{n}{hint}")
    if any(os.path.isfile(os.path.join(LAPTOP, d, "index.html")) for d in ("", "templates", "frontend")):
        ok("laptop index.html")
    else:
        bad("index.html not found in laptop/, laptop/templates/ or laptop/frontend/")
    jp = os.path.join(JETSON, "protocol.py")
    if os.path.isfile(jp) and "parse_qgc_wpl" not in open(jp, encoding="utf-8", errors="ignore").read():
        bad("jetson/protocol.py has no parse_qgc_wpl - it must be the full protocol.py")

    print("Code starts without errors")
    import_in(LAPTOP, "api_proxy", {"GCS_RADIO_PORT": "socket://127.0.0.1:9001"})
    import_in(JETSON, "radio_bridge", {"GCS_RADIO_PORT": "socket://127.0.0.1:9002"})

    print("Ports free")
    for port, what in ((5000, "web page"), (9001, "virtual radio, laptop side"), (9002, "virtual radio, Jetson side")):
        if port_in_use(port):
            bad(f"port {port} ({what}) is already in use - close windows left over from an earlier run")
        else:
            ok(f"port {port} free")


def wait(port, seconds):
    end = time.time() + seconds
    while time.time() < end:
        if port_in_use(port):
            print(f"Port {port} is up.")
            return 0
        time.sleep(0.5)
    print(f"Nothing started listening on port {port} within {seconds} s.")
    return 1


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--wait":
        sys.exit(wait(int(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 30))
    check()
    print("\nAll good." if not fails else f"\n{fails} problem(s) found - fix the FAIL lines above.")
    sys.exit(1 if fails else 0)
