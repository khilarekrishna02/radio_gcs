#!/usr/bin/env python3
"""Virtual SiK radio pair for testing at home (no hardware, no drivers).

The laptop GCS connects to port 9001 and the "Jetson" to port 9002; every byte
one side sends is relayed to the other, exactly like two paired radios.

    python virtual_radio.py

Type in this window:   drop   -> cut both links (the bridges must reconnect by themselves)
                       status -> who is connected
                       quit   -> exit
"""
import socket
import threading

HOST = "127.0.0.1"
PORTS = {"laptop": 9001, "jetson": 9002}
OTHER = {"laptop": "jetson", "jetson": "laptop"}
conns = {"laptop": None, "jetson": None}
lock = threading.Lock()


def pump(name, sock):
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                break
            with lock:
                dst = conns[OTHER[name]]
            if dst is not None:          # like a real radio: nobody listening -> bytes are lost
                try:
                    dst.sendall(data)
                except OSError:
                    pass
    except OSError:
        pass
    finally:
        with lock:
            if conns[name] is sock:
                conns[name] = None
        try:
            sock.close()
        except OSError:
            pass
        print(f"[radio] {name} disconnected")


def serve(name, port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, port))
    srv.listen(1)
    while True:
        sock, _ = srv.accept()
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with lock:
            old, conns[name] = conns[name], sock
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        print(f"[radio] {name} connected")
        threading.Thread(target=pump, args=(name, sock), daemon=True).start()


def main():
    for name, port in PORTS.items():
        threading.Thread(target=serve, args=(name, port), daemon=True).start()
    print(f"[radio] virtual radio up: laptop -> socket://{HOST}:{PORTS['laptop']}   jetson -> socket://{HOST}:{PORTS['jetson']}")
    print("[radio] commands: drop | status | quit")
    while True:
        try:
            cmd = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == "quit":
            break
        if cmd == "status":
            with lock:
                print({k: (v is not None) for k, v in conns.items()})
        if cmd == "drop":
            with lock:
                socks = [s for s in conns.values() if s is not None]
            for s in socks:
                try:
                    s.close()
                except OSError:
                    pass
            print("[radio] links dropped - both ends should reconnect within a few seconds")


if __name__ == "__main__":
    main()
