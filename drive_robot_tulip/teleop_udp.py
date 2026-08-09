import socket
import json
import struct
import sys
import threading
import time

UDP_IP = "192.168.4.1"
UDP_PORT = 8888

# ========== KEYBOARD (cross-platform) ==========
if sys.platform == "win32":
    import msvcrt as msvcrt_mod

    def getch():
        return msvcrt_mod.getch().decode("utf-8", errors="ignore")

    def kbhit():
        return msvcrt_mod.kbhit()
else:
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def kbhit():
        import select
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def restore_term():
    if sys.platform != "win32":
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ========== UDP ==========
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.05)
sock.bind(("0.0.0.0", 0))
local_port = sock.getsockname()[1]

running = True
linear = 0.0
angular = 0.0

def recv_loop():
    global running
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            try:
                imu = json.loads(data.decode())
                r = imu.get("roll", 0)
                p = imu.get("pitch", 0)
                y = imu.get("yaw", 0)
                print(f"\r  roll={r:6.1f}  pitch={p:6.1f}  yaw={y:6.1f}  "
                      f"linear={linear:4.2f}  angular={angular:4.2f}  ", end="", flush=True)
            except json.JSONDecodeError:
                pass
        except socket.timeout:
            pass
        except OSError:
            break

print("=" * 50)
print("  TULIP ESP32 - UDP Teleop")
print("=" * 50)
print(f"  Target: {UDP_IP}:{UDP_PORT}")
print()
print("  WASD / Arrow keys  :  gerak")
print("  Spasi              :  stop")
print("  Q / ESC            :  keluar")
print("=" * 50)
print()

t = threading.Thread(target=recv_loop, daemon=True)
t.start()

try:
    while running:
        if kbhit():
            ch = getch().lower()
            if ch == "w" or ch == "\x00H":
                linear = min(linear + 0.2, 1.0)
            elif ch == "s" or ch == "\x00P":
                linear = max(linear - 0.2, -1.0)
            elif ch == "a" or ch == "\x00K":
                angular = max(angular - 0.3, -1.0)
            elif ch == "d" or ch == "\x00M":
                angular = min(angular + 0.3, 1.0)
            elif ch == " ":
                linear = 0.0
                angular = 0.0
            elif ch == "q" or ch == "\x1b":
                break
            # Arrow keys on Linux send escape sequences
            if ch == "\x1b":
                ch2 = getch()
                ch3 = getch()
                if ch2 == "[":
                    if ch3 == "A":
                        linear = min(linear + 0.2, 1.0)
                    elif ch3 == "B":
                        linear = max(linear - 0.2, -1.0)
                    elif ch3 == "D":
                        angular = max(angular - 0.3, -1.0)
                    elif ch3 == "C":
                        angular = min(angular + 0.3, 1.0)

        msg = json.dumps({"linear": round(linear, 2), "angular": round(angular, 2)})
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.15)

except KeyboardInterrupt:
    pass
finally:
    sock.sendto(json.dumps({"linear": 0, "angular": 0}).encode(), (UDP_IP, UDP_PORT))
    running = False
    restore_term()
    sock.close()
    print("\n\ndone.")
