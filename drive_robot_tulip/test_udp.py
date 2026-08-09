import json
import select
import socket
import sys
import time

if len(sys.argv) < 2:
    print("Usage: python3 test_udp.py <ESP32_IP>")
    print("Example: python3 test_udp.py 192.168.1.100")
    sys.exit(1)

esp_ip = sys.argv[1]
esp_port = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.3)
sock.bind(("0.0.0.0", 0))

def send(linear, angular):
    msg = json.dumps({"linear": linear, "angular": angular})
    sock.sendto(msg.encode(), (esp_ip, esp_port))

print("\n=== TULIP UDP TEST ===")
print(f"Target: {esp_ip}:{esp_port}\n")
print("  w=maju   s=mundur   a=kiri   d=kanan")
print("  spasi=stop   q=quit\n")

last_cmd = time.time()

try:
    while True:
        r, _, _ = select.select([sys.stdin, sock], [], [], 0.1)

        if sys.stdin in r:
            ch = sys.stdin.read(1).lower()
            if ch == "w":
                send(0.3, 0.0); print(">> maju   linear=0.3")
            elif ch == "s":
                send(-0.3, 0.0); print(">> mundur linear=-0.3")
            elif ch == "a":
                send(0.0, 0.5); print(">> kiri   angular=0.5")
            elif ch == "d":
                send(0.0, -0.5); print(">> kanan  angular=-0.5")
            elif ch == " ":
                send(0.0, 0.0); print(">> STOP")
            elif ch == "q":
                break
            last_cmd = time.time()

        if sock in r:
            try:
                data, _ = sock.recvfrom(1024)
                imu = json.loads(data.decode())
                print(f"  IMU  roll={imu['roll']:7.1f}  pitch={imu['pitch']:7.1f}  yaw={imu['yaw']:7.1f}")
            except (json.JSONDecodeError, KeyError):
                pass

        if time.time() - last_cmd > 0.8:
            send(0.0, 0.0)
            last_cmd = time.time()

except KeyboardInterrupt:
    pass
finally:
    send(0.0, 0.0)
    sock.close()
    print("Done.")
