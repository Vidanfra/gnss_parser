import socket
import base64
import serial
import threading
import pynmea2

# === NTRIP CASTER SETTINGS ===
NTRIP_CASTER = '192.148.213.42'
NTRIP_PORT = 2101
MOUNTPOINT = 'VRS3M'
USERNAME = 'vicentedf88'
PASSWORD = 'Danvila1999'

# === SERIAL SETTINGS ===
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

def connect_to_ntrip():
    """Connect to the NTRIP caster and skip ICY header"""
    credentials = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()
    request = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"User-Agent: NTRIP PythonClient\r\n"
        f"Authorization: Basic {credentials}\r\n"
        f"\r\n"
    )

    sock = socket.create_connection((NTRIP_CASTER, NTRIP_PORT))
    sock.sendall(request.encode())

    # Skip ICY 200 OK header — read until empty line
    header = b""
    while b"\r\n\r\n" not in header:
        header += sock.recv(1)

    print("Connected to NTRIP caster and header skipped.")
    return sock

def forward_rtcm(ntrip_sock, serial_port):
    """Continuously forward RTCM3 data from caster to GNSS receiver"""
    while True:
        try:
            data = ntrip_sock.recv(1024)
            if not data:
                print("Disconnected from NTRIP caster.")
                break
            serial_port.write(data)
        except Exception as e:
            print(f"Error forwarding RTCM: {e}")
            break

def ntrip_monitor(serial_port, ntrip_sock):
    """Monitor NMEA output and relay GGA back to caster"""
    while True:
        try:
            line = serial_port.readline().decode(errors='ignore').strip()
            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                try:
                    msg = pynmea2.parse(line)
                    print(f"NMEA: Fix={msg.gps_qual}, Lat={msg.latitude}, Lon={msg.longitude}")
                    # Relay GGA to NTRIP (some VRS services require it)
                    ntrip_sock.sendall((line + "\r\n").encode())
                except pynmea2.nmea.ChecksumError:
                    pass
        except Exception as e:
            print(f"NMEA read error: {e}")
            break

def main():
    try:
        ntrip_sock = connect_to_ntrip()
    except Exception as e:
        print(f"NTRIP connection failed: {e}")
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")

            # Start RTCM forwarding in background
            threading.Thread(target=forward_rtcm, args=(ntrip_sock, ser), daemon=True).start()

            # Monitor and relay NMEA (especially GGA)
            ntrip_monitor(ser, ntrip_sock)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    main()
