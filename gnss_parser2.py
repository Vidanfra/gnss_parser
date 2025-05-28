import socket
import base64
import serial
import threading
import pynmea2
import time

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
    """Monitor NTRIP corrections and relay GGA position data"""
    while True:
        try:
            line = serial_port.readline().decode(errors='ignore').strip()
            if line.startswith("$G") and line[3:6] == "GGA":
                try:
                    msg = pynmea2.parse(line)
                    # Relay GGA to NTRIP (some VRS services require it)
                    print(f"NMEA GGA: latitude: {msg.latitude}, longitude: {msg.longitude}, status: {msg.status}")
                    # Forward GGA to NTRIP caster
                    ntrip_sock.sendall((line + "\r\n").encode())
                except pynmea2.nmea.ChecksumError:
                    pass
        except Exception as e:
            print(f"NMEA read error: {e}")
            break
def stream_position(serial_port, frequency=1):
    """Stream position data at a specified frequency"""
    try:
        command = f'GPGGA {frequency}\r\n'
        serial_port.write(command.encode()) #binary
        print(f"Streaming position data at {frequency} Hz.")
    except Exception as e:
        print(f"Error streaming position: {e}")

def stream_heading(serial_port, frequency=1):
    """Stream heading data at a specified frequency"""
    try:
        command = f'GPTHS {frequency}\r\n'
        serial_port.write(command.encode()) #binary
        print(f"Streaming heading data at {frequency} Hz.")
    except Exception as e:
        print(f"Error streaming heading: {e}")
    
def read_position(line):
    """Read position from serial port if available"""
    if line.startswith("$G") and line[3:6] == "GGA":
        try:
            msg = pynmea2.parse(line)
            print(f"NMEA: Fix={msg.gps_qual}, Lat={msg.latitude}, Lon={msg.longitude}")
            return msg.latitude, msg.longitude
        except pynmea2.nmea.ParseError:
            print(f"NMEA parse error: {line}")
            return None
        except pynmea2.nmea.ChecksumError:
            print(f"NMEA checksum error: {line}")
            return None
        except Exception as e:
            print(f"NMEA read error: {e}")
            return None

def read_heading(line):
    """Read heading from serial port if available"""
    if line.startswith("$G") and line[3:6] == "THS":
        # Strip checksum
        data = line.split('*')[0]
        fields = data.split(',')
        if len(fields) < 3:
            return None
        # Assuming the format is $GNGTHS,heading,status
 
        heading = fields[1]
        status = fields[2]
        print(f"NMEA GNTHS: Heading: {heading}°, Status: {status}")

        if status.upper() != 'A':  # 'A' usually means valid
            return None
        return float(heading)

def main():
    try:
        ntrip_sock = connect_to_ntrip()
    except Exception as e:
        print(f"NTRIP connection failed: {e}")
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")

            # Stream position and heading data
            stream_position(ser, frequency=1)
            stream_heading(ser, frequency=1)
            time.sleep(1)

            # Start RTCM forwarding in background
            threading.Thread(target=forward_rtcm, args=(ntrip_sock, ser), daemon=True).start()

            # Monitor NTRIP corrections and forward GGA postion data for VRS
            threading.Thread(target=ntrip_monitor, args=(ser, ntrip_sock), daemon=True).start()

            # Read position and heading in the main thread
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                read_position(line)
                read_heading(line)
                time.sleep(1)


    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user, closing connections.")
    finally:

        if ntrip_sock:
            ntrip_sock.close()
            print("NTRIP socket closed.")
        print("Exiting GNSS parser.")

if __name__ == "__main__":
    main()


