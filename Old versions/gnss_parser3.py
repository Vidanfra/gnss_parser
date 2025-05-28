import socket
import base64
import serial
import threading
import pynmea2
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import geopandas as gpd
import contextily as ctx

# === NTRIP CASTER SETTINGS ===
NTRIP_CASTER = '192.148.213.42'
NTRIP_PORT = 2101
MOUNTPOINT = 'VRS3M'
USERNAME = 'vicentedf88'
PASSWORD = 'Danvila1999'

# === SERIAL SETTINGS ===
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

# === DATA STORAGE ===
latitudes = []
longitudes = []
headings = []
courses = []
speeds = []
timestamps = []  # seconds since start
start_time = time.time()  # for timestamping

# === NMEA PARSERS ===
def read_position(line):
    if line[3:6] == "GGA":
        try:
            msg = pynmea2.parse(line)
            print(f"[GGA] Fix={msg.gps_qual}, Lat={msg.latitude}, Lon={msg.longitude}, HDOP={msg.horizontal_dil}, Sats={msg.num_sats}")
            if msg.gps_qual < 1:
                print("No valid GPS fix.")
                return
            latitudes.append(msg.latitude)
            longitudes.append(msg.longitude)
            timestamps.append(time.time() - start_time)
        except pynmea2.nmea.ParseError:
            print(f"GGA parse error: {line}")


def read_heading(line):
    if line[3:6] == "THS":
        try:
            fields = line.split('*')[0].split(',')
            heading = fields[1]
            heading = float(heading) if heading else 0.0
            status = fields[2]
            if status.upper() == 'A':
                print(f"[THS] Heading: {heading:.2f}°")
                headings.append(heading)
        except (ValueError, IndexError):
            print(f"THS parse error: {line}")


def read_course_speed(line):
    if line[3:6] == "VTG":
        try:
            msg = pynmea2.parse(line)
            if hasattr(msg, 'speed_knots'):
                sog = float(msg.speed_knots)
                speeds.append(sog)
                print(f"[VTG] SOG: {sog} knots")
            if hasattr(msg, 'true_track') and msg.true_track:
                cog = float(msg.true_track)
                courses.append(cog)
                print(f"[VTG] COG: {cog} deg")
        except pynmea2.nmea.ParseError:
            print(f"VTG parse error: {line}")


def dispatch_nmea(line, ntrip_sock=None):
    """Dispatch NMEA sentences to appropriate handlers."""
    if not line.startswith("$"):
        return
    sentence_type = line[3:6]

    if sentence_type == "GGA":
        read_position(line)
        if ntrip_sock:
            try:
                ntrip_sock.sendall((line + "\r\n").encode())
            except Exception as e:
                print(f"Failed to send GGA to caster: {e}")
    elif sentence_type == "THS":
        read_heading(line)
    elif sentence_type == "VTG":
        read_course_speed(line)


# === NTRIP HANDLING ===
def connect_to_ntrip():
    credentials = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()
    request = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"User-Agent: NTRIP PythonClient\r\n"
        f"Authorization: Basic {credentials}\r\n"
        f"\r\n"
    )

    sock = socket.create_connection((NTRIP_CASTER, NTRIP_PORT))
    sock.sendall(request.encode())

    header = b""
    while b"\r\r" not in header and b"\r\n\r\n" not in header:
        header += sock.recv(1)

    print("Connected to NTRIP caster and header skipped.")
    return sock


def forward_rtcm(ntrip_sock, serial_port):
    while True:
        try:
            data = ntrip_sock.recv(1024)
            if not data:
                print("Disconnected from NTRIP caster.")
                break
            serial_port.write(data)
        except Exception as e:
            print(f"RTCM error: {e}")
            break


# === SERIAL READER LOOP ===
def serial_reader_loop(serial_port, ntrip_sock):
    while True:
        try:
            line = serial_port.readline().decode(errors='ignore').strip()
            if line.startswith("$G"):
                dispatch_nmea(line, ntrip_sock)
        except Exception as e:
            print(f"Serial read error: {e}")
            return


# === STREAM CONFIG COMMANDS ===
def stream_position(serial_port, frequency=1):
    try:
        serial_port.write(f'GPGGA {frequency}\r\n'.encode())
        print(f"Requested GPGGA at {frequency} Hz")
    except Exception as e:
        print(f"Error starting GGA stream: {e}")


def stream_heading(serial_port, frequency=1):
    try:
        serial_port.write(f'GPTHS {frequency}\r\n'.encode())
        print(f"Requested GPTHS at {frequency} Hz")
    except Exception as e:
        print(f"Error starting THS stream: {e}")

def stream_course_speed(serial_port, frequency=1):
    try:
        serial_port.write(f'GPVTG {frequency}\r\n'.encode())
        print(f"Requested GPVTG at {frequency} Hz")
    except Exception as e:
        print(f"Error starting VTG stream: {e}")

# === REAL-TIME MAP ===
# Global reference to axes
fig, ax = plt.subplots(figsize=(8, 8))

def animate_map(i):
    ax.clear()
    if latitudes and longitudes:
        gdf = gpd.GeoDataFrame(
            {'geometry': gpd.points_from_xy(longitudes, latitudes)},
            crs='EPSG:4326'
        ).to_crs(epsg=3857)
        gdf.plot(ax=ax, marker='o', color='red', markersize=5)

        # Get bounds and apply padding
        xmin, ymin, xmax, ymax = gdf.total_bounds
        x_pad = (xmax - xmin) * 0.1  # fallback pad in meters
        y_pad = (ymax - ymin) * 0.1 
        if x_pad < 50: 
            x_pad = 50
        if y_pad < 50:
            y_pad = 50

        ax.set_xlim(xmin - x_pad, xmax + x_pad)
        ax.set_ylim(ymin - y_pad, ymax + y_pad)

        ctx.add_basemap(ax, source=ctx.providers.Esri.WorldImagery)
        ax.set_title("Real-time GNSS Position on Satellite Map")
        ax.set_axis_off()


# === MAIN ENTRY POINT ===
def main():
    try:
        ntrip_sock = connect_to_ntrip()
    except Exception as e:
        print(f"NTRIP connection failed: {e}")
        return
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")

            # Start streaming commands to GNSS receiver
            stream_position(ser)
            stream_heading(ser)
            stream_course_speed(ser)
            # Allow some time for the GNSS receiver to start streaming
            time.sleep(1)

            # Start RTCM forwarding and serial reading in background threads
            threading.Thread(target=forward_rtcm, args=(ntrip_sock, ser), daemon=True).start()
            threading.Thread(target=serial_reader_loop, args=(ser, ntrip_sock), daemon=True).start()

            # Set up real-time plotting
            ani = animation.FuncAnimation(fig, animate_map, interval=1000)
            plt.show()

    except serial.SerialException as e:
        print(f"Serial error in main: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user, closing.")
    finally:
        if ntrip_sock:
            ntrip_sock.close()
            print("Closed NTRIP connection.")

if __name__ == "__main__":
    main()
