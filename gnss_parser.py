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

# === DATA STORAGE ===
gga_data = []
ths_data = []
vtg_data = []
zda_data = []
rmc_slave_data = []

# === GLOBAL STATE ===
running = True


# === NMEA PARSERS ===
def parse_gpgga(line): 
    '''GGA sentence parsing: utc, lat, lat_dir, lon, lon_dir, fix_quality, num_sats, hdop, 
    altitude, alt_unit, geoid_sep, geoid_unit, age_gps_data, ref_station_id'''
    if line[3:6] == "GGA":
        try:
            msg = pynmea2.parse(line)
            #print(f"[GGA] Fix={msg.gps_qual}, Lat={msg.latitude}, Lon={msg.longitude}, HDOP={msg.horizontal_dil}, Sats={msg.num_sats}")
            if msg.gps_qual < 1:
                print("No valid GPS fix.")
            gga_data = msg
        except pynmea2.nmea.ParseError:
            print(f"GGA parse error: {line}")


def parse_gpths(line):
    '''GPTHS - True Heading and Status sentence parsing: heading, status'''
    if line[3:6] == "THS":
        try:
            fields = line.split('*')[0].split(',')
            heading = fields[1]
            heading = float(heading) if heading else 0.0
            status = fields[2]
            #if status.upper() == 'A':
                #print(f"[THS] Heading: {heading:.2f}°")
            ths_data = {
                'heading': heading,
                'status': status
            }
        except (ValueError, IndexError):
            print(f"THS parse error: {line}")

def parse_gpvtg(line):
    '''GPVTG - Course Over Ground and Speed sentence parsing: course_true, course indicator (T), course_mag, course indicator (M), speed_knots,  speed_knots_unit (N), speed_kph, speed_kph_unit (K), mode indicator'''
    if line[3:6] == "VTG":
        try:
            msg = pynmea2.parse(line)
            if hasattr(msg, 'speed_knots'):
                sog = float(msg.speed_knots)
                #print(f"[VTG] SOG: {sog} knots")
            if hasattr(msg, 'true_track') and msg.true_track:
                cog = float(msg.true_track)
                #print(f"[VTG] COG: {cog} deg")
            vtg_data = msg
        except pynmea2.nmea.ParseError:
            print(f"VTG parse error: {line}")

def parse_gpzda(line):
    '''GPZDA - Time and Date sentence parsing: utc, day, month, year, local_zone_hour, local_zone_minute'''
    if line[3:6] == "ZDA":
        try:
            msg = pynmea2.parse(line)
            #print(f"[ZDA] UTC: {msg.utc}, Date: {msg.day}/{msg.month}/{msg.year}")
            zda_data = msg
        except pynmea2.nmea.ParseError:
            print(f"ZDA parse error: {line}")

def parse_gprmch(line):
    '''GPRMCH - RMC Slave Antenna sentence parsing: utc, status, lat, lat_dir, lon, lon_dir, speed_knots, course_true, date, magnetic_variation, mag_var_dir, mode_indicator, mode_status'''
    if line[3:6] == "RMC" and line[7] == 'H':
        try:
            msg = pynmea2.parse(line)
            #print(f"[RMC] Lat: {msg.latitude}, Lon: {msg.longitude}, Speed: {msg.speed_knots} knots")
            rmc_slave_data = msg
        except pynmea2.nmea.ParseError:
            print(f"RMC Slave antenna parse error: {line}")


def dispatch_nmea(line, ntrip_sock=None):
    """Dispatch NMEA sentences to appropriate handlers."""
    if not line.startswith("$"):
        return
    sentence_type = line[3:6]

    if sentence_type == "GGA":
        parse_gpgga(line)
        # If GGA is parsed, send it to NTRIP caster if connected
        if ntrip_sock:
            try:
                ntrip_sock.sendall((line + "\r\n").encode())
            except Exception as e:
                print(f"Failed to send GGA to caster: {e}")
    elif sentence_type == "THS":
        parse_gpths(line)
    elif sentence_type == "VTG":
        parse_gpvtg(line)
    elif sentence_type == "ZDA":
        parse_gpzda(line)
    elif sentence_type == "RMC" and line[7] == 'H':
        parse_gprmch(line)

def read_gga():
    """Read GGA data from the stored list."""
    return gga_data
def read_ths():
    """Read THS data from the stored list."""
    return ths_data
def read_vtg():
    """Read VTG data from the stored list."""
    return vtg_data
def read_zda():
    """Read ZDA data from the stored list."""
    return zda_data
def read_rmch():
    """Read RMC Slave Antenna data from the stored list."""
    return rmc_slave_data
def read_position():
    """Read position data from the stored GGA data."""
    if gga_data:
        return {
            'latitude': gga_data.latitude,
            'longitude': gga_data.longitude,
            'altitude': gga_data.altitude
        }
    return None
def read_heading():
    """Read heading data from the stored THS data."""
    if ths_data:
        return ths_data['heading']
    return None
def read_course_speed():
    """Read course and speed data from the stored VTG data."""
    if vtg_data:
        return {
            'course': vtg_data.true_track,
            'speed': vtg_data.speed_knots
        }
    return None
def read_time_date():
    """Read time and date data from the stored ZDA data."""
    if zda_data:
        return {
            'utc': zda_data.utc,
            'day': zda_data.day,
            'month': zda_data.month,
            'year': zda_data.year
        }
    return None
def read_rmc_slave():
    """Read Recommended Minimum GNSS Slave Antenna data from the stored RMC data."""
    if rmc_slave_data:
        return {
            'latitude': rmc_slave_data.latitude,
            'longitude': rmc_slave_data.longitude,
            'speed_knots': rmc_slave_data.speed_knots,
            'course_true': rmc_slave_data.true_track
        }
    return None


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
    # small delay to ensure port is ready
    time.sleep(0.5)
    while running:
        if not serial_port.is_open:
            print("Serial port is not open. Stopping reader loop.")
            break
        try:
            raw_line = serial_port.readline()
            if not raw_line:
                continue
            try:
                line = raw_line.decode(errors='ignore').strip()
            except Exception as decode_e:
                print(f"Decode error: {decode_e}")
                continue
            if line.startswith("$G"):
                dispatch_nmea(line, ntrip_sock)
        except Exception as e:
            print(f"Serial reader loop error: {e}")
            time.sleep(1)  # Wait before retrying
    try:
        serial_port.close()
        print("Closed serial port.")
    except Exception:
        pass
    print("Exiting serial reader loop.")

# === RTCM FORWARDING LOOP ===
def forward_rtcm(ntrip_sock, serial_port):
    while running:
        try:
            data = ntrip_sock.recv(1024)
            if not data:
                print("No RTCM data received. Stopping forward loop.")
                break
            serial_port.write(data)
        except Exception as e:
            print(f"RTCM forward error: {e}")
            time.sleep(1)
    try:
        ntrip_sock.close()
        print("Closed NTRIP connection.")
    except Exception:
        pass
    print("Exiting RTCM forward loop.")

# === STREAM CONFIG COMMANDS ===
def stream_nmea(serial_port, command, frequency):
    """Send command to GNSS receiver to start streaming NMEA sentences."""
    command = command.upper()
    try:
        if frequency == 0:
            serial_port.write(f'{command}\r\n'.encode())
            print(f"Requested {command}")
        else:
            serial_port.write(f'{command} {frequency}\r\n'.encode())
            print(f"Requested {command} at {frequency} Hz")
    except Exception as e:
        print(f"Error starting {command} stream: {e}")

# === GNSS PARSER MODULE ===
def launch_gnss_parser():
    """Launch the GNSS parser module."""
    try:
        ntrip_sock = connect_to_ntrip()
        print("Connected to NTRIP caster and header skipped.")
    except Exception as e:
        print(f"NTRIP connection failed: {e}")
        return


    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")

        # Send NMEA stream commands
        stream_nmea(ser, 'GPGGA', 1)
        stream_nmea(ser, 'GPTHS', 1)
        stream_nmea(ser, 'GPVTG', 1)

        # Give receiver time to start
        time.sleep(1)

        # Start background threads
        reader_thread = threading.Thread(target=serial_reader_loop, args=(ser, ntrip_sock))
        rtcm_thread = threading.Thread(target=forward_rtcm, args=(ntrip_sock, ser))
        reader_thread.start()
        rtcm_thread.start()

def close_gnss_parser():
    """Close the GNSS parser module."""
    running = False

# === MAIN ENTRY POINT ===
def main():
    try:
        ntrip_sock = connect_to_ntrip()
        print("Connected to NTRIP caster and header skipped.")
    except Exception as e:
        print(f"NTRIP connection failed: {e}")
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")

            # Send NMEA stream commands
            stream_nmea(ser, 'GPGGA', 1)
            stream_nmea(ser, 'GPTHS', 1)
            stream_nmea(ser, 'GPVTG', 1)

            # Give receiver time to start
            time.sleep(1)

            # Start background threads
            reader_thread = threading.Thread(target=serial_reader_loop, args=(ser, ntrip_sock))
            rtcm_thread = threading.Thread(target=forward_rtcm, args=(ntrip_sock, ser))
            reader_thread.start()
            rtcm_thread.start()

            # Main loop: wait for user interrupt
            try:
                while running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("KeyboardInterrupt received. Stopping...")
                running = False

            # Wait for threads to finish
            reader_thread.join()
            rtcm_thread.join()

    except Exception as e:
        print(f"Serial setup failed: {e}")
    finally:
        running = False
        try:
            ntrip_sock.close()
            print("Closed NTRIP connection.")
        except Exception:
            pass

#if __name__ == "__main__":
#    main()
