import socket
import base64
import serial
import threading
import pynmea2
import time
from datetime import datetime


class GNSSParser:
    """
    GNSSParser handles NMEA streaming from a GNSS receiver over serial,
    forwards RTCM corrections via NTRIP, and parses selected NMEA sentences.
    """
    def __init__(
        self,
        serial_port: str,
        baud_rate: int,
        ntrip_caster: str,
        ntrip_port: int,
        mountpoint: str,
        username: str,
        password: str,
        command_freq: float,
        timeout: float = 1.0,
        
    ):
        # Serial and NTRIP settings
        self.serial_port_name = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ntrip_caster = ntrip_caster
        self.ntrip_port = ntrip_port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.command_freq = command_freq

        # Runtime handles
        self.serial_port = None
        self.ntrip_sock = None
        self.running = False
        self.reader_thread = None
        self.rtcm_thread = None

        # Parsed data storage
        self.gga_data = None
        self.ths_data = None
        self.vtg_data = None
        self.zda_data = None
        self.rmc_slave_data = None

    def start(self):
        """Start serial reading and RTCM forwarding."""
        self.running = True
        try:
            # Connect to NTRIP caster
            self.ntrip_sock = self._connect_to_ntrip()
            print("Connected to NTRIP caster and header skipped.")

            # Open serial port
            self.serial_port = serial.Serial(
                self.serial_port_name,
                self.baud_rate,
                timeout=self.timeout,
            )
            print(f"Opened serial port {self.serial_port_name} at {self.baud_rate} baud.")

            # Send NMEA stream commands
            self._send_stream_commands()

            # Delay for receiver startup
            time.sleep(1)

            # Launch background threads
            self.reader_thread = threading.Thread(
                target=self._reader_loop, daemon=True
            )
            self.rtcm_thread = threading.Thread(
                target=self._rtcm_loop, daemon=True
            )
            self.reader_thread.start()
            self.rtcm_thread.start()

        except Exception as e:
            print(f"GNSS parser start failed: {e}")
            self.stop()

    def stop(self):
        """Stop threads and close connections."""
        self.running = False

        # Close serial port
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                print("Serial port closed.")
        except Exception:
            pass

        # Close NTRIP socket
        try:
            if self.ntrip_sock:
                self.ntrip_sock.close()
                print("NTRIP connection closed.")
        except Exception:
            pass

    def _connect_to_ntrip(self):
        credentials = base64.b64encode(
            f"{self.username}:{self.password}".encode()
        ).decode()
        request = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP PythonClient\r\n"
            f"Authorization: Basic {credentials}\r\n"
            f"\r\n"
        )
        sock = socket.create_connection((self.ntrip_caster, self.ntrip_port))
        sock.sendall(request.encode())

        header = b""
        while b"\r\n\r\n" not in header:
            header += sock.recv(1)
        return sock

    def _send_stream_commands(self):
        commands = [
            ("GPGGA", self.command_freq),
            ("GPTHS", self.command_freq),
            ("GPVTG", self.command_freq),
            #("GPZDA", self.command_freq),
            #("GPRMCH", self.command_freq),
        ]
        for cmd, freq in commands:
            self._stream_nmea(cmd, freq)

    def _stream_nmea(self, command: str, frequency: int):
        try:
            sentence = f"{command.upper()} {frequency}\r\n"
            self.serial_port.write(sentence.encode())
            print(f"Requested {command} at {frequency} Hz")
        except Exception as e:
            print(f"Error starting {command} stream: {e}")

    def _reader_loop(self):
        # Allow internal handles to initialize
        time.sleep(0.5)
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue
                line = raw.decode(errors='ignore').strip()
                if line.startswith("$G"):
                    self._dispatch_nmea(line)
            except Exception as e:
                print(f"Serial reader error: {e}")
                time.sleep(1)
        print("Exiting reader loop.")

    def _rtcm_loop(self):
        while self.running and self.ntrip_sock:
            try:
                data = self.ntrip_sock.recv(1024)
                if not data:
                    break
                self.serial_port.write(data)
            except Exception as e:
                print(f"RTCM error: {e}")
                time.sleep(1)
        print("Exiting RTCM loop.")

    def _dispatch_nmea(self, line: str):
        typ = line[3:6]
        if typ == "GGA":
            self._parse_gpgga(line)
            self._send_to_ntrip(line)
        elif typ == "THS":
            self._parse_gpths(line)
        elif typ == "VTG":
            self._parse_gpvtg(line)
        elif typ == "ZDA":
            self._parse_gpzda(line)
        elif typ == "RMC" and len(line) > 7 and line[7] == 'H':
            self._parse_gprmch(line)

    def _send_to_ntrip(self, line: str):
        try:
            if self.ntrip_sock:
                self.ntrip_sock.sendall((line + "\r\n").encode())
        except Exception as e:
            print(f"Failed to send GGA to NTRIP: {e}")

    # === NMEA parsing methods ===
    def _parse_gpgga(self, line: str):
        '''Parse GPGGA sentence and store data in gga_data attribute.
        (('Timestamp', 'timestamp', <function timestamp at 0x000002D6D9D38E00>), ('Latitude', 'lat'), 
        ('Latitude Direction', 'lat_dir'), ('Longitude', 'lon'), ('Longitude Direction', 'lon_dir'), 
        ('GPS Quality Indicator', 'gps_qual', <class 'int'>), ('Number of Satellites in use', 'num_sats'), 
        ('Horizontal Dilution of Precision', 'horizontal_dil'), ('Antenna Alt above sea level (mean)', 'altitude', <class 'float'>), 
        ('Units of altitude (meters)', 'altitude_units'), ('Geoidal Separation', 'geo_sep'), ('Units of Geoidal Separation (meters)', 'geo_sep_units'), 
        ('Age of Differential GPS Data (secs)', 'age_gps_data'), ('Differential Reference Station ID', 'ref_station_id'))'''
        try:
            msg = pynmea2.parse(line)
            if msg.gps_qual < 1:
                print("No valid GPS fix.")
            self.gga_data = msg
            #print(msg.fields)
            #print(msg.data)
        except pynmea2.nmea.ParseError:
            print(f"GGA parse error: {line}")

    def _parse_gpths(self, line: str):
        try:
            fields = line.split('*')[0].split(',')
            heading = float(fields[1]) if fields[1] else 0.0
            status = fields[2] if len(fields) > 2 else ''
            self.ths_data = {'heading': heading, 'status': status}
            #print(self.ths_data)
        except Exception:
            print(f"THS parse error: {line}")

    def _parse_gpvtg(self, line: str):
        '''Parse GPVTG sentence and store data in vtg_data attribute.
        (('True Track made good', 'true_track', <class 'float'>), ('True Track made good symbol', 'true_track_sym'), 
        ('Magnetic Track made good', 'mag_track', <class 'decimal.Decimal'>), ('Magnetic Track symbol', 'mag_track_sym'), 
        ('Speed over ground knots', 'spd_over_grnd_kts', <class 'decimal.Decimal'>), ('Speed over ground symbol', 'spd_over_grnd_kts_sym'), 
        ('Speed over ground kmph', 'spd_over_grnd_kmph', <class 'float'>), ('Speed over ground kmph symbol', 'spd_over_grnd_kmph_sym'), ('FAA mode indicator', 'faa_mode'))
        '''
        try:
            msg = pynmea2.parse(line)
            self.vtg_data = msg
            #print(msg.fields)
            #print(msg.data)
        except pynmea2.nmea.ParseError:
            print(f"VTG parse error: {line}")

    def _parse_gpzda(self, line: str):
        try:
            msg = pynmea2.parse(line)
            self.zda_data = msg
        except pynmea2.nmea.ParseError:
            print(f"ZDA parse error: {line}")

    def _parse_gprmch(self, line: str):
        try:
            msg = pynmea2.parse(line)
            self.rmc_slave_data = msg
        except pynmea2.nmea.ParseError:
            print(f"RMC parse error: {line}")

    # === Public accessors ===
    def get_gga(self):
        return self.gga_data

    def get_ths(self):
        return self.ths_data

    def get_vtg(self):
        return self.vtg_data

    def get_zda(self):
        return self.zda_data

    def get_rmc_slave(self):
        return self.rmc_slave_data

    def get_position(self):
        if self.gga_data:
            return {
                'latitude': self.gga_data.latitude,
                'longitude': self.gga_data.longitude,
                'altitude': self.gga_data.altitude,
                'gnss_quality': self.gga_data.gps_qual,
            }
        return None

    def get_heading(self):
        if isinstance(self.ths_data, dict):
            return self.ths_data.get('heading')
        return None

    def get_course_speed(self):
        if self.vtg_data:
            return {
                'course': self.vtg_data.true_track,
                'speed_kmh': self.vtg_data.spd_over_grnd_kmph,
            }
        return None

    def get_time_date(self):
        if self.zda_data:
            return {
                'utc': self.zda_data.utc,
                'day': self.zda_data.day,
                'month': self.zda_data.month,
                'year': self.zda_data.year,
            }
        return None


# === Example usage ===
if __name__ == '__main__':
    parser = GNSSParser(
        serial_port='COM5',
        baud_rate=115200,
        ntrip_caster='192.148.213.42',
        ntrip_port=2101,
        mountpoint='VRS3M',
        username='vicentedf88',
        password='Danvila1999',
        commands_freq=1,
    )
    parser.start()
    try:
        while True:
            time.sleep(2)
            pos = parser.get_position()
            if pos:
                print(pos)
    except KeyboardInterrupt:
        parser.stop()
        print("GNSS parser stopped.")