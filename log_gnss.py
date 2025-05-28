from gnss_parser import GNSSParser
from datetime import datetime

# This script initializes a GNSS parser and logs GNSS data to a CSV file.
parser = GNSSParser(
    serial_port='COM5',
    baud_rate=115200,
    ntrip_caster='192.148.213.42',
    ntrip_port=2101,
    mountpoint='VRS3M',
    username='vicentedf88',
    password='Danvila1999'
)

from datetime import datetime

def main():
    try:
        # Initialize the GNSS parser
        parser.start()
        print("GNSS parser launched successfully.")
    except Exception as e:
        print(f"Error launching GNSS parser: {e}")
        return

    while True:
        try:
            print("Waiting for GNSS data...")
            # Read the GNSS data from a file
            pos = parser.read_position()
            heading = parser.read_heading()
            course = parser.read_course_speed()

            # Print the GNSS data
            if pos is None or heading is None or course is None:
                print("No GNSS data available yet.")
                continue
            init_time = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            filename = f"gnss_data_{init_time}.csv"
            data_line = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}]  Latitude={pos["latitude"]}, Longitude={pos["longitude"]}, Altitude={pos["altitude"]}, Heading={heading}, COG={course['course']}, SOG={course['speed']}"
            print(data_line)

            with open(filename, 'a') as file:
                file.write(data_line + '\n')
        except KeyboardInterrupt:
            print("Exiting GNSS data logging.")
            break
        except Exception as e:
            print(f"Error reading GNSS data: {e}")
        finally:
            # On shutdown
            parser.stop()
            print("GNSS parser closed.")

if __name__ == "__main__":
    main()
