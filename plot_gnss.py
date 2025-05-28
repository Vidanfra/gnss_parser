import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import numpy as np
import time
from gnss_parser import GNSSParser

# === GNSS Parser Initialization ===
parser = GNSSParser(
    serial_port='COM5',
    baud_rate=115200,
    ntrip_caster='192.148.213.42',
    ntrip_port=2101,
    mountpoint='VRS3M',
    username='vicentedf88',
    password='Danvila1999'
)

# === Data Buffers ===
time_data = []
lat_data = []
lon_data = []
heading_data = []
cog_data = []
sog_data = []

# === Setup Plot ===
fig, axs = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
axs[0].set_ylabel('Latitude')
axs[1].set_ylabel('Longitude')
axs[2].set_ylabel('Heading (deg)')
axs[3].set_ylabel('COG (deg)')
axs[4].set_ylabel('SOG (km/h)')
axs[4].set_xlabel('Time')

lat_line, = axs[0].plot([], [], label="Latitude")
lon_line, = axs[1].plot([], [], label="Longitude")
heading_line, = axs[2].plot([], [], label="Heading")
cog_line, = axs[3].plot([], [], label="COG")
sog_line, = axs[4].plot([], [], label="SOG")

# Initialize std fill lists
std_fills = [None] * 5

for ax in axs:
    ax.grid(True)

# === Animation Update Function ===
def update(frame):
    pos = parser.get_position()
    heading = parser.get_heading()
    course = parser.get_course_speed()

    if pos is None or heading is None or course is None:
        return

    now = datetime.now().strftime('%H:%M:%S')
    time_data.append(now)
    lat_data.append(pos['latitude'])
    lon_data.append(pos['longitude'])
    heading_data.append(heading)
    cog_data.append(course['course'])
    sog_data.append(course['speed_kmh'])

    # Limit buffer size
    max_points = 100
    if len(time_data) > max_points:
        time_data.pop(0)
        lat_data.pop(0)
        lon_data.pop(0)
        heading_data.pop(0)
        cog_data.pop(0)
        sog_data.pop(0)

    # Update plots and standard deviation bands
    data_series = [lat_data, lon_data, heading_data, cog_data, sog_data]
    lines = [lat_line, lon_line, heading_line, cog_line, sog_line]

    for i, (ax, line, data) in enumerate(zip(axs, lines, data_series)):
        x_vals = range(len(time_data))
        line.set_data(x_vals, data)

        if len(data) > 1:
            mean = np.mean(data)
            std = np.std(data)
            upper = [mean + std] * len(data)
            lower = [mean - std] * len(data)
            if std_fills[i]:
                std_fills[i].remove()
            std_fills[i] = ax.fill_between(x_vals, lower, upper, color='gray', alpha=0.3)

        ax.relim()
        ax.autoscale_view()

    axs[4].set_xticks(range(len(time_data)))
    axs[4].set_xticklabels(time_data, rotation=45, ha='right')

# === Main Function ===
def main():
    try:
        parser.start()
        ani = animation.FuncAnimation(fig, update, interval=500)
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        parser.stop()
        print("GNSS parser stopped.")

if __name__ == '__main__':
    main()