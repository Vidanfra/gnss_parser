import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import numpy as np
import time
import contextily as ctx
import geopandas as gpd
from shapely.geometry import Point
from gnss_parser import GNSSParser
import pandas as pd
import os

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

# === Constants ===
MIN_PAD = 40  # Minimum padding in meters

# === Data Buffers ===
time_data = []
lat_data = []
lon_data = []
heading_data = []
cog_data = []
sog_data = []

# === Setup Plot ===
fig_ts, axs = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
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

# === Setup Satellite Map Figure ===
fig_map, ax_map = plt.subplots(figsize=(8, 8))
ax_map.set_title('Real-Time Position on Satellite Map')
# Initialize empty GeoDataFrame for points
gdf = gpd.GeoDataFrame(geometry=[], crs='EPSG:4326')
# Plot initial empty layer
scatter = None

# === File for Logging Data ===
init_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"gnss_data_{init_time}.csv"

# === Function to Log Data with Header ===
def log_data(filename, pos, heading, course):
    # Prepare header and data line
    header = "timestamp,latitude,longitude,altitude,gnss_quality,heading,cog,sog_kmh"
    data_line = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')},{pos['latitude']},{pos['longitude']},{pos['altitude']},{pos['gnss_quality']},{heading},{course['course']},{course['speed_kmh']}"

    # Write header if file is new or empty
    write_header = not os.path.exists(filename) or os.path.getsize(filename) == 0
    with open(filename, 'a') as file:
        if write_header:
            file.write(header + '\n')
        file.write(data_line + '\n')

    # Optional: print to console
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}]  Latitude={pos['latitude']}, Longitude={pos['longitude']}, Altitude={pos['altitude']}, Fix={pos['gnss_quality']} Heading={heading}, COG={course['course']}, SOG={course['speed_kmh']}")


# === Animation Update Function ===
def update(frame):
    global scatter, gdf
    pos = parser.get_position()
    heading = parser.get_heading()
    course = parser.get_course_speed()

    if pos is None or heading is None or course is None:
        return
    # Log data to file
    log_data(filename, pos, heading, course)

    # Timestamped data
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
            std_fills[i] = ax.fill_between(x_vals, lower, upper, color='blue', alpha=0.3)

        ax.relim()
        ax.autoscale_view()

    axs[4].set_xticks(range(len(time_data)))
    axs[4].set_xticklabels(time_data, rotation=45, ha='right')

    # Update satellite map
    # Add new point to the GeoDataFrame
    point = Point(pos['longitude'], pos['latitude'])
    new_point = gpd.GeoDataFrame([{'geometry': point}], crs="EPSG:4326")
    gdf = pd.concat([gdf, new_point], ignore_index=True)
    # Reproject to Web Mercator
    gdf_web = gdf.to_crs(epsg=3857)
    ax_map.clear()
    gdf_web.plot(ax=ax_map, marker='o', color='red', markersize=5)

    # Get bounds and apply padding
    xmin, ymin, xmax, ymax = gdf_web.total_bounds
    x_pad = (xmax - xmin) * 0.1  # fallback pad in meters
    y_pad = (ymax - ymin) * 0.1 
    # Ensure minimum padding
    if x_pad < MIN_PAD: 
        x_pad = MIN_PAD
    if y_pad < MIN_PAD:
        y_pad = MIN_PAD

    ax_map.set_xlim(xmin - x_pad, xmax + x_pad)
    ax_map.set_ylim(ymin - y_pad, ymax + y_pad)

    ctx.add_basemap(ax_map, source=ctx.providers.Esri.WorldImagery)
    ax_map.set_title("Real-time GNSS Position on Satellite Map")
    ax_map.set_axis_off()


# === Main Function ===
def main():
    try:
        parser.start()
        print("GNSS parser launched successfully.")
        # Start animations
        ani_ts = animation.FuncAnimation(fig_ts, update, interval=500)
        ani_map = animation.FuncAnimation(fig_map, update, interval=500)
        plt.show()
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        parser.stop()
        print("GNSS parser stopped.")

if __name__ == '__main__':
    main()