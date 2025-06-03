import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog
import matplotlib.dates as mdates
# Import libraries for mapping
import geopandas as gpd
import contextily as ctx
from shapely.geometry import Point, LineString
import utm

# Select CSV file using Windows file explorer
def select_file():
    root = Tk()
    root.withdraw()
    return filedialog.askopenfilename(filetypes=[("CSV Files", "*.csv")])

# Load the CSV file
file_path = select_file()
if not file_path:
    print("No file selected.")
    exit()
filename = file_path.split("/")[-1].split(".")[0]

# Read and parse the CSV
df = pd.read_csv(file_path, parse_dates=["timestamp"])
df.set_index("timestamp", inplace=True)

# --- Setup variables ---
# Choose the interval to plot
#df = df[df["gnss_quality"] == 4]  # Filter for RTK fixes
#df = df[df["gnss_quality"] == 5]  # Filter for RTK Float fixes
#df = df[df["gnss_quality"] == 2] # Filter for DGPS fixes 
rolling_window = 30  # Rolling window for standard deviation

# Create GeoDataFrame with WGS84 (lat/lon)
geometry = [Point(xy) for xy in zip(df["longitude"], df["latitude"])]
gdf = gpd.GeoDataFrame(df, geometry=geometry, crs="EPSG:4326")

# Convert to Web Mercator for tile compatibility
gdf = gdf.to_crs(epsg=3857)

# Create LineString from point sequence
line = LineString(gdf.geometry.tolist())
line_gdf = gpd.GeoDataFrame(geometry=[line], crs=gdf.crs)

# Get bounds and pad for visual clarity
xmin, ymin, xmax, ymax = gdf.total_bounds
padding = 20  # meters
xmin -= padding
xmax += padding
ymin -= padding
ymax += padding

# Plot
fig, ax = plt.subplots(figsize=(20, 20))
fig.suptitle("GNSS Track with Connected Path", fontsize=16)
fig.canvas.manager.set_window_title("Map - " + filename)

# Plot all points
gdf.plot(ax=ax, markersize=5, color='blue', label='GNSS Fixes')

# Plot the connecting line
line_gdf.plot(ax=ax, color='red', linewidth=2, label='Track Line')

# Set plot limits
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)

# Add basemap
ctx.add_basemap(ax, source=ctx.providers.Esri.WorldImagery)
plt.legend()


# Plot position and navigation data

# Plot position (latitude, longitude, altitude)
# --- Setup variables ---
columns = df.columns[:3]  # 'latitude', 'longitude', 'altitude'
n_vars = len(columns)

# --- Plot Latitude, Longitude, Altitude ---
fig1, axes1 = plt.subplots(n_vars, 1, figsize=(12, 3 * n_vars), sharex=True)
fig1.suptitle("Latitude, Longitude, and Altitude", fontsize=16)
fig1.canvas.manager.set_window_title("Position - " + filename)

if n_vars == 1:
    axes1 = [axes1]

for ax, col in zip(axes1, columns):
    signal = df[col]
    std = signal.rolling(rolling_window).std()
    
    ax.plot(signal.index, signal, label=col, color='blue')
    ax.fill_between(signal.index, signal - std, signal + std, color='red', alpha=0.3, label='±1 STD')
    
    ax.set_ylabel(col)
    ax.legend()
    ax.grid(True)

axes1[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time")
plt.tight_layout()

# Plot Easting, Northing, Altitude in UTM coordinates

# Step 1: Create geometry from lat/lon
geometry = [Point(xy) for xy in zip(df["longitude"], df["latitude"])]
gdf = gpd.GeoDataFrame(df, geometry=geometry, crs="EPSG:4326")  # WGS84

# Step 2: Get median lat/lon
median_lat = df["latitude"].median()
median_lon = df["longitude"].median()

# Step 3: Automatically detect UTM zone from median lat/lon
easting, northing, utm_zone_number, utm_zone_letter = utm.from_latlon(median_lat, median_lon)
utm_epsg = int(32600 + utm_zone_number) if median_lat >= 0 else int(32700 + utm_zone_number)

# Step 4: Project to UTM
utm_df = gdf.to_crs(epsg=utm_epsg)

# Step 5: Extract UTM coordinates
utm_df["easting"] = utm_df.geometry.x
utm_df["northing"] = utm_df.geometry.y


# --- Plot Easting, Northing, Altitude ---
fig2, axes2 = plt.subplots(n_vars, 1, figsize=(12, 3 * n_vars), sharex=True)
fig2.suptitle("UTM Position (Easting, Northing, Altitude)", fontsize=16)
fig2.canvas.manager.set_window_title("UTM Position - " + filename)

if n_vars == 1:
    axes2 = [axes2]

for ax, col in zip(axes2, ["easting", "northing", "altitude"]):
    signal = utm_df[col]
    std = signal.rolling(rolling_window).std()
    
    ax.plot(signal.index, signal, label=col, color='blue')
    ax.fill_between(signal.index, signal - std, signal + std, color='red', alpha=0.3, label='±1 STD')
    
    ax.set_ylabel(col)
    ax.legend()
    ax.grid(True)

axes2[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time")
plt.tight_layout()

# Plot navigation data: GNSS Quality, Heading, COG, SOG
columns = ['gnss_quality', 'heading', 'cog', 'sog_kmh']
n_vars = len(columns)

# Create subplots
fig3, axes3 = plt.subplots(n_vars, 1, figsize=(12, 3 * n_vars), sharex=True)
fig3.suptitle("GNSS Quality, Heading, COG, and SOG", fontsize=16)
fig3.canvas.manager.set_window_title("Navigation - " + filename)

# Ensure axes3 is iterable
if n_vars == 1:
    axes3 = [axes3]

for ax, col in zip(axes3, columns):
    signal = df[col]
    
    # Only compute STD if signal is continuous
    if col != 'gnss_quality':
        std = signal.rolling(rolling_window).std()
        ax.fill_between(signal.index, signal - std, signal + std, color='red', alpha=0.3, label='±1 STD')
    
    ax.plot(signal.index, signal, label=col, color='blue')
    ax.set_ylabel(col)
    ax.legend()
    ax.grid(True)

# Format x-axis as time
axes3[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time")
plt.tight_layout()

plt.show()

