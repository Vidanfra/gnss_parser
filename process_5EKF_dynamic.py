import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog
import matplotlib.dates as mdates
# Import libraries for mapping
import geopandas as gpd
import contextily as ctx
from shapely.geometry import Point, LineString
import utm
import fivestatesEKF as EKF
import numpy as np

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

# --- (A) Build the original GeoDataFrame (WGS84 → WebMercator) ---------------

# 1. Create a GeoDataFrame in lat/lon (EPSG:4326)
geometry = [Point(lon, lat) for lon, lat in zip(df["longitude"], df["latitude"])]
gdf = gpd.GeoDataFrame(df.copy(), geometry=geometry, crs="EPSG:4326")

# 2. Reproject to Web Mercator (EPSG:3857) for easy basemap plotting
gdf = gdf.to_crs(epsg=3857)


# --- (B) Run the EKF on every (lat, lon) pair to collect EKF outputs ----------

# Example EKF parameters (you already have these)
h_samp = 1 / 20 # Sample time (s) corresponding to 20 Hz
gnss_freq = 10 #  GNSS measurement 10 times slower (10 Hz)
Z = 2
Qd = np.diag([0.01, 0.01])
Rd = np.diag([5, 5])

ekf = EKF.EKF5States(Qd, Rd, Z, h_samp, 'LL')

# Step through all GNSS fixes in chronological order:
ekf_states = []
for lon, lat in zip(df["longitude"], df["latitude"]):
    x_hat = ekf.step(lat, lon)
    ekf_states.append(x_hat)

# Extract EKF‐latitude, EKF‐longitude (the first two elements of each state)
ekf_lats = np.array([x_hat[0] for x_hat in ekf_states])
ekf_lons = np.array([x_hat[1] for x_hat in ekf_states])
ekf_sog = np.array([x_hat[2] for x_hat in ekf_states])
ekf_cog = np.array([x_hat[3] for x_hat in ekf_states])

# Build EKF GeoDataFrame (in EPSG:4326), then reproject to 3857
ekf_geometry = [Point(lon, lat) for lon, lat in zip(ekf_lons, ekf_lats)]
gdfEKF = gpd.GeoDataFrame(df.copy(), geometry=ekf_geometry, crs="EPSG:4326")
gdfEKF = gdfEKF.to_crs(epsg=3857)

# Also build a LineString connecting all EKF points (in WebMercator)
line_ekf = LineString(gdfEKF.geometry.tolist())
line_ekf_gdf = gpd.GeoDataFrame(geometry=[line_ekf], crs=gdfEKF.crs)


# ─────────────────────────────────────────────────────────────────────────────
# (C) Plot #1: Map with original GNSS fixes (red stars) & EKF (blue triangles +
#              blue connecting line)
# ─────────────────────────────────────────────────────────────────────────────

# Compute bounding box + padding in WebMercator
xmin, ymin, xmax, ymax = gdf.total_bounds
padding = 20  # meters
xmin -= padding; xmax += padding
ymin -= padding; ymax += padding

fig, ax = plt.subplots(figsize=(20, 20))
fig.suptitle("GNSS Track with Original (red stars) & EKF (blue triangles + line)", fontsize=16)
fig.canvas.manager.set_window_title("Map - " + filename)

# (1) Plot original GNSS points as red stars (marker='*', no line)
gdf.plot(
    ax=ax,
    color='red',
    marker='*',
    markersize=30,
    linestyle='None',
    label='Original GNSS Fixes'
)

# (2) Plot EKF points as blue triangles (marker='^', no line)
gdfEKF.plot(
    ax=ax,
    color='blue',
    marker='^',
    markersize=20,
    linestyle='None',
    label='EKF Positions'
)

# (3) Plot EKF connecting line in solid blue
line_ekf_gdf.plot(
    ax=ax,
    color='blue',
    linewidth=2,
    label='EKF Path'
)

ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)

# Add a basemap underneath (Esri World Imagery)
ctx.add_basemap(ax, source=ctx.providers.Esri.WorldImagery)

ax.legend(fontsize=14)
ax.set_axis_off()


# ─────────────────────────────────────────────────────────────────────────────
# (D) Plot #2: Time series of Latitude, Longitude, Altitude
#             – overlay EKF latitude & longitude in dashed green
# ─────────────────────────────────────────────────────────────────────────────

columns = ["latitude", "longitude", "altitude"]
n_vars = len(columns)

fig1, axes1 = plt.subplots(n_vars, 1, figsize=(12, 3 * n_vars), sharex=True)
fig1.suptitle("Latitude, Longitude, and Altitude with EKF Overlay", fontsize=16)
fig1.canvas.manager.set_window_title("Position - " + filename)

if n_vars == 1:
    axes1 = [axes1]

for ax, col in zip(axes1, columns):
    signal = df[col]
    std = signal.rolling(rolling_window).std()

    # Plot original signal in solid blue
    ax.plot(signal.index, signal, label=f"{col} (orig)", color='blue')
    ax.fill_between(
        signal.index,
        signal - std,
        signal + std,
        color='red',
        alpha=0.3,
        label='±1 STD'
    )

    # Overlay EKF latitude/longitude as dashed green (altitude has no EKF counterpart)
    if col == "latitude":
        ax.plot(
            df.index,
            ekf_lats,
            label="latitude (EKF)",
            color='green',
            linestyle='--'
        )
    elif col == "longitude":
        ax.plot(
            df.index,
            ekf_lons,
            label="longitude (EKF)",
            color='green',
            linestyle='--'
        )

    ax.set_ylabel(col, fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True)

axes1[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time", fontsize=12)
plt.tight_layout(rect=[0, 0, 1, 0.97])  # leave room for the suptitle


# ─────────────────────────────────────────────────────────────────────────────
# (E) Plot #3: UTM Easting, Northing, Altitude with EKF overlay
# ─────────────────────────────────────────────────────────────────────────────

# (E.1) Rebuild a GeoDataFrame in lat/lon (EPSG:4326)
gdf_ll = gpd.GeoDataFrame(df.copy(),
                          geometry=[Point(lon, lat) for lon, lat in zip(df["longitude"], df["latitude"])],
                          crs="EPSG:4326")

# (E.2) Find median lat/lon to pick a UTM zone
median_lat = df["latitude"].median()
median_lon = df["longitude"].median()
_, _, utm_zone_number, _ = utm.from_latlon(median_lat, median_lon)
utm_epsg = int(32600 + utm_zone_number) if median_lat >= 0 else int(32700 + utm_zone_number)

# (E.3) Project both original and EKF to the same UTM CRS
utm_gdf   = gdf_ll.to_crs(epsg=utm_epsg)
utm_gdfEKF = gdfEKF.to_crs(epsg=utm_epsg)

# (E.4) Extract easting/northing from each
utm_gdf["easting"]  = utm_gdf.geometry.x
utm_gdf["northing"] = utm_gdf.geometry.y

utm_gdfEKF["easting"]  = utm_gdfEKF.geometry.x
utm_gdfEKF["northing"] = utm_gdfEKF.geometry.y

# (E.5) Plot Easting, Northing, Altitude time series with EKF overlays
columns_utm = ["easting", "northing", "altitude"]
n_vars_utm = len(columns_utm)

fig2, axes2 = plt.subplots(n_vars_utm, 1, figsize=(12, 3 * n_vars_utm), sharex=True)
fig2.suptitle("UTM Position (Easting, Northing, Altitude) with EKF Overlay", fontsize=16)
fig2.canvas.manager.set_window_title("UTM Position - " + filename)

if n_vars_utm == 1:
    axes2 = [axes2]

for ax, col in zip(axes2, columns_utm):
    signal = utm_gdf[col]
    std = signal.rolling(rolling_window).std()

    # Original in solid blue + ±1 STD in red shade
    ax.plot(signal.index, signal, label=f"{col} (orig)", color='blue')
    ax.fill_between(
        signal.index,
        signal - std,
        signal + std,
        color='red',
        alpha=0.3,
        label='±1 STD'
    )

    # Overlay EKF for easting/northing only
    if col in ["easting", "northing"]:
        ekf_signal = utm_gdfEKF[col]
        ax.plot(
            signal.index,
            ekf_signal,
            label=f"{col} (EKF)",
            color='green',
            linestyle='--'
        )
    # No EKF altitude, so skip

    ax.set_ylabel(col, fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True)

axes2[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time", fontsize=12)
plt.tight_layout(rect=[0, 0, 1, 0.95])


# ─────────────────────────────────────────────────────────────────────────────
# (F) Plot #4: Navigation data (gnss_quality, heading, cog, sog_kmh) with EKF overlays
# ─────────────────────────────────────────────────────────────────────────────

# Add to dataframe for plotting
df["sog_kmh_ekf"] = ekf_sog
df["cog_ekf"] = ekf_cog

print(df["sog_kmh_ekf"])
print(df["cog_ekf"])
# ----------------------------------------------------
# Navigation Plot (GNSS Quality, Heading, COG, SOG)
nav_columns = ['gnss_quality', 'heading', 'cog', 'sog_kmh']
ekf_overlays = {'cog': 'cog_ekf', 'sog_kmh': 'sog_kmh_ekf'}
n_vars_nav = len(nav_columns)

fig3, axes3 = plt.subplots(n_vars_nav, 1, figsize=(12, 3 * n_vars_nav), sharex=True)
fig3.suptitle("GNSS Quality, Heading, COG, and SOG (with EKF Overlays)", fontsize=16)
fig3.canvas.manager.set_window_title("Navigation - " + filename)

if n_vars_nav == 1:
    axes3 = [axes3]

for ax, col in zip(axes3, nav_columns):
    signal = df[col]
    
    # Plot ±1 STD fill only for continuous raw signals
    if col != 'gnss_quality':
        std = signal.rolling(rolling_window).std()
        ax.fill_between(
            signal.index,
            signal - std,
            signal + std,
            color='red',
            alpha=0.3,
            label='±1 STD'
        )

    # Plot raw signal
    ax.plot(signal.index, signal, label=f'{col} (Raw)', color='blue')

    # Plot EKF overlay if available
    if col in ekf_overlays:
        ekf_col = ekf_overlays[col]
        ax.plot(df.index, df[ekf_col], label=f'{col} (EKF)', color='green', linestyle='--')

    ax.set_ylabel(col, fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True)

axes3[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time", fontsize=12)
plt.tight_layout(rect=[0, 0, 1, 0.95])

# ─────────────────────────────────────────────────────────────────────────────
plt.show()