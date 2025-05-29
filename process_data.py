import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog
import matplotlib.dates as mdates
# Import folium for mapping
import folium
from folium.plugins import PolyLineTextPath
import webbrowser
import os

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

# Drop gnss_quality column
df = df.drop(columns=["gnss_quality"])


# Plot position on a satellite map with higher zoom and concentric rings
lat_lon = df[['latitude', 'longitude']].dropna()
midpoint = [lat_lon.latitude.mean(), lat_lon.longitude.mean()]

# Create a map with a higher zoom level
m = folium.Map(location=midpoint, zoom_start=21, tiles="Esri.WorldImagery")

# Draw the GNSS track
coordinates = lat_lon.values.tolist()
folium.PolyLine(locations=coordinates, color='blue', weight=3).add_to(m)

# Add markers for start and end
folium.Marker(coordinates[0], popup="Start", icon=folium.Icon(color='green')).add_to(m)
folium.Marker(coordinates[-1], popup="End", icon=folium.Icon(color='red')).add_to(m)

# Add concentric rings every 10 cm up to 1 meter
for i in range(1, 11):
    folium.Circle(
        location=midpoint,
        radius=i * 0.1,  # i * 0.1 meter = 10 cm increments
        color="orange",
        fill=False,
        weight=1,
        opacity=0.5
    ).add_to(m)

# Save and open the map
m.save(filename)
webbrowser.open('file://' + os.path.realpath(filename))

# Parameters
rolling_window = 10
columns = df.columns[:3]  # Select only the first three columns for plotting
n_vars = len(columns)

# Create subplots
fig, axes = plt.subplots(n_vars, 1, figsize=(12, 3 * n_vars), sharex=True)
fig.suptitle("Latitude, Longitude, and Altitude", fontsize=16) 
fig.canvas.manager.set_window_title("Position - " + filename)  # Set window title
if n_vars == 1:
    axes = [axes]

for ax, col in zip(axes, columns):
    signal = df[col]
    std = signal.rolling(rolling_window).std()
    
    ax.plot(signal.index, signal, label=col, color='blue')
    ax.fill_between(signal.index, signal - std, signal + std, color='red', alpha=0.3, label='±1 STD')
    
    ax.set_ylabel(col)
    ax.legend()
    ax.grid(True)

# Format x-axis as time
axes[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time")
plt.tight_layout()

# Parameters
rolling_window = 10
columns = df.columns[3:]  # Select only the last three columns for plotting
n_vars = len(columns)

# Create subplots
fig, axes = plt.subplots(n_vars, 1, figsize=(12, 3 * n_vars), sharex=True)
fig.suptitle("Heading, COG, and SOG", fontsize=16)
fig.canvas.manager.set_window_title("Navigation - " + filename)  # Set window title
if n_vars == 1:
    axes = [axes]

for ax, col in zip(axes, columns):
    signal = df[col]
    std = signal.rolling(rolling_window).std()
    
    ax.plot(signal.index, signal, label=col, color='blue')
    ax.fill_between(signal.index, signal - std, signal + std, color='red', alpha=0.3, label='±1 STD')
    
    ax.set_ylabel(col)
    ax.legend()
    ax.grid(True)

# Format x-axis as time
axes[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
plt.xticks(rotation=45)
plt.xlabel("Time")
plt.tight_layout()


# Show the plots
plt.show()

