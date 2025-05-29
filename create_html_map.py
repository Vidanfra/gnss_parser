import pandas as pd
from tkinter import Tk, filedialog
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

# Plot position on a satellite map with higher zoom and concentric rings
lat_lon = df[['latitude', 'longitude']].dropna()
midpoint = [lat_lon.latitude.mean(), lat_lon.longitude.mean()]

# Create a map with a higher zoom level
m = folium.Map(location=midpoint, zoom_start=23, tiles="Esri.WorldImagery")

# Draw the GNSS track
coordinates = lat_lon.values.tolist()
folium.PolyLine(locations=coordinates, color='blue', weight=3).add_to(m)

# Add markers for start and end
folium.Marker(coordinates[0], popup="Start", icon=folium.Icon(color='green')).add_to(m)
folium.Marker(coordinates[-1], popup="End", icon=folium.Icon(color='red')).add_to(m)

'''
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
'''

# Save and open the map
map_filename = "Plots/" + filename + "_map.html"
m.save(map_filename)
webbrowser.open('file://' + os.path.realpath(map_filename))