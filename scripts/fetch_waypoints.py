from qgis.gui import QgsMapToolEmitPoint
from pyproj import Proj, transform
from pathlib import Path
from datetime import datetime
import csv

def display_point(pointTool):
    #print(pointTool)
    P3857 = Proj(init='epsg:3857')
    P4326 = Proj(init='epsg:4326')
    home = str(Path.home())
    csvPath = home + '/PyQGIS/waypoints.csv'
    csv_file = open(csvPath, 'w')
    csv_writer = csv.writer(csv_file, delimiter=',',quotechar='"',quoting=csv.QUOTE_ALL)
    cood = []
    for point in pointTool:
        cood.append(point)
    long,lat = transform(P3857,P4326, cood[0], cood[1])
    print(lat, long)
    csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        [lat, long]])

# a reference to our map canvas
canvas = iface.mapCanvas()

# this QGIS tool emits as QgsPoint after each click on the map canvas
pointTool = QgsMapToolEmitPoint(canvas)

pointTool.canvasClicked.connect(display_point)

canvas.setMapTool(pointTool)