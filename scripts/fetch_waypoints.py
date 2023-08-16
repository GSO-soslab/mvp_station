from PyQt5.QtWidgets import QAction
from qgis.utils import iface
from qgis.gui import QgsMapToolEmitPoint
from pyproj import Proj, transform
from datetime import datetime
import csv
from pathlib import Path

class Button():
    def __init__(self):
        self.iface = iface

        #creates a button in the panel window
        self.action = QAction("Send Waypoints", iface.mainWindow())
        self.iface.addToolBarIcon(self.action)
        
        #on click, runs this function
        self.action.triggered.connect(self.run_function)
        self.canvas = iface.mapCanvas()
        # this QGIS tool emits as QgsPoint after each click on the map canvas
        self.pointTool = QgsMapToolEmitPoint(self.canvas)
        self.pointTool.canvasClicked.connect(self.display_point)

        #Keeps the cursor on the map
        self.canvas.setMapTool(self.pointTool)

        #list of waypoints.
        self.waypoints = []


    def run_function(self):
        # This function could trigger the backend function directly
        # or could emit a signal connected to the backend function
        print("Waypoints sent")
        self.iface.removeToolBarIcon(self.action)
        home = str(Path.home())
        csvPath = home + '/PyQGIS/waypoints.csv'
        csv_file = open(csvPath, 'w')
        csv_writer = csv.writer(csv_file, delimiter=',',quotechar='"',quoting=csv.QUOTE_ALL)
        # a reference to our map canvas
        csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                            self.waypoints])

        #Once sent, the script stops running and the cursor is set to the pan tool.
        self.toolPan = QgsMapToolPan(self.canvas)
        self.canvas.setMapTool(self.toolPan)

    def display_point(self, pointTool):
        #print(pointTool)
        P3857 = Proj(init='epsg:3857')
        P4326 = Proj(init='epsg:4326')
        cood = []
        for point in pointTool:
            cood.append(point)
        self.long,self.lat = transform(P3857,P4326, cood[0], cood[1])
        print(self.lat, self.long)
        fn = "/home/soslab/PyQGIS/point.shp"

        #displays the point on the map
        layerFields = QgsFields()
        writer = QgsVectorFileWriter(fn, 'UTF-8', layerFields, QgsWkbTypes.Point, QgsCoordinateReferenceSystem("EPSG:3857"), "ESRI Shapefile")
        feat = QgsFeature()
        feat.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(cood[0], cood[1])))
        writer.addFeature(feat)
        iface.addVectorLayer(fn, '', 'ogr')
        del(writer)

        self.waypoints.append([self.lat, self.long])
        
Button()