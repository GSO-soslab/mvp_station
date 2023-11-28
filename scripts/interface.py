from PyQt5.QtWidgets import QAction
from qgis.core import QgsProject
from qgis.utils import iface
from qgis.gui import QgsMapToolEmitPoint
from pyproj import Proj, transform
from datetime import datetime
import csv
from pathlib import Path
import os
import sys

class Interface():
    def __init__(self):
        self.iface = iface
        self.toolbar = self.iface.addToolBar("MVP Station")
        
        #creates the button in the panel window
        self.action = QAction("Send Waypoints / Exit", iface.mainWindow())
        self.toolbar.addAction(self.action)
        #on click, runs this function
        self.action.triggered.connect(self.run_function)

        #Undo button
        self.undo = QAction("Undo Point", iface.mainWindow())
        self.toolbar.addAction(self.undo)
        self.undo.triggered.connect(self.undo_waypoint)

        #Service Button
        self.service = QAction("Terminal", iface.mainWindow())
        self.toolbar.addAction(self.service)
        self.service.triggered.connect(self.service_callBack)

        self.canvas = iface.mapCanvas()
        # this QGIS tool emits as QgsPoint after each click on the map canvas
        self.pointTool = QgsMapToolEmitPoint(self.canvas)
        self.pointTool.canvasClicked.connect(self.display_point)

        #Keeps the cursor on the map
        self.canvas.setMapTool(self.pointTool)

        #list of waypoints.
        self.waypoints = []
        self.flag = 0

    def service_callBack(self):
        print("Opened Terminal")
        os.system("gnome-terminal -e 'bash -c \"exec bash\"'")

    def undo_waypoint(self):
        if self.waypoints == []:
            print("There's no point to remove")
        else:
            self.waypoints.pop()
            print(f"Points Selected: {len(self.waypoints)}")
            self.point_name = f"point{len(self.waypoints)}"
            layer = QgsProject.instance().mapLayersByName(self.point_name)[0]
            QgsProject.instance().removeMapLayer(layer)
        
    def run_function(self):
        # This function could trigger the backend function directly
        # or could emit a signal connected to the backend function
        if self.waypoints == []:
            print("No points were selected")
        else:
            print("Waypoints sent")
            home = str(Path.home())
            if (sys.platform == "win32"):
                path = home + '\\PyQGIS\\waypoints.csv'
            elif(sys.platform == "linux"):
                path = home + '/PyQGIS/waypoints.csv'
            csv_file = open(path, 'w')
            csv_writer = csv.writer(csv_file, delimiter=',',quotechar='"',quoting=csv.QUOTE_ALL)
            # a reference to our map canvas
            csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                self.waypoints,
                                self.flag])

        #Once sent, the script stops running and the cursor is set to the pan tool.
        self.iface.removeToolBarIcon(self.action)
        self.iface.removeToolBarIcon(self.undo)
        self.iface.removeToolBarIcon(self.service)

        self.iface.mainWindow().removeToolBar(self.toolbar)

        self.toolPan = QgsMapToolPan(self.canvas)
        self.canvas.setMapTool(self.toolPan)

    def display_point(self, pointTool):
        P3857 = Proj(init='epsg:3857')
        P4326 = Proj(init='epsg:4326')
        cood = []
        for point in pointTool:
            cood.append(point)
        #Transfrom from canvas frame to map frame
        self.long,self.lat = transform(P3857,P4326, cood[0], cood[1])

        #creates the folder to store individual point.
        home = str(Path.home())

        if (sys.platform == "linux"):
            path = home + "/PyQGIS/Points/"
            if not os.path.exists(path):
                os.makedirs(path)
        elif (sys.platform == "win32"):
            path = home + "\\PyQGIS\\Points\\"
            if not os.path.exists(path):
                os.makedirs(path)
        self.point_name = f"point{len(self.waypoints)}"
        fn = path+ self.point_name +".shp"

        #displays the point on the map
        layerFields = QgsFields()
        writer = QgsVectorFileWriter(fn, 'UTF-8', layerFields, QgsWkbTypes.Point, QgsCoordinateReferenceSystem("EPSG:3857"), "ESRI Shapefile")
        feat = QgsFeature()
        feat.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(cood[0], cood[1])))
        writer.addFeature(feat)
        iface.addVectorLayer(fn, '', 'ogr')
        del(writer)

        self.waypoints.append([self.lat, self.long])
        print(f"Points Selected: {len(self.waypoints)}")
        
l = Interface()