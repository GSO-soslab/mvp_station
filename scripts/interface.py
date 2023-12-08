from PyQt5.QtWidgets import QAction, QInputDialog
from qgis.core import QgsProject, Qgis, QgsLayerTreeGroup, QgsFields
import qgis.core as core
from qgis.utils import iface
from qgis.gui import QgsMapToolEmitPoint, QgsMapToolPan
from pyproj import Proj, transform
from datetime import datetime
import csv
from pathlib import Path
import os
import sys
import random

class Interface():
    def __init__(self):
        
        self.iface = iface
        self.canvas = iface.mapCanvas()
        self.iface.messageBar().pushMessage(f"[INFO]", "Click Add Point to select Waypoints", level=Qgis.Info)
        self.toolbar = self.iface.addToolBar("MVP Station")

        #Add Waypoints
        self.undo = QAction("Add Point", iface.mainWindow())
        self.toolbar.addAction(self.undo)
        self.undo.triggered.connect(self.add_waypoint)

        #Undo button
        self.undo = QAction("Undo Point", iface.mainWindow())
        self.toolbar.addAction(self.undo)
        self.undo.triggered.connect(self.undo_waypoint)

        #creates the button in the panel window
        self.action = QAction("Send Waypoints / Exit", iface.mainWindow())
        self.toolbar.addAction(self.action)
        #on click, runs this function
        self.action.triggered.connect(self.run_function)

        #Terminal Button
        self.service = QAction("Terminal", iface.mainWindow())
        self.toolbar.addAction(self.service)
        self.service.triggered.connect(self.service_callBack)

        #list of waypoints.
        self.waypoints = []
        self.flag = 0
        self.alt = 0

    def service_callBack(self):
        self.iface.messageBar().pushMessage("[INFO]", "Opened Terminal", level=Qgis.Info)
        os.system("gnome-terminal -e 'bash -c \"exec bash\"'")

    def undo_waypoint(self):
        if self.waypoints == []:
            self.iface.messageBar().pushMessage(f"[WARN]", f"No more points to remove", level=Qgis.Warning)
        else:
            self.waypoints.pop()
            print(f"[INFO]:Points Selected: {len(self.waypoints)}")
            self.point_name = f"point{len(self.waypoints)}"
            layer = QgsProject.instance().mapLayersByName(self.point_name)[0]
            QgsProject.instance().removeMapLayer(layer)
        
    def run_function(self):
        # This function could trigger the backend function directly
        # or could emit a signal connected to the backend function
        if self.waypoints == []:
            self.iface.messageBar().pushMessage(f"[INFO]", "Safely Exited", level=Qgis.Info)
        else:
            self.iface.messageBar().pushMessage(f"[INFO]", "Waypoints Sent", level=Qgis.Info)
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
        # self.iface.removeToolBarIcon(self.action)
        # self.iface.removeToolBarIcon(self.undo)
        # self.iface.removeToolBarIcon(self.service)

        # self.iface.mainWindow().removeToolBar(self.toolbar)

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
        #Altitude Text Box
        tempTuple = QInputDialog.getText(None, "Enter Altitude",f"Negative value is sub surface. If {int(self.alt)} meters, leave blank: ")
        if tempTuple[1]:
            alt = tempTuple[0]
            if alt != '':
                self.alt = float(alt)

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
            writer = core.QgsVectorFileWriter(fn, 'UTF-8', layerFields, core.QgsWkbTypes.Point, core.QgsCoordinateReferenceSystem("EPSG:3857"), "ESRI Shapefile")
            feat = core.QgsFeature()
            feat.setGeometry(core.QgsGeometry.fromPointXY(core.QgsPointXY(cood[0], cood[1])))
            writer.addFeature(feat)
            del(writer)

            #Places in Group
            layer = core.QgsVectorLayer(fn, self.point_name, "ogr")
            self.myGroup.insertChildNode(-1, core.QgsLayerTreeLayer(layer))
            QgsProject.instance().addMapLayer(layer, False)

            self.waypoints.append([self.lat, self.long, self.alt])
            print(f"[INFO]:Points Selected: {len(self.waypoints)}")

    def add_waypoint(self):
        # this QGIS tool emits as QgsPoint after each click on the map canvas
        self.pointTool = QgsMapToolEmitPoint(self.canvas)
        self.pointTool.canvasClicked.connect(self.display_point)

        #Textbox to name the group
        tempTuple = QInputDialog.getText(None, "Mission", "Name your Mission")
        if tempTuple[1]:
            group_name = tempTuple[0]
            if group_name != '':
                self.group_name = group_name
            else:
                self.group_name = "unnamed_mission" + str(random.randint(0,9))
        else:
            self.group_name = "unnamed_mission" + str(random.randint(0,9))
        self.root = QgsProject.instance().layerTreeRoot()
        self.myGroup = QgsLayerTreeGroup(self.group_name)
        self.root.insertChildNode(0,self.myGroup)
        # self.myGroup = self.root.addGroup(self.group_name)
        
        #Keeps the cursor on the map
        self.canvas.setMapTool(self.pointTool)

            
l = Interface()