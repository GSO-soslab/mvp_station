import PyQt5.QtWidgets as widgets
import qgis.core as core
from qgis.utils import iface
import qgis.gui as gui
from pyproj import Proj, transform
from datetime import datetime
from pathlib import Path
import os
import sys
import csv
import random

class Interface():
    def __init__(self):
        
        self.iface = iface
        self.canvas = iface.mapCanvas()
        self.root = core.QgsProject.instance().layerTreeRoot()
        self.iface.messageBar().pushMessage(f"[INFO]", "Click Add Point to select Waypoints", level=core.Qgis.Info)
        self.toolbar = self.iface.addToolBar("MVP Station")

        #Add Waypoints
        self.undo = widgets.QAction("Add Point", iface.mainWindow())
        self.toolbar.addAction(self.undo)
        self.undo.triggered.connect(self.add_waypoint)

        #Undo button
        self.undo = widgets.QAction("Undo Point", iface.mainWindow())
        self.toolbar.addAction(self.undo)
        self.undo.triggered.connect(self.undo_waypoint)

        #creates the button in the panel window
        self.action = widgets.QAction("Send Waypoints / Exit", iface.mainWindow())
        self.toolbar.addAction(self.action)
        #on click, runs this function
        self.action.triggered.connect(self.run_function)

        #Terminal Button
        self.service = widgets.QAction("Terminal", iface.mainWindow())
        self.toolbar.addAction(self.service)
        self.service.triggered.connect(self.service_callBack)

        #list of waypoints.
        self.waypoints = []
        self.flag = 0
        self.alt = 0
        self.lat_list = []
        self.long_list = []
        self.alt_list = []

    def add_waypoint(self):
        # this QGIS tool emits as QgsPoint after each click on the map canvas
        self.pointTool = gui.QgsMapToolEmitPoint(self.canvas)
        self.pointTool.canvasClicked.connect(self.display_point)

        #Textbox to name the group
        tempTuple = widgets.QInputDialog.getText(None, "Mission", "Name your Mission")
        if tempTuple[1]:
            group_name = tempTuple[0]
            if group_name != '':
                self.group_name = group_name
            else:
                self.group_name = "unnamed_mission" + str(random.randint(0,9))
        else:
            self.group_name = "unnamed_mission" + str(random.randint(0,9))
        self.myGroup = core.QgsLayerTreeGroup(self.group_name)
        self.root.insertChildNode(0,self.myGroup)
        # self.myGroup = self.root.addGroup(self.group_name)
        
        #Keeps the cursor on the map
        self.canvas.setMapTool(self.pointTool)
    
    def display_point(self, pointTool):
        P3857 = Proj(init='epsg:3857')
        P4326 = Proj(init='epsg:4326')
        cood = []
        for point in pointTool:
            cood.append(point)        
        #Transfrom from canvas frame to map frame
        self.long,self.lat = transform(P3857,P4326, cood[0], cood[1])
        #Altitude Text Box
        tempTuple = widgets.QInputDialog.getText(None, "Enter Altitude",f"Negative value is sub surface. If {int(self.alt)} meters, leave blank: ")
        if tempTuple[1]:
            alt = tempTuple[0]
            if alt != '':
                self.alt = float(alt)

            self.lat_list.append(self.lat)
            self.long_list.append(self.long)
            self.alt_list.append(self.alt)
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
            self.point_name = f"wpt_{len(self.waypoints)}_{self.lat_list[-1]}_{self.long_list[-1]}_{self.alt_list[-1]}_{self.group_name}"
            fn = path+ self.point_name +".shp"

            #displays the point on the map
            layerFields = core.QgsFields()
            writer = core.QgsVectorFileWriter(fn, 'UTF-8', layerFields, core.QgsWkbTypes.Point, core.QgsCoordinateReferenceSystem("EPSG:3857"), "ESRI Shapefile")
            feat = core.QgsFeature()
            feat.setGeometry(core.QgsGeometry.fromPointXY(core.QgsPointXY(cood[0], cood[1])))
            writer.addFeature(feat)
            del(writer)

            #Places in Group
            layer = core.QgsVectorLayer(fn, self.point_name, "ogr")
            self.myGroup.insertChildNode(-1, core.QgsLayerTreeLayer(layer))
            core.QgsProject.instance().addMapLayer(layer, False)

            self.waypoints.append([self.lat, self.long, self.alt])
            print(f"[INFO]:Points Selected: {len(self.waypoints)}")

    def undo_waypoint(self):
        if self.waypoints == []:
            self.iface.messageBar().pushMessage(f"[WARN]", f"No more points to remove", level=core.Qgis.Warning)
        else:
            self.waypoints.pop()
            print(f"[INFO]:Points Selected: {len(self.waypoints)}")
            self.point_name = f"wpt_{len(self.waypoints)}_{self.lat_list[-1]}_{self.long_list[-1]}_{self.alt_list[-1]}_{self.group_name}"
            self.lat_list.pop()
            self.long_list.pop()
            self.alt_list.pop()
            layer = core.QgsProject.instance().mapLayersByName(self.point_name)[0]
            core.QgsProject.instance().removeMapLayer(layer)
            self.canvas.refresh()
        
    def service_callBack(self):
        self.iface.messageBar().pushMessage("[INFO]", "Opened Terminal", level=core.Qgis.Info)
        os.system("gnome-terminal -e 'bash -c \"exec bash\"'")


    def run_function(self):
        # This function could trigger the backend function directly
        # or could emit a signal connected to the backend function
        l = self.printLayerTreeHierarchy(self.root)
        if l != []:
            self.items = gui.QgsCheckableComboBox()
            self.items.addItems(l)
            dlg = widgets.QDialog()
            dlg.setWindowTitle("Select Mission from dropdown")
            dlg.setFixedWidth(300)
            layout = widgets.QVBoxLayout()
            layout.addWidget(self.items)
            button = widgets.QPushButton("OK")
            button.clicked.connect(self.send_waypoints)
            button.clicked.connect(dlg.accept)
            layout.addWidget(button)
            dlg.setLayout(layout)
            dlg.exec_()
        else:
            self.iface.messageBar().pushMessage("[WARN]", "There are no missions to upload", level=core.Qgis.Warning)

    def send_waypoints(self):
        waypoints = []
        for layers in core.QgsProject.instance().mapLayers().values():
            if self.items.checkedItems()[0] in layers.name():
                points = layers.name().split('_')
                waypoints.append([float(points[2]), float(points[3]), float(points[4])])
        self.iface.messageBar().pushMessage(f"[INFO]", "Waypoints Sent", level=core.Qgis.Info)
        home = str(Path.home())
        if (sys.platform == "win32"):
            path = home + '\\PyQGIS\\waypoints.csv'
        elif(sys.platform == "linux"):
            path = home + '/PyQGIS/waypoints.csv'
        csv_file = open(path, 'w')
        csv_writer = csv.writer(csv_file, delimiter=',',quotechar='"',quoting=csv.QUOTE_ALL)
        # a reference to our map canvas
        csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                            waypoints,
                            self.flag])
        ##HERE

        #[TEST]:Once sent, the script stops running and the cursor is set to the pan tool.
        # self.iface.removeToolBarIcon(self.action)
        # self.iface.removeToolBarIcon(self.undo)
        # self.iface.removeToolBarIcon(self.service)

        # self.iface.mainWindow().removeToolBar(self.toolbar)

        self.toolPan = gui.QgsMapToolPan(self.canvas)
        self.canvas.setMapTool(self.toolPan)
        #Clear the queue.
        self.waypoints = []

    def printLayerTreeHierarchy(self,node):
        group_list = []
        for child in node.children():
            if isinstance(child, core.QgsLayerTreeGroup):
                # print(f"{indent}Group: {child.name()}")
                group_list.append(child.name())
        return group_list
        
l = Interface()