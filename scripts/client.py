#!/usr/bin/env python3
import socket
# import qgis
# from pyproj import Proj, transform

class Visualise:
    def __init__(self, HOST, PORT):
        self.HOST = HOST
        self.PORT = PORT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

    def connect(self):
        data = self.s.recv(1024)
        if data != b"":
            data = data.decode()
            self.res = data.strip('][').split(',')
            self.res = list(self.res)
            if self.res[0] == 'GPS':
                self.parse_gps()
            elif self.res[0] == 'ATTITUDE':
                self.parse_yaw()
            elif self.res[0] == 'BAD_DATA':
                print("Bad data")
                    # res = [float(i) for i in res]

    def parse_gps(self):
        #['GPS', ' 40999994.0', '71000000.0', '2140.774169921875']
        # P3857 = Proj(init='epsg:3857')
        # P4326 = Proj(init='epsg:4326')
        lat = float(self.res[1]) /10e5
        long = float(self.res[2]) /10e5
        alt = float(self.res[3])
        print(lat, long)

        # x,y = transform(P4326, P3857, long , lat)
        # fn = "/home/soslab/PyGQIS/point.shp"
        # writer = qgis.QgsVectorFileWriter(fn, 'UTF-8', layerFields, QgsWkbTypes.Point, QgsCoordinateReferenceSystem("EPSG:3857"), "ESRI Shapefile")

        # feat = QgsFeature()
        # feat.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(x,y)))
        # writer.addFeature(feat)

        # iface.addVectorLayer(fn, '', 'ogr')
        # del(writer)

    def parse_yaw(self):
        #[ATTITUDE, -0.01279214583337307,-0.03361418843269348,-0.01279214583337307]'
        roll = float(self.res[1]) 
        pitch = float(self.res[2])
        yaw = float(self.res[3])
        print(yaw)

if __name__ == "__main__":
    vis = Visualise(HOST="192.168.1.186", PORT=8080)
    while 1:
        vis.connect()