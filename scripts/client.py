#!/usr/bin/env python3
import socket
import csv
import pyproj
import os


class Visualise:
    def __init__(self, HOST, PORT):
        self.HOST = HOST
        self.PORT = PORT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        
        self.csv_data='/home/soslab/PyGQIS/data.csv'
        if os.path.exists(self.csv_data):
            os.remove(self.csv_data)
        self.csv_file = open(self.csv_data, 'a')
        header = ['X', 'Y','ROLL','PITCH','YAW']
        dw = csv.DictWriter(self.csv_file, delimiter=',', fieldnames=header)
        dw.writeheader()
        self.csv_file.close()
        self.x,self.y,self.roll, self.pitch, self.yaw = 0,0,0,0,0

    def connect_to_gcs(self):
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
            self.write_to_csv()
                    # res = [float(i) for i in res]

    def parse_gps(self):
        #['GPS', ' 40999994.0', '71000000.0', '2140.774169921875']
        lat = float(self.res[1]) /10e5
        long = float(self.res[2]) /10e5
        self.alt = float(self.res[3])
        transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
        self.x, self.y = transformer.transform(long, lat)

    def parse_yaw(self):
        #[ATTITUDE, -0.01279214583337307,-0.03361418843269348,-0.01279214583337307]'
        self.roll = float(self.res[1]) 
        self.pitch = float(self.res[2])
        self.yaw = float(self.res[3])

    def write_to_csv(self):
        self.csv_file = open(self.csv_data, 'a')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',',quotechar='"',quoting=csv.QUOTE_ALL)
        self.csv_writer.writerow([self.x,self.y,self.roll, self.pitch, self.yaw])
        self.csv_file.close()
        


if __name__ == "__main__":
    vis = Visualise(HOST="192.168.1.186", PORT=8080)
    while 1:
        vis.connect_to_gcs()