#!/usr/bin/env python3
import socket
import csv
import pyproj
import os
from datetime import datetime
from pathlib import Path
import sys
import yaml
import os

class Visualise:
    def __init__(self, HOST, PORT):
        self.HOST = HOST
        self.PORT = PORT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        #Connect to GCS
        self.s.connect((self.HOST, self.PORT))
        
        home = str(Path.home())

        #Logs data
        if (sys.platform == "win32"):
            self.data_path = home + '\\PyQGIS'
        elif (sys.platform == "linux"):
            self.data_path = home + '/PyQGIS'
    
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)

        if (sys.platform == "win32"):
            self.csv_data = self.data_path + "\\data.csv"
        elif (sys.platform == "linux"):
            self.csv_data = self.data_path + "/data.csv"
    
        self.csv_file = open(self.csv_data, 'w')
        header = ['TIME','BATTERY', 'X', 'Y','LAT','LONG','ROLL','PITCH','YAW',]
        dw = csv.DictWriter(self.csv_file, delimiter=',', fieldnames=header)
        dw.writeheader()
        self.csv_file.close()

        if (sys.platform == "win32"):
            self.wpt_data = self.data_path + "\\waypoints.csv"
        elif (sys.platform == "linux"):
            self.wpt_data = self.data_path + "/waypoints.csv"

        self.wpt_file = open(self.wpt_data, 'w')
        self.wpt_file.close()
        #Variable declarations
        self.x,self.y,self.roll, self.pitch, self.yaw = 0,0,0,0,0
        self.lat, self.long = 0,0
        self.battery = 0.0
        self.point =""

    def connect_to_gcs(self):
        #Socket reception
        data = self.s.recv(1024)
        if data != b"":
            data = data.decode()
            self.res = data.strip('][').split(',')
            self.res = list(self.res)
            print(self.res[0])
            if self.res[0] == 'GPS':
                self.parse_gps()
            elif self.res[0] == 'ATTITUDE':
                self.parse_yaw()
            elif self.res[0] == 'BAD_DATA':
                print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: Bad data")
            elif self.res[0] == 'BATTERY_STATUS':
                self.parse_battery()

            #Write to CSV
            self.write_to_csv()
            
            #Read Waypoints
            self.get_waypoints()

    def get_waypoints(self):
        self.wpt_file = open(self.wpt_data, 'r')
        reader = csv.reader(self.wpt_file)
        if (sys.platform == 'win32'):
            for row in reader: 
                flag = row[2]
                if self.point != row[1]:
                    self.point = row[1]
                    self.s.sendall(self.point.encode())
                #Send service flag
                break
        elif (sys.platform == 'linux'):
            for row in reader: 
                flag = row[2]
                if self.point != row[1]:
                    self.point = row[1]
                    self.s.sendall(self.point.encode())
                #Send service flag
        self.wpt_file.close()

    def parse_gps(self):
        #['GPS', ' 40999994.0', '71000000.0', '2140.774169921875']
        self.lat = float(self.res[1]) /10e5
        self.long = float(self.res[2]) /10e5
        self.alt = round(float(self.res[3]))
        transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
        self.x, self.y = transformer.transform(self.long, self.lat)

    def parse_yaw(self):
        #[ATTITUDE, -0.01279214583337307,-0.03361418843269348,-0.01279214583337307]'
        self.roll = float(self.res[1]) 
        self.pitch = float(self.res[2])
        self.yaw = float(self.res[3])
    
    def parse_battery(self):
        #[BATTERY_STATUS, 94]
        self.battery = float(self.res[1])
        print(self.battery)

    def write_to_csv(self):
        self.csv_file = open(self.csv_data, 'a')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',',quotechar='"',quoting=csv.QUOTE_ALL)
        self.csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                  self.battery,
                                  self.x, self.y,
                                  self.lat, self.long, 
                                  self.roll, self.pitch, self.yaw, 
                                  ])
        self.csv_file.close()
        


if __name__ == "__main__":
    dir_path = os.path.dirname(os.path.realpath(__file__))
    os.chdir(dir_path)

    with open("../config/client_param.yaml", "r") as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    
    vis = Visualise(HOST=yaml_data["Socket"]["hostname"], PORT=yaml_data["Socket"]["port"])
    while 1:
        vis.connect_to_gcs()