#!/usr/bin/env python3
import driver
import rospy
import time
import tf
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
import threading
import os
import yaml

class CommsROSVx:
    def __init__(self, device:str, baud:int):
        #MAVLink Connection
        self.vx = driver.VxChannel(device=device, baud=baud)
        self.tx_heartbeat()
        
        #ROS Subscriber Info
        self.enable_gps = rospy.get_param("enable_gps", True)
        sub_gps_topic = rospy.get_param("gps_topic", "/gps")

        self.enable_battery = rospy.get_param("enable_battery", True)
        sub_battery_topic = rospy.get_param("battery_topic", "/battery")
        
        self.enable_depth = rospy.get_param("enable_depth", True)
        sub_depth_topic = rospy.get_param("depth_topic", "/depth")
        
        self.enable_odom = rospy.get_param("enable_odom", True)
        sub_odom_topic = rospy.get_param("odometry", "/odom")
        
        #Queue size = 1 to get the latest message.
        rospy.Subscriber(sub_gps_topic, NavSatFix, self.tx_gps, queue_size=1)
        rospy.Subscriber(sub_depth_topic, FluidPressure, self.tx_depth, queue_size=1)
        rospy.Subscriber(sub_odom_topic,Odometry, self.tx_odom, queue_size=1)
        rospy.Subscriber(sub_battery_topic,Int16, self.tx_battery, queue_size=1)

        #Some global varibles
        self.depth = 0
        self.x = threading.Thread(target=self.read_wp).start()
        self.waypoints = []
        self.count = 0

    def read_wp(self):
        while True:
            msg = self.vx.recv_match(type=["MISSION_CLEAR_ALL","MISSION_ITEM", "MISSION_COUNT"]) 
            print("Received Waypoints from gcs:", msg)
            if msg != None:
                if msg.get_type() == "MISSION_CLEAR_ALL":
                    self.waypoints.clear()
                elif msg.get_type() == "MISSION_COUNT":
                    self.count = msg.count
                elif msg.get_type() == "MISSION_ITEM":
                    lat = msg.x
                    long = msg.y
                    self.waypoints.append([lat, long])
            if ((self.waypoints != []) and (len(self.waypoints) == self.count)):
                    print(self.waypoints)
            time.sleep(1)


    #Send Hearbeat.
    def tx_heartbeat(self):
        self.vx.send_hearbeat()
        rospy.loginfo("Heartbeat Sent")

    #Send GPS
    def tx_gps(self, msg):
        if self.enable_gps:
            lat = round(msg.latitude * 10e5)
            longitude = round(msg.longitude *10e5)
            self.tx_heartbeat()
            self.vx.send_gps(latitude=lat, longitude=longitude, depth = self.depth)
            # rospy.loginfo("GPS Sent")
    
    def tx_depth(self, msg):
        if self.enable_depth:
            self.depth = msg.fluid_pressure

    #Send Attitude
    def tx_odom(self, msg):
        if self.enable_odom:
            x = msg.pose.pose.orientation.x
            y = msg.pose.pose.orientation.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([x,y,z,w])
            self.tx_heartbeat()
            self.vx.send_attitude(roll=np.degrees(roll), pitch=np.degrees(pitch), yaw=np.degrees(yaw))
            time.sleep(0.5)
            # rospy.loginfo("Attitude Sent")

    def tx_battery(self, msg):
        if self.enable_battery:
            self.vx.send_battery(msg.data)
            # rospy.loginfo("Battery Sent")

if __name__ == "__main__":
    dir_path = os.path.dirname(os.path.realpath(__file__))
    os.chdir(dir_path)

    with open("../config/vehicle_param.yaml", "r") as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    rospy.init_node("Vx_to_MAVLink")
    VxComms = CommsROSVx(device=yaml_data["MAVLink"]["connection_string"], baud=yaml_data["MAVLink"]["baud"])
    rospy.spin()