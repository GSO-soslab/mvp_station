#!/usr/bin/env python3
import driver
import rospy
import tf
from sensor_msgs.msg import NavSatFix, FluidPressure
from nav_msgs.msg import Odometry


class CommsROSVx:
    def __init__(self, device:str, baud:int):
        #MAVLink Connection
        self.vx = driver.VxChannel(device=device, baud=baud)
        self.tx_heartbeat()

        #ROS Subscriber Info
        sub_gps_topic = rospy.get_param("gps_topic", "/gps")
        sub_battery_topic = rospy.get_param("battery_topic", "/battery")
        sub_depth_topic = rospy.get_param("depth_topic", "/depth")
        sub_odom_topic = rospy.get_param("odometry", "/odom")
        
        rospy.Subscriber(sub_gps_topic, NavSatFix, self.tx_gps)
        rospy.Subscriber(sub_depth_topic, FluidPressure, self.tx_depth)
        rospy.Subscriber(sub_odom_topic,Odometry, self.tx_odom)
        # rospy.Subscriber(sub_battery_topic,Odometry, self.tx_odom)

        #Some global varibles
        self.depth = 0

    def tx_heartbeat(self):
        self.vx.send_hearbeat()
        rospy.loginfo("Heartbeat Sent")

    def tx_gps(self, msg):
        lat = round(msg.latitude * 10e5)
        longitude = round(msg.longitude *10e5)
        self.tx_heartbeat()
        self.vx.send_gps(latitude=lat, longitude=longitude, depth = self.depth)
        rospy.loginfo("GPS Sent")
    
    def tx_depth(self, msg):
        self.depth = msg.fluid_pressure

    def tx_odom(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        quaternion = (x,y,z,w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.tx_heartbeat()
        self.vx.send_attitude(roll=roll, pitch=pitch, yaw=yaw)
        rospy.loginfo("Attitude Sent")


if __name__ == "__main__":
    rospy.init_node("Vx_to_MAVLink")
    VxComms = CommsROSVx('/dev/ttyUSB0', 57600)
    rospy.spin()