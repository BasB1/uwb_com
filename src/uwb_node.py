#!/usr/bin/env python
import pypozyx as pzx
import rospy
import yaml
import os 
from nav_msgs.msg import Odometry
import json
from pypozyx import Data
import zlib

class Communicate(object):
    def __init__(self, pozyx, destination):
        self.destination = destination
        self.pozyx = pozyx
        self.read_data = ""
        self.odom_data = Odometry()
        self.odom_data_prev = Odometry()
        self.rx_info = pzx.RXInfo()
    
    def odomData(self, data):
        self.odom_data = data
        
    def txData(self):
        x = {
           "p":  #pose
                {"px": round(self.odom_data.pose.pose.position.x, 3), #position
                "py": round(self.odom_data.pose.pose.position.y, 3),
                "pz": round(self.odom_data.pose.pose.position.z, 3),
                "ox": round(self.odom_data.pose.pose.orientation.x, 3), #orientation
                "oy": round(self.odom_data.pose.pose.orientation.y, 3),
                "oz": round(self.odom_data.pose.pose.orientation.z, 3),
                "ow": round(self.odom_data.pose.pose.orientation.w, 3)},
            "t":  #twist
                {"lx": round(self.odom_data.twist.twist.linear.x, 3), #linear
                "ly": round(self.odom_data.twist.twist.linear.y, 3),
                "lz": round(self.odom_data.twist.twist.linear.z, 3),
                "ax": round(self.odom_data.twist.twist.angular.x, 3), #angulaer
                "ay": round(self.odom_data.twist.twist.angular.y, 3),
                "az": round(self.odom_data.twist.twist.angular.z, 3)}
            }
        s = json.dumps(x)
        comp_data = zlib.compress(str(s))
        data = Data([ord(c) for c in comp_data])
        self.pozyx.sendData(self.destination, data)
                
    def rxData(self):       
        self.pozyx.getRxInfo(self.rx_info)
        data = Data([0]*self.rx_info[1])
        self.pozyx.readRXBufferData(data)   
        message = str() 
        
        for i in data:
            message = message + chr(i)
        
        s = zlib.decompress(message)
        y = json.loads(s)
        
        odom_data_pub = Odometry()
        
        odom_data_pub.pose.pose.position.x = y['p']['px']
        odom_data_pub.pose.pose.position.y = y['p']['py']
        odom_data_pub.pose.pose.position.z = y['p']['pz']
        odom_data_pub.pose.pose.orientation.x = y['p']['ox']
        odom_data_pub.pose.pose.orientation.y = y['p']['oy']
        odom_data_pub.pose.pose.orientation.z = y['p']['oz']
        odom_data_pub.pose.pose.orientation.w = y['p']['ow']

        odom_data_pub.twist.twist.linear.x = y['t']['lx']
        odom_data_pub.twist.twist.linear.y = y['t']['ly']
        odom_data_pub.twist.twist.linear.z = y['t']['lz']
        odom_data_pub.twist.twist.angular.x = y['t']['ax']
        odom_data_pub.twist.twist.angular.y = y['t']['ay']
        odom_data_pub.twist.twist.angular.z = y['t']['az']
        
        return odom_data_pub

def main():
    try:
        odom_data = com.rxData()
        pub.publish(odom_data)
    except Exception as e:
        pass
    com.txData()

if __name__ == "__main__":
    rospy.init_node('uwb_node')
    
    serial_port = str(rospy.get_param('~serial_port', pzx.get_first_pozyx_serial_port()))
    frequency = float(rospy.get_param('~frequency', 10))
    rate = rospy.Rate(frequency)
    
    robot_number = rospy.get_param('~robot_number')
    
    protocol = str(rospy.get_param('~protocol', 'precise')) 
    
    if protocol == 'fast':
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_FAST
    elif protocol == 'precise':
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_PRECISION
    else:
        rospy.logerr("Wrong value given for protocol. Either give: 'fast' or 'precise'")
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_PRECISION
        
    pozyx = pzx.PozyxSerial(serial_port)
    pozyx.setRangingProtocol(ranging_protocol)
    
    destination = rospy.get_param('~destination', 0x6e2f)
    tx_topic = str(rospy.get_param('~tx_topic', 'uwb_server_tx'))
    rx_topic = str(rospy.get_param('~rx_topic', 'uwb_server_rx'))
    
    pub = rospy.Publisher(rx_topic, Odometry, queue_size = 10)
    com = Communicate(pozyx, destination)
    rospy.subscriber(tx_topic, Odometry, com.odomData)
    
    while not rospy.is_shutdown():
        main()         
        rate.sleep()