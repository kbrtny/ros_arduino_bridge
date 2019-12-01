#!/usr/bin/env python

import rospy
from ros_arduino_python.lsm6 import LSM6
from sensor_msgs.msg import Imu
import thread

class RomiIMU():
    def __init__(self):
        rospy.init_node('RomiIMU', log_level=rospy.DEBUG)
        
        rospy.on_shutdown(self.shutdown)
        
        rate = rospy.Rate(100)
        
        self.imu_msg = Imu()
        
        self.mutex = thread.allocate_lock()
        self.imu = LSM6()
        
        self.imu_pub = rospy.Publisher('raw', Imu, queue_size=5)
        self.mutex.acquire()
        self.imu.enable()
        self.mutex.release()
        
        while not rospy_is_shutdown():
            self.mutex.acquire()
            self.imu.read()
            self.mutex.release()
            
            self.imu_msg.header.stamp = rospy.get_rostime()
            
            self.imu_msg.angular_velocity.x = self.imu.g.x
            self.imu_msg.angular_velocity.y = self.imu.g.y
            self.imu_msg.angular_velocity.z = self.imu.g.z
            
            self.imu_msg.linear_acceleration.x = self.imu.a.x
            self.imu_msg.linear_acceleration.y = self.imu.a.y
            self.imu_msg.linear_acceleration.z = self.imu.a.z
            
            self.imu_pub.publish(self.imu_msg)
            
            rate.sleep()
            
    def shutdown(self):
        
if __name__ == '__main__':
    myRomiImu = RomiIMU()
