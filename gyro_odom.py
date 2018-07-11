#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import struct
global scaling
global rel_z
global abs_z
global pub
global odom_pub
rel_z=0
abs_z=0
def callbackZ(data):
    global rel_z
    rel_z = radians(data.data)
def my_callback(event):
    global rel_z
    global abs_z
    abs_z = abs_z+ rel_z*0.1
    pub.publish(Vector3(x=0, y=0, z=abs_z))
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "gyro_odom"
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, abs_z)
    #odom_quat[3]=1.0
    odom.pose.pose = Pose(Point(0., 0., 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3( 0, 0, rel_z))
    odom_pub.publish(odom)

rospy.init_node('gyro_odom')
rospy.Timer(rospy.Duration(0.1), my_callback)
odom_pub = rospy.Publisher("/gyro/odom", Odometry, queue_size=3)
pub = rospy.Publisher("/gyro/sum", Vector3, queue_size=3)
rospy.Subscriber("/gyro/z", Float32, callbackZ)
r = rospy.Rate(10) # 10hz
rospy.spin()
