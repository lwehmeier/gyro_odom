#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
import struct
import tf2_ros
import tf2_geometry_msgs
global tfBuffer
global scaling
global rel_z
global abs_z
global pub
global odom_pub
rel_z=0
abs_z=0
GYRO_SCALE = -1.1 #gyro scale does not quite match actual rotation
DRIFT=(0.007 - 0.00005 -0.000075)*GYRO_SCALE
global dynamic_drift
dynamic_drift = 0.0
def callbackZ(data):
    global rel_z
    rel_z = radians(data.data*GYRO_SCALE)
def callbackPose(data):
    global last_pose
    last_pose = data
def calcDynamicDrift(event):
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    pose.header.frame_id = "odom"
    pose.header.stamp = rospy.Time.now()
    transf = tfBuffer.lookup_transform("odom", "map", rospy.Time(),timeout=rospy.Duration(2))
    tgt = tf2_geometry_msgs.do_transform_pose(pose, transf)
    quaternion = [tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    drift = yaw/(rospy.Time.now().secs - runtime) #in rad/s; absolute yaw
    print("odometry: yaw: "+str(abs_z))
    print("computed drift odom<->map: "+str(drift))
    global dynamic_drift
    if True or not drift < 0.0005:
        dynamic_drift += drift
        print("new dynamic drift estmate: "+str(dynamic_drift))
def my_callback(event):
    global rel_z
    global abs_z
    abs_z = abs_z+ (rel_z-DRIFT-dynamic_drift)*0.1 #4hz
    #print(abs_z)
    pub.publish(Vector3(x=0, y=0, z=abs_z))
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "base_footprint"
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, abs_z)
    #odom_quat[3]=1.0
    odom.pose.pose = Pose(Point(0., 0., 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "odom"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3( 0, 0, rel_z))
    odom_pub.publish(odom)

rospy.init_node('gyro_odom')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
global runtime
runtime = rospy.Time.now().secs
odom_pub = rospy.Publisher("/gyro/odom", Odometry, queue_size=3)
pub = rospy.Publisher("/gyro/sum", Vector3, queue_size=3)
rospy.Subscriber("/gyro/z", Float32, callbackZ)
rospy.Subscriber("/slam_out_pose", PoseStamped, callbackPose)
rospy.Timer(rospy.Duration(0.1), my_callback)
#rospy.Timer(rospy.Duration(60), calcDynamicDrift) # update drift estimate once per minute
r = rospy.Rate(10) # 10hz
rospy.spin()
