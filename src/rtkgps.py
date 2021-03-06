#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry.msg import PoseWithCovariance
import numpy as np
from sensor_msgs.msg import Range

position1 = np.array([0,0,0])
position2 = np.array([0,0,0])

def calculate_real_range():
    rospy.init_node('rtk_range', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('rtk_range', Range, queue_size=10)
    rospy.Subscriber("/ak1/piksi_multi/enu_pose_best_fix",PoseWithCovariance, set_pose_1)
    rospy.Subsciber("/ak2/piksi_multi/enu_pose_best_fix",PoseWithCovariance, set_pose_2)
    while not rospy.is_shutdown():
        real_range = np.linalg.norm(position1-position2)
        range = Range()
        range.range = real_range
        pub.publish(range)
        rate.sleep()

def set_pose_1(msg):
    global position1
    position1 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

def set_pose_2(msg):
    global position2
    position2 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

if __name__ == '__main__':
    try:
        calculate_real_range()
    except rospy.ROSInterruptException:
        pass
