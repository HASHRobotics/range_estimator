#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
import numpy as np
from sensor_msgs.msg import Range
from piksi_rtk_msgs.msg import BaselineNed

position1 = np.array([0,0,0])
position2 = np.array([0,0,0])

def calculate_real_range():
    rospy.init_node('rtk_range', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('rtk_range', Range, queue_size=10)
    rospy.Subscriber("/ak2/piksi/baseline_ned",BaselineNed, set_pose_2)
    # rospy.Subsciber("/ak2/piksi_multi/enu_pose_best_fix",PoseWithCovariance, set_pose_2)
    global position2
    global position1
    while not rospy.is_shutdown():
        real_range = np.linalg.norm(position1-position2)
        range = Range()
        range.range = real_range
        range.header.stamp = rospy.get_rostime()
        pub.publish(range)
        rate.sleep()

# def set_pose_1(msg):
#     global position1
#     position1 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

def set_pose_2(msg):
    global position2
    position2 = np.array([msg.e, msg.n])

if __name__ == '__main__':
    try:
        calculate_real_range()
    except rospy.ROSInterruptException:
        pass
