#!/usr/bin/env python
import rospy
import serial
import time
from sensor_msgs.msg import Range
from collections import deque
from statistics import median

def get_median(q):
    sorted_list = sorted(q)
    return median(sorted_list)

def get_range(median,mean):
    if(abs(median-mean)<0.15):
        print("Returning mean \n")
        return mean
    else:
        print("Returning median \n")
        return median

def distance_measurement():
    pub = rospy.Publisher('estimate_range', Range, queue_size=10)
    rospy.init_node('decawave', anonymous=True)
    rate = rospy.Rate(10)
    number_of_elements_to_consider_for_filtering = 10
    while not rospy.is_shutdown():
        deca_port = rospy.get_param('USB_port')
        #print(deca_port)
        ser = serial.Serial(
            port = deca_port,
            baudrate=115200,
            timeout=0.1
        )
        #port='/dev/ttyACM0'

        ser.write(b'\r\r')
        res=ser.read(100000)
        time.sleep(1)
        ser.write(b'lec\r')
        print(res)

        q = deque()
        total = 0
        while True:
            res=ser.read(100)
            distance = Range()

            if len(res)>0:
                readings = res.split('\r\n')
                for reading in readings:
                    if 'DIST' in reading:
                        try:
                            range_deca = float(reading.split(',')[-1])
                            if range_deca:
                                q.append(range_deca)
                                if len(q) <= number_of_elements_to_consider_for_filtering:
                                    total += range_deca
                                else:
                                    removed_element = q.popleft()
                                    print(q)
                                    total = total - removed_element + range_deca
                                    mean = total/len(q)
                                    median = get_median(q)
                                    distance.range = get_range(median,mean)
                                    #distance.range = total/len(q)
                                    distance.header.stamp = rospy.get_rostime()
                                    rospy.loginfo(distance)
                                    pub.publish(distance)
                        except ValueError as e:
                            rospy.loginfo("A string found")

if __name__ == "__main__":
    try:
        distance_measurement()
    except rospy.ROSInterruptException:
        pass
