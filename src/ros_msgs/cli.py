#!/usr/bin/env python
import sys
import os

rootDir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.append(os.path.join(rootDir, 'devel', 'lib', 'python2.7', 'dist-packages'))

import rospy
from ros_msgs.msg import IsAttached

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/gazebo/is_attached', IsAttached, queue_size=3)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        pub.publish()
        rate.sleep()

if __name__ == '__main__':
    talker()
