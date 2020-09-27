#!/usr/bin/env python2
import sys
import os

rootDir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.append(os.path.join(rootDir, 'devel', 'lib', 'python2.7', 'dist-packages'))

import rospy
from ros_msgs.msg import IsAttached
from ros_msgs.srv import Attach
from ros_msgs.srv import Detach

def callSimpleService(url, msg):
    rospy.wait_for_service(url)
    try:
        service = rospy.ServiceProxy(url, msg)
        response = service()
        return response.ok
    except rospy.ServiceException as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)

    isAttachedPublisher = rospy.Publisher(
        '/example/is_attached', IsAttached, queue_size=3
    )

    key = ''
    while(key not in ['q', 'Q']):
        key = raw_input(
            '________________________\n'+
            '| A : attach           |\n'+
            '| D : dttach           |\n'+
            '| S : show is attached |\n'+
            '| Q : quit             |\n'+
            '------------------------\n'+
            'please input >> '
        )

        if key in ['a', 'A']:
            callSimpleService('/example/attach', Attach)
        elif key in ['d', 'D']:
            callSimpleService('/example/detach', Detach)
        elif key in ['s', 'S']:
            isAttachedPublisher.publish()
