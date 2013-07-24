#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_comms')
import sys
import rospy
from robot_comms.srv import *

def hello_client(ID):
    rospy.wait_for_service('hello_filter')
    try:
        function = rospy.ServiceProxy('hello_filter', ImageFilter) #handle for calling the servie
        resp1 = function(ID) ##now we can call it normally 
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    hello_client(1)
    print "Sent"
