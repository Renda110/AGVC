#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_comms')
from robot_comms.srv import *
import rospy

state = 0
def handle_hello(req):
    print "State =  %d"%(req.filter_id)
    global state
    state = req.filter_id   
    return ImageFilterResponse(1)

def hello_server():
    rospy.init_node('Hello_server')
    s = rospy.Service('hello_filter', ImageFilter, handle_hello)
    print state
    rospy.spin()

if __name__ == "__main__":
    hello_server()
