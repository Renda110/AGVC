#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_comms')
from robot_comms.srv import *
import rospy

state = 0
def handle_filter(req):
    print "State =  %d"%(req.filter_id)
    global state
    state = req.filter_id   
    return ImageFilterResponse(1)

def start_server():
    rospy.init_node('robot_comm_node')
    s = rospy.Service('image_filter', ImageFilter, handle_filter)
    rospy.spin()

if __name__ == "__main__":
    start_server()
