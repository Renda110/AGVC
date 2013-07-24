#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_comms')
from robot_comms.srv import *
import rospy

#server run on the home base
class BaseServer:

    def __init__(self):
        self.setupServer()
	state = 0
	def handle_hello(req):
	    print "State =  %d"%(req.filter_id)
	    global state
	    state = req.filter_id   
	    return ImageFilterResponse(1)

	def setupServer(self):

 	    #Set up the base services 
	    s = rospy.Service('hello_filter', ImageFilter, handle_hello)
	    print state
	    rospy.spin()


    #base client call, call to the robot
    #Send a message to other com unit 	 

#Send Robot goal

#Change robot Image filter

#Change robot mode
