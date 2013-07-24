#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_comms')
from robot_comms.srv import *
from robot_comms.msg import *
import rospy

#server run on the Robots
class RobotServer:

    def __init__(self):
	state = 0
	rospy.init_node('robot_comm_node')
 	#self.image_pub = rospy.Publisher("robot_comms_output",Image)
	rospy.Subscriber("/base_comms_output",baseMsg,self.main_callback)
	rospy.spin()


#Subscribe to the home base topic 
	
    def main_callback(self,data):

	##change image filter
	if data.type == 1:
	     self.changeImageFilter(data.message)

#Change image filter, send to image module service 

    def changeImageFilter(self,ID):
	   #rospy.wait_for_service('hello_filter')
	   
	    try:
	        function = rospy.ServiceProxy('change_image_filter', ImageFilter) #handle for calling the servie
	        resp1 = function(ID) ##now we can call it normally 
	        return resp1.result
	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e



#base client call, call to the robot
#Send a message to other com unit 	 

#Send Robot goal

#Change robot Image filter

#Change robot mode



if __name__ == "__main__":
   rs = RobotServer()
