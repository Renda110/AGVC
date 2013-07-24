#!/usr/bin/env python
import roslib
roslib.load_manifest('robot_comms')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

from robot_comms.srv import *


##TO be run on the robot
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("video_output",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) #import the image
    self.filterState = 0

    s = rospy.Service('change_image_filter', ImageFilter, self.filter_change)	

  def filter_change(self,req):
    print "filter changed"
    self.filterState = req.filter_id   
    return ImageFilterResponse(1)

  def callback(self,data):
    try:
      #self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8") #convert to openCV
      self.cv_image = self.bridge.imgmsg_to_cv(data)	
    except CvBridgeError, e:
      print e
#Run algorithm 
    if self.filterState == 0:
	    self.drawCircleRED() 
    elif self.filterState == 1:
	   self.drawCircle()
     #Wait a bit   
    cv.WaitKey(3)
	
#Publish the image
    try:
        self.image_pub.publish(self.bridge.cv_to_imgmsg(self.cv_image, "bgr8"))
    	    #  self.image_pub.publish(self.bridge.cv_to_imgmsg(self.cv_image))
    except CvBridgeError, e:
      print e

		
  def edgeDetection(self):

	    nump_im=np.asarray(self.cv_image)
	    img_blur = cv2.GaussianBlur(nump_im, (21, 21), 0)
	    img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
	    imgray = cv2.cvtColor(img_blur,cv2.COLOR_BGR2GRAY)
	    ret,thresh = cv2.threshold(imgray,127,255,0)
    
	    #contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	    #cv2.drawContours(imgray,contours,-1,(0,255,0),3)
	    self.cv_image = cv.fromarray(thresh)

  def drawCircle(self):

    	(cols,rows) = cv.GetSize(self.cv_image)
   	if cols > 60 and rows > 60 :
      	    cv.Circle(self.cv_image, (200,200), 50, 255,50)	

  def drawCircleRED(self):

    	(cols,rows) = cv.GetSize(self.cv_image)
   	if cols > 60 and rows > 60 :
      	    cv.Circle(self.cv_image, (200,200), 50, (0,0,255),50)
	
def main(args):
  rospy.init_node('image_converter', anonymous=True)	
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
