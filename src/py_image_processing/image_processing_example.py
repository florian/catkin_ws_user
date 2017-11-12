#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')

import numpy as np

from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
    self.gray_pub = rospy.Publisher("/image_processing/gray_img",Image, queue_size=1)
    self.threshold_pub = rospy.Publisher("/image_processing/threshold_img",Image, queue_size=1)



    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    self.gray_pub.publish(self.bridge.cv2_to_imgmsg(gray,"mono8"))

    #bi_gray
    bi_gray_max = 245
    bi_gray_min = 215
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
    self.threshold_pub.publish(self.bridge.cv2_to_imgmsg(thresh1,"mono8"))

    #gauss
    MAX_KERNEL_LENGTH = 2;
    i= 5
    dst=cv2.GaussianBlur(cv_image,(5,5),0,0)

    #edge
    dx = 1;
    dy = 1;
    ksize = 3; #1,3,5,7
    scale = 1
    delta = 0
    edge_img=cv2.Sobel(thresh1, cv2.CV_8UC1, dx, dy, ksize, scale, delta, cv2.BORDER_DEFAULT)

    #bi_rgb
    r_max = 244;
    r_min = 0;
    g_max = 255;
    g_min = 0;
    b_max = 255;
    b_min = 0;

    thresh_flatten = thresh1.flatten()
    bool_lst = (thresh_flatten>=245).reshape(thresh1.shape)
    white_points =  zip(*np.where(bool_lst))

    objectPoints = np.array([
    [28, 0, 0],
    [28, 23, 0],
    [60, 0, 0],
    [60, 23, 0],
    [90, 0, 0],
    [90, 23, 0] ], dtype = np.float).reshape((6,3,1))

    imagePoints = np.array([
    [160, 420],
    [160, 310],
    [225, 450],
    [220, 300],
    [340, 500],
    [320, 290] ], dtype = np.float).reshape((6, 2, 1))

    cameraMatrix = np.array([
    [614.1699, 0, 329.9491],
    [0, 614.9002, 237.2788],
    [0, 0, 1] ])

    distCoeffs = np.array([[0.1115, -0.1089, 0, 0]])

    rotation_vector = np.zeros((3,1))
    translation_vector = np.zeros((3,1))


    _, rotation_vector, translation_vector = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)


    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

    inv_rotation_matrix = rotation_matrix.T
    inv_rotation_vector = np.dot(-inv_rotation_matrix,rotation_vector)

    yaw = np.arctan2(inv_rotation_matrix[1,0], inv_rotation_matrix[0,0])
    pitch = np.arctan2(-inv_rotation_matrix[2,0], np.sqrt(np.power(inv_rotation_matrix[2,1],2) + np.power(inv_rotation_matrix[2,2],2)))
    roll = np.arctan2(inv_rotation_matrix[2,1],inv_rotation_matrix[2,2] )


    print "yaw", yaw
    print "pitch", pitch
    print "roll",roll


    projected_points, _ = cv2.projectPoints(objectPoints, rotation_vector, translation_vector, cameraMatrix, distCoeffs)    
    print projected_points




    #bi_rgb = cv2.merge((b,g,r))
    #bi_hsv
    h_max = 255;
    h_min = 0;
    s_max = 255;
    s_min= 0;
    v_max = 252;
    v_min = 0;
    hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    h,s,v = cv2.split(hsv)

    for j in xrange(hsv.shape[0]):
      for i in xrange(hsv.shape[1]):
        if  (v[j,i]>= v_min and v[j,i]<= v_max and s[j,i]>= s_min and s[j,i]<= s_max and h[j,i]>= h_min and h[j,i]<= h_max):
          h[j,i]=0
          s[j,i]=0
          v[j,i]=0
        else:
          h[j,i]=255
          s[j,i]=255
          v[j,i]=255

    bi_hsv = cv2.merge((h,s,v))

    # titles = ['Original Image', 'GRAY','BINARY','GAUSS','EDGE','BI_RGB','BI_HSV']
    # images = [cv_image, gray, thresh1,dst,edge_img,bi_rgb,bi_hsv]
    #
    # for i in xrange(7):
    #   plt.subplot(2,4,i+1),plt.imshow(images[i],'gray')
    #   plt.title(titles[i])
    #   plt.xticks([]),plt.yticks([])
    #
    # plt.show()
    # print("Done")

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)
      

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
