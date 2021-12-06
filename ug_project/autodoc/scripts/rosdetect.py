#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Detector:

    def __init__(self):

      self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.image_cb, queue_size=1, buff_size=2**24)

    def image_cb(self, data):
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
        print(e)
      #image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
      print("Saving Image")
      cv2.imwrite('/home/kartik/catkin_ws/src/ug_project/autodoc/scripts/woundimage.png',cv_image)
      print("Done Saving")
      return cv_image

def main(args):

  rospy.init_node('detector_node')
  img=Detector()
  try:
    rospy.sleep(0.5)
  except KeyboardInterrupt:
    print("ShutDown")
  cv2.destroyAllWindows()

if __name__=='__main__':
  main(sys.argv)
