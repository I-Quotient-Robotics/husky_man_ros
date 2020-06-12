#!/usr/bin/env python
import socket
import time
import rospy
from std_msgs.msg import UInt32MultiArray

class ArucoDetectClient:
  def __init__(self):
    self._sub = rospy.Subscriber("aruco_marker_publisher/markers_list", UInt32MultiArray, self.arucoDetectCallback)

    try:
      self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.server_address = ("192.168.43.40", 8848)
      # self.server_address = ("", 8848)

    except:
      print 'Build UDP socket error!! exit~'
      exit(-1)

  def arucoDetectCallback(self, msg):
  	marker_id = "/"
  	marker = msg.data
	for index in range(len(marker)):
	  marker_id = str(marker[index]) + marker_id
	marker_id = marker_id + str(len(marker))
	self.client_socket.sendto(marker_id, self.server_address)

def main():
  rospy.init_node('aruco_detect_client')
  ADC = ArucoDetectClient()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass