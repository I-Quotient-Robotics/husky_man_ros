#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib

from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest
from kinova_msgs.msg import PoseVelocity, JointVelocity, FingerPosition
from kinova_msgs.msg import SetFingersPositionAction
from kinova_msgs.msg import SetFingersPositionGoal
from kinova_msgs.msg import SetFingersPositionActionGoal
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from kinova_msgs.srv import HomeArm, Start, Stop
from std_msgs.msg import UInt32MultiArray
from sensor_msgs.msg import Joy
from keyboard.msg import Key

class BrainControlInterface:
  def __init__(self):
    self._kinova_pose_init = Pose()
    self._sub = rospy.Subscriber("left_arm_driver/out/finger_position", FingerPosition, self.leftFingerPositionInit)
    self._sub = rospy.Subscriber("right_arm_driver/out/finger_position", FingerPosition, self.rightFingerPositionInit)
    self._sub = rospy.Subscriber("keyboard/keydown", Key, self.KeyboradDownCallBack)
    self._sub = rospy.Subscriber("keyboard/keyup", Key, self.KeyboradUpCallBack)
    self._pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    self._pub_left_pose_velocity = rospy.Publisher("left_arm_driver/in/cartesian_velocity", PoseVelocity, queue_size=1)
    self._pub_left_joint_velocity = rospy.Publisher('left_arm_driver/in/joint_velocity', JointVelocity, queue_size=1)
    self._pub_right_pose_velocity = rospy.Publisher("right_arm_driver/in/cartesian_velocity", PoseVelocity, queue_size=1)
    self._pub_right_joint_velocity = rospy.Publisher('right_arm_driver/in/joint_velocity', JointVelocity, queue_size=1)
    self._pub_finger_position_left = rospy.Publisher('left_arm_driver/fingers_action/finger_positions/goal', SetFingersPositionActionGoal, queue_size=10)
    self._pub_finger_position_right = rospy.Publisher('right_arm_driver/fingers_action/finger_positions/goal', SetFingersPositionActionGoal, queue_size=10)
    self._finger_left = actionlib.SimpleActionClient('left_arm_driver/fingers_action/finger_positions', SetFingersPositionAction)
    self._finger_right = actionlib.SimpleActionClient('left_arm_driver/fingers_action/finger_positions', SetFingersPositionAction)
    self._home_client_left = rospy.ServiceProxy('left_arm_driver/in/home_arm', HomeArm)
    self._home_client_right = rospy.ServiceProxy('right_arm_driver/in/home_arm', HomeArm)
    self._driver_stop_client_left = rospy.ServiceProxy('/left_arm_driver/in/stop', Stop)
    self._driver_start_client_left = rospy.ServiceProxy('/left_arm_driver/in/start', Start)
    self._driver_stop_client_right = rospy.ServiceProxy('/left_arm_driver/in/stop', Stop)
    self._driver_start_client_right = rospy.ServiceProxy('/left_arm_driver/in/start', Start)

    self.vel_x_base = 0
    self.vel_z_base = 0
    self.vel_scale_base = 0.2
    self.vel_x_left = 0
    self.vel_y_left = 0
    self.vel_z_left = 0
    self.vel_x_right = 0
    self.vel_y_right = 0
    self.vel_z_right = 0

    self.vel_joint1_left = 0
    self.vel_joint2_left = 0
    self.vel_joint3_left = 0
    self.vel_joint4_left = 0
    self.vel_joint5_left = 0
    self.vel_joint6_left = 0

    self.vel_joint1_right = 0
    self.vel_joint2_right = 0
    self.vel_joint3_right = 0
    self.vel_joint4_right = 0
    self.vel_joint5_right = 0
    self.vel_joint6_right = 0

    self.finger_left = 0
    self.finger_right = 0

    self.control_mode_left = 0
    self.control_mode_right = 0

    self.vel_left = 1
    self.vel_right = 1

    self.joint_direction = 1

    self.left_stop = True
    self.right_stop = True
    self.finger_stop = True
    self.allstart = 0
    self.driver_start = 0
    self.left_joint_stop = True
    self.right_joint_stop = True
    self.base_stop = True

    self.pose_vel_left = PoseVelocity()
    self.pose_vel_right = PoseVelocity()
    self.joint_vel_left = JointVelocity()
    self.joint_vel_right = JointVelocity()
    self.finger_positions_left = SetFingersPositionGoal()
    self.finger_positions_right = SetFingersPositionGoal()
    self.finger_positions_msg_left = SetFingersPositionActionGoal()
    self.finger_positions_msg_right = SetFingersPositionActionGoal()
    self.vel_cmd_msg = Twist()

  def KeyboradDownCallBack(self, state):

    if state.code == 119:
      self.vel_x_left = -1
      rospy.loginfo("left x back")
    elif state.code == 115:
      self.vel_x_left = 1
      rospy.loginfo("left x forward")
    elif state.code == 97:
      rospy.loginfo("left y forward")
      self.vel_y_left = 1
    elif state.code == 100:
      rospy.loginfo("left y back")
      self.vel_y_left = -1
    elif state.code == 113:
      rospy.loginfo("left z back")
      self.vel_z_left = -1
    elif state.code == 101:
      rospy.loginfo("left z forward")
      self.vel_z_left = 1
    elif state.code == 105:
      rospy.loginfo("right x forward")
      self.vel_x_right = 1
    elif state.code == 107:
      rospy.loginfo("right x back")
      self.vel_x_right = -1
    elif state.code == 106:
      rospy.loginfo("right y back")
      self.vel_y_right = -1
    elif state.code == 108:
      rospy.loginfo("right y forward")
      self.vel_y_right = 1
    elif state.code == 117:
      rospy.loginfo("right z back")
      self.vel_z_right = -1
    elif state.code == 111:
      rospy.loginfo("right z forward")
      self.vel_z_right = 1
    elif state.code ==  122:
      rospy.loginfo("left linear mode")
      self.control_mode_left = 0
    elif state.code == 120:
      rospy.loginfo("left angular mode")
      self.control_mode_left = 1
    elif state.code == 118:
      if self.vel_left <= 1.9:
        rospy.loginfo("left vel up")
        self.vel_left += 0.1 
    elif state.code == 98:
      if self.vel_left >= 0.3:
        rospy.loginfo("left vel down")
        self.vel_left -= 0.1
    elif state.code ==  47:
      rospy.loginfo("right linear mode")
      self.control_mode_right = 0
    elif state.code == 46:
      rospy.loginfo("right angular mode")
      self.control_mode_right = 1
    elif state.code == 109:
      if self.vel_right <= 1.9:
        rospy.loginfo("left vel up")
        self.vel_right += 0.1 
    elif state.code == 110:
      if self.vel_right >= 0.3:
        rospy.loginfo("left vel down")
        self.vel_right -= 0.1 
    elif state.code == 114:
      self.fingerControl(3000, self._finger_left)
      rospy.loginfo("left finger close")
    elif state.code == 102:
      rospy.loginfo("left finger open")
      self.fingerControl(0, self._finger_left)
    elif state.code == 104:
      rospy.loginfo("right finger close")
      self.fingerControl(3000, self._finger_right)
    elif state.code == 121:
      rospy.loginfo("right finger open")
      self.fingerControl(0, self._finger_right)

    elif state.code == 13:
      self.allstart += 1
      self.allstart = self.allstart%2
      rospy.loginfo("keyboard control start")
    elif state.code == 27:
      self.driver_start += 1
      self.driver_start = self.driver_start%2
      if self.driver_start == 1:
        self._driver_stop_client_left.call()
        self._driver_stop_client_right.call()
        rospy.loginfo("stop all driver")
      if self.driver_start == 0:
        self._driver_start_client_left.call()
        self._driver_start_client_right.call()
        rospy.loginfo("start all driver")

    elif state.code == 103:
      rospy.loginfo("left home")
      self._home_client_left.call()
    elif state.code == 116:
      rospy.loginfo("right home")
      self._home_client_right.call()

    elif state.code == 273:
      rospy.loginfo("base x forward")
      self.vel_x_base = 1
    elif state.code == 274:
      rospy.loginfo("base x back")
      self.vel_x_base = -1
    elif state.code == 275:
      rospy.loginfo("base z base")
      self.vel_z_base = -1
    elif state.code == 276:
      rospy.loginfo("base z forward")
      self.vel_z_base = 1
    elif state.code == 280:
      if self.vel_scale_base <= 1.0:
        rospy.loginfo("base vel up")
        rospy.loginfo("%f", self.vel_scale_base)
        self.vel_scale_base += 0.1
    elif state.code == 281:
      if self.vel_scale_base >= 0:
        rospy.loginfo("base vel down")
        rospy.loginfo("%f", self.vel_scale_base)
        self.vel_scale_base -= 0.1

    elif state.code == 96:
      self.joint_direction = -1

    elif state.code == 49:
      self.joint_vel_left.joint1 = 10.0 * self.vel_left * self.joint_direction
      self.vel_joint1_left = self.joint_direction
    elif state.code == 50:
      self.vel_joint2_left = self.joint_direction
      self.joint_vel_left.joint2 = 10.0 * self.vel_left * self.joint_direction
    elif state.code == 51:
      self.vel_joint3_left = self.joint_direction
      self.joint_vel_left.joint3 = 10.0 * self.vel_left * self.joint_direction
    elif state.code == 52:
      self.vel_joint4_left = self.joint_direction
      self.joint_vel_left.joint4 = 10.0 * self.vel_left * self.joint_direction
    elif state.code == 53:
      self.vel_joint5_left = self.joint_direction
      self.joint_vel_left.joint5 = 10.0 * self.vel_left * self.joint_direction
    elif state.code == 54:
      self.vel_joint6_left = self.joint_direction
      self.joint_vel_left.joint6 = 10.0 * self.vel_left * self.joint_direction
    elif state.code == 55:
      self.joint_vel_right.joint1 = 10.0 * self.vel_right * self.joint_direction
      self.vel_joint1_right = self.joint_direction
    elif state.code == 56:
      self.vel_joint2_right = self.joint_direction
      self.joint_vel_right.joint2 = 10.0 * self.vel_right * self.joint_direction
    elif state.code == 57:
      self.vel_joint3_right = self.joint_direction
      self.joint_vel_right.joint3 = 10.0 * self.vel_right * self.joint_direction
    elif state.code == 48:
      self.vel_joint4_right = self.joint_direction
      self.joint_vel_right.joint4 = 10.0 * self.vel_right * self.joint_direction
    elif state.code == 45:
      self.vel_joint5_right = self.joint_direction
      self.joint_vel_right.joint5 = 10.0 * self.vel_right * self.joint_direction
    elif state.code == 61:
      self.vel_joint6_right = self.joint_direction
      self.joint_vel_right.joint6 = 10.0 * self.vel_right * self.joint_direction

    if self.vel_x_left != 0 or self.vel_y_left != 0 or self.vel_z_left != 0 :
      if self.control_mode_left == 0:
        self.pose_vel_left.twist_linear_x = self.vel_x_left * self.vel_left
        self.pose_vel_left.twist_linear_y = self.vel_y_left * self.vel_left
        self.pose_vel_left.twist_linear_z = self.vel_z_left * self.vel_left
      if self.control_mode_left == 1:
        self.pose_vel_left.twist_angular_x = self.vel_x_left * self.vel_left
        self.pose_vel_left.twist_angular_y = self.vel_y_left * self.vel_left
        self.pose_vel_left.twist_angular_z = self.vel_z_left * self.vel_left
      self.left_stop = False

    if self.vel_x_right != 0 or self.vel_y_right != 0 or self.vel_z_right != 0 :
      if self.control_mode_right == 0:
        self.pose_vel_right.twist_linear_x = self.vel_x_right * self.vel_right
        self.pose_vel_right.twist_linear_y = self.vel_y_right * self.vel_right
        self.pose_vel_right.twist_linear_z = self.vel_z_right * self.vel_right
      if self.control_mode_right == 1:
        self.pose_vel_right.twist_angular_x = self.vel_x_right * self.vel_right
        self.pose_vel_right.twist_angular_y = self.vel_y_right * self.vel_right
        self.pose_vel_right.twist_angular_z = self.vel_z_right * self.vel_right
      self.right_stop = False

    if self.vel_joint1_left != 0 or self.vel_joint2_left != 0 or self.vel_joint3_left != 0 or self.vel_joint4_left != 0 or self.vel_joint5_left != 0 or self.vel_joint6_left != 0:
      self.left_joint_stop = False      
    if self.vel_joint1_right != 0 or self.vel_joint2_right != 0 or self.vel_joint3_right != 0 or self.vel_joint4_right != 0 or self.vel_joint5_right != 0 or self.vel_joint6_right != 0:
      self.right_joint_stop = False      

    if self.left_stop == False or self.right_stop == False or self.right_joint_stop == False or self.left_joint_stop == False and self.allstart == 1:
      self.pubThread(self.pose_vel_left, self.pose_vel_right, self.joint_vel_left, self.joint_vel_right)
 
    if self.vel_x_base != 0 or self.vel_z_base != 0:
      self.base_stop = False
      self.pubBaseThread(self.vel_x_base, self.vel_z_base)
      
  def KeyboradUpCallBack(self, keys):
  
    if keys.code == 49:
      self.vel_joint1_left = 0
      self.joint_vel_left.joint1 = 0.0
    elif keys.code == 50:
      self.vel_joint2_left = 0
      self.joint_vel_left.joint2 = 0.0
    elif keys.code == 51:
      self.vel_joint3_left = 0
      self.joint_vel_left.joint3 = 0.0
    elif keys.code == 52:
      self.vel_joint4_left = 0
      self.joint_vel_left.joint4 = 0.0
    elif keys.code == 53:
      self.vel_joint5_left = 0
      self.joint_vel_left.joint5 = 0.0
    elif keys.code == 54:
      self.vel_joint6_left = 0
      self.joint_vel_left.joint6 = 0.0
    elif keys.code == 55:
      self.vel_joint1_right = 0
      self.joint_vel_right.joint1 = 0.0
    elif keys.code == 56:
      self.vel_joint2_right = 0
      self.joint_vel_right.joint2 = 0.0
    elif keys.code == 57:
      self.vel_joint3_right = 0
      self.joint_vel_right.joint3 = 0.0
    elif keys.code == 48:
      self.vel_joint4_right = 0
      self.joint_vel_right.joint4 = 0.0
    elif keys.code == 45:
      self.vel_joint5_right = 0
      self.joint_vel_right.joint5 = 0.0
    elif keys.code == 61:
      self.vel_joint6_right = 0
      self.joint_vel_right.joint6 = 0.0

    elif keys.code == 96:
      self.joint_direction = 1

    if keys.code == 273 or keys.code == 274:
      self.vel_x_base = 0

    if keys.code == 275 or keys.code == 276:
      self.vel_z_base = 0

    if keys.code == 119 or keys.code == 115:
      self.pose_vel_left.twist_linear_x = 0
      self.pose_vel_left.twist_angular_x = 0
      self.vel_x_left = 0

    if keys.code == 97 or keys.code == 100:
      self.pose_vel_left.twist_linear_y = 0
      self.pose_vel_left.twist_angular_y = 0
      self.vel_y_left = 0

    if keys.code == 113 or keys.code == 101:
      self.pose_vel_left.twist_linear_z = 0
      self.pose_vel_left.twist_angular_z = 0
      self.vel_z_left = 0

    if keys.code == 105 or keys.code == 107:
      self.pose_vel_right.twist_linear_x = 0
      self.pose_vel_right.twist_angular_x = 0
      self.vel_x_right = 0

    if keys.code == 106 or keys.code == 108:
      self.pose_vel_right.twist_linear_y = 0
      self.pose_vel_right.twist_angular_y = 0
      self.vel_y_right = 0

    if keys.code == 117 or keys.code == 111:
      self.pose_vel_right.twist_linear_z = 0
      self.pose_vel_right.twist_angular_z = 0
      self.vel_z_right = 0

    if self.vel_x_left == 0 and self.vel_y_left == 0 and self.vel_z_left == 0:
      self.left_stop = True

    if self.vel_x_right == 0 and self.vel_y_right == 0 and self.vel_z_right == 0:
      self.right_stop = True

    if self.vel_x_base == 0 and self.vel_z_base == 0:
      self.base_stop = True

    if keys.code == 114 or keys.code == 102:
      self.finger_left = 0
    if keys.code == 121 or keys.code == 104:
      self.finger_right = 0

    if self.vel_joint1_left == 0 and self.vel_joint2_left == 0 and self.vel_joint3_left == 0 and self.vel_joint4_left == 0 and self.vel_joint5_left == 0 and self.vel_joint6_left == 0:
      self.left_joint_stop = True
    if self.vel_joint1_right == 0 and self.vel_joint2_right == 0 and self.vel_joint3_right == 0 and self.vel_joint4_right == 0 and self.vel_joint5_right == 0 and self.vel_joint6_right == 0:
      self.right_joint_stop = True

  def pubMsg(self, pose_vel_left, pose_vel_right, joint_vel_left, joint_vel_right):

    rospy.loginfo("arm..")
    rate = rospy.Rate(100)
    
    while self.left_stop == False or self.right_stop == False or self.left_joint_stop == False or self.right_joint_stop == False:
      if self.allstart == 1:
        if not self.left_stop:
          self._pub_left_pose_velocity.publish(pose_vel_left)
        if not self.right_stop:
          self._pub_right_pose_velocity.publish(pose_vel_right)
        if not self.left_joint_stop:
          self._pub_left_joint_velocity.publish(joint_vel_left)
        if not self.right_joint_stop:
          self._pub_right_joint_velocity.publish(joint_vel_right)
      rate.sleep()

  def fingerControl(self, position, client):
    finger_goal = SetFingersPositionGoal()
    finger_goal.fingers.finger1 = position
    finger_goal.fingers.finger2 = position
    finger_goal.fingers.finger3 = position
    client.send_goal(finger_goal)

  def pubBaseMsg(self, vel_x, vel_z):
    vel_cmd_msg = Twist()
    vel_cmd_msg.linear.x = vel_x * self.vel_scale_base
    vel_cmd_msg.angular.z = vel_z * self.vel_scale_base
    rate = rospy.Rate(10)
    while self.vel_x_base != 0 or self.vel_z_base != 0:
      self._pub_cmd_vel.publish(vel_cmd_msg)
      rate.sleep()
      
  def pubThread(self, pose_vel_left, pose_vel_right, joint_vel_left, joint_vel_right):
    t1 = threading.Thread(target=self.pubMsg,args=(pose_vel_left, pose_vel_right, joint_vel_left, joint_vel_right))
    t1.start()

  def pubFingerThread(self, finger_left, finger_right):
    fingers = threading.Thread(target=self.pubFingerMsg,args=(finger_left, finger_right))
    fingers.start()

  def pubBaseThread(self, vel_x, vel_z):
    base = threading.Thread(target=self.pubBaseMsg,args=(vel_x, vel_z))
    base.start()

  def leftFingerPositionInit(self, finger_positions_msg):
    self.finger_positions_msg_left.goal.fingers.finger1 = finger_positions_msg.finger1
    self.finger_positions_msg_left.goal.fingers.finger2 = finger_positions_msg.finger2

  def rightFingerPositionInit(self, finger_positions_msg):
    self.finger_positions_msg_right.goal.fingers.finger1 = finger_positions_msg.finger1
    self.finger_positions_msg_right.goal.fingers.finger2 = finger_positions_msg.finger2

def main():
  rospy.init_node('brain_control_interface')
  BCI = BrainControlInterface()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
