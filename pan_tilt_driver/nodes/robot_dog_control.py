#!/usr/bin/env python
import math
import rospy

# pan tilt variables
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

publisher = 0
speed_mod = 0.5 # alter to change all movement speed
move = Twist()

def joy_callback(data):
# Move dog with joystick

  global publisher

  move.linear.x = 0
  move.linear.y = 0
  move.angular.z = 0

# Left joystick

  forward_back = data.axes[1]*speed_mod
  side_side = data.axes[0]*speed_mod/2

# Right joystick

  turn = data.axes[2]*speed_mod*2

# Set motion

  move.linear.x = forward_back
  move.linear.y = side_side
  move.angular.z = turn

  publisher.publish(move)

def main():
  global publisher
  rospy.init_node("robot_dog_control_node")
  rospy.Subscriber("joy", Joy, joy_callback)
  publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

  rospy.spin()

if __name__ == '__main__':
  main()
