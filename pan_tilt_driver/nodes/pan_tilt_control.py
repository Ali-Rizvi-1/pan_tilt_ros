#!/usr/bin/env python
import math
import rospy

# pan tilt variables
from sensor_msgs.msg import Joy
from pan_tilt_msgs.msg import PanTiltCmdDeg

delta_value = 0.1
pan_tilt_yaw = 0.0
pan_tilt_pitch = 0.0

publisher = 0

# zoom variables

zoom_level = 0

def joy_callback(data):
# pan tilt control
  global publisher
  global pan_tilt_yaw, pan_tilt_pitch

  # left Dpad
  if data.axes[10]==1 and data.buttons[4]!=1:
    pan_tilt_pitch -= delta_value
    # rospy.loginfo("up")

  # down Dpad
  if data.axes[9]==-1 and data.buttons[4]!=1:
    pan_tilt_yaw -= delta_value
    # rospy.loginfo("right")

  # right Dpad
  if data.axes[10]==-1 and data.buttons[4]!=1:
    pan_tilt_pitch += delta_value
    # rospy.loginfo("down")

  # up Dpad
  if data.axes[9]==1 and data.buttons[4]!=1:
    pan_tilt_yaw += delta_value
    # rospy.loginfo("left")

  # horz_axis = data.axes[2]
  # vert_axis = data.axes[5]

  # if horz_axis != 0 and vert_axis != 0:
  #   pan_tilt_pitch = -vert_axis*60
  #   pan_tilt_yaw   =  horz_axis*60

  # button RB
  if data.buttons[5]==1:
    pan_tilt_pitch = 0
    pan_tilt_yaw = 0
    # rospy.loginfo("reset")

  if(pan_tilt_pitch > 60):
    pan_tilt_pitch = 60
  if(pan_tilt_pitch < -60):
    pan_tilt_pitch = -60

  if(pan_tilt_yaw > 60):
    pan_tilt_yaw = 60
  if(pan_tilt_yaw < -60):
    pan_tilt_yaw = -60

  command = PanTiltCmdDeg()
  command.speed = 20.0
  command.yaw = pan_tilt_yaw
  command.pitch = pan_tilt_pitch

  publisher.publish(command)

  # # Zoom camera control
  # global publisher
  # global pan_tilt_yaw, pan_tilt_pitch
  #
  # # left Dpad and L1
  # if data.axes[10]==1 and data.buttons[5]==1:
  #   pan_tilt_pitch -= delta_value
  #   # rospy.loginfo("up")
  #
  # # down Dpad and L1
  # if data.buttons[11]==-1 and data.buttons[5]==1:
  #   pan_tilt_yaw -= delta_value
  #   # rospy.loginfo("right")
  #
  # # right Dpad and L1
  # if data.buttons[10]==-1 and data.buttons[5]==1:
  #   pan_tilt_pitch += delta_value
  #   # rospy.loginfo("down")
  #
  # # up Dpad and L1
  # if data.buttons[11]==1 and data.buttons[5]==1:
  #   pan_tilt_yaw += delta_value
  #   # rospy.loginfo("left")

def main():
  global publisher
  rospy.init_node("pan_tilt_control_node")
  rospy.Subscriber("joy", Joy, joy_callback)
  publisher = rospy.Publisher('pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=10)

  rospy.loginfo("PanTilt Control Start")
  rospy.spin()

if __name__ == '__main__':
  main()
