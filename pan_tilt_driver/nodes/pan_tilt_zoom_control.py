#!/usr/bin/env python

"""
Pan-Tilt Camera Controller for ROS 1 (Noetic)
Controls an IQR pan-tilt platform.

Features:
1. Point tracking - Aims camera at 3D points
2. Direct angle control - Sets pan/tilt angles
3. Transform chain handling
"""

import math
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float64
from pan_tilt_msgs.msg import PanTiltCmdDeg

class PanTiltController:
    def __init__(self):
        """Initialize the controller node with hardware-specific parameters."""
        rospy.init_node('pan_tilt_controller')
        
        # Initialize all systems
        self._init_system_parameters()
        self._init_tf_system()
        self._init_ros_interfaces()
        
        rospy.loginfo('Pan-Tilt controller initialized and ready')

    def _init_system_parameters(self):
        """Initialize hardware-specific parameters."""
        # Hardware limits for IQR platform
        self.PAN_LIMIT = math.radians(60)    # ±60 degrees
        self.TILT_LIMIT = math.radians(60)   # ±60 degrees
        self.MIN_SPEED = 1
        self.MAX_SPEED = 30
        self.DEFAULT_SPEED = 15
        
        # Mechanical offsets (in meters) - Update with actual values
        self.OFFSETS = {
            'yaw_z': 0.023,
            'pitch_z': 0.062,
            'surface_z': 0.032,
            'camera_x': -0.0199,
            'camera_z': 0.04585,
            'optical_x': 0.0726
        }

    def _init_tf_system(self):
        """Initialize the TF2 system for coordinate transformations."""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _init_ros_interfaces(self):
        """Initialize ROS publishers and subscribers."""
        # Publishers
        self.ptz_command_pub = rospy.Publisher(
            '/pan_tilt_cmd_deg', 
            PanTiltCmdDeg, 
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber('target_point', Point, self.point_callback)
        rospy.Subscriber('pan_tilt_angles', Point, self.angle_callback)

        # Commented zoom control for reference
        # To control zoom:
        # rostopic pub /set_level_command std_msgs/Int16 "data: 20"

    def calculate_pan_tilt_angles(self, target_point_msg):
        """Calculate pan/tilt angles from target point."""
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = "camera_optical_frame"
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.point = target_point_msg

            # Transform to base_link
            transform_base = self.tf_buffer.lookup_transform(
                'pan_tilt_base_link',
                "camera_optical_frame",
                rospy.Time(0)
            )
            target_base = tf2_geometry_msgs.do_transform_point(
                point_stamped, 
                transform_base
            )

            # Calculate angles
            pan = math.atan2(target_base.point.y, target_base.point.x)
            xy_distance = math.sqrt(
                target_base.point.x**2 + 
                target_base.point.y**2
            )
            tilt = math.atan2(
                target_base.point.z - self.OFFSETS['yaw_z'], 
                xy_distance
            )

            # Apply joint limits
            pan_deg = math.degrees(np.clip(pan, -self.PAN_LIMIT, self.PAN_LIMIT))
            tilt_deg = math.degrees(np.clip(tilt, -self.TILT_LIMIT, self.TILT_LIMIT))

            rospy.loginfo(
                f'Calculated angles - Pan: {pan_deg:.2f}°, Tilt: {tilt_deg:.2f}°'
            )

            return pan_deg, tilt_deg

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f'Transform error: {str(e)}')
            return None, None

    def send_ptz_command(self, pan_deg, tilt_deg, speed=None):
        """Send command to the PTZ hardware."""
        try:
            cmd = PanTiltCmdDeg()
            cmd.yaw = pan_deg
            cmd.pitch = tilt_deg
            cmd.speed = speed if speed else self.DEFAULT_SPEED
            
            # Validate and clip values
            cmd.yaw = np.clip(cmd.yaw, -60.0, 60.0)
            cmd.pitch = np.clip(cmd.pitch, -60.0, 60.0)
            cmd.speed = np.clip(cmd.speed, self.MIN_SPEED, self.MAX_SPEED)
            
            self.ptz_command_pub.publish(cmd)
            
        except Exception as e:
            rospy.logerr(f'Error sending PTZ command: {str(e)}')

    def point_callback(self, msg):
        """Handle incoming target points."""
        try:
            # Calculate pan/tilt angles
            pan_deg, tilt_deg = self.calculate_pan_tilt_angles(msg)
            
            if pan_deg is not None and tilt_deg is not None:
                # Send command to hardware
                self.send_ptz_command(pan_deg, tilt_deg)
                
        except Exception as e:
            rospy.logerr(f'Error in point callback: {str(e)}')

    def angle_callback(self, msg):
        """Handle direct angle control."""
        try:
            # Convert from radians to degrees for hardware
            pan_deg = math.degrees(msg.x)
            tilt_deg = math.degrees(msg.y)
            self.send_ptz_command(pan_deg, tilt_deg)
            
        except Exception as e:
            rospy.logerr(f'Error in angle callback: {str(e)}')

def main():
    """Main function."""
    try:
        controller = PanTiltController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f'Error in main: {str(e)}')

if __name__ == '__main__':
    main()