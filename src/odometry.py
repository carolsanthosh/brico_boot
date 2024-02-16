#!/usr/bin/env python

import rospy
from math import cos, sin
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class WheelOdometry:
    def __init__(self):
        rospy.init_node('wheel_odometry_node')

        # Parameters
        self.base_width = 0.45
        self.wheel_radius = 0.06775
	self.PI = math.pi

        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timestamp and initial RPM values
        self.last_time = rospy.Time.now()
        self.last_L_rpm = 0.0
        self.last_R_rpm = 0.0

        # ROS publishers and subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
	self.tf_broadcaster_laser = tf.TransformBroadcaster()
        rospy.Subscriber('/left_motor_rpm', Float64, self.left_rpm_callback)
        rospy.Subscriber('/right_motor_rpm', Float64, self.right_rpm_callback)

        rospy.spin()

    def left_rpm_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        L_rpm = msg.data
        R_rpm = self.last_R_rpm  # Assuming the right wheel RPM remains constant between updates

	L_vel = (2*self.PI*L_rpm)/60
	R_vel = (2*self.PI*R_rpm)/60
        # Calculate linear and angular velocities
        linear_velocity = ((self.wheel_radius * (L_vel + R_vel))/2) 
        angular_velocity = ((self.wheel_radius * (R_vel - L_vel))/self.base_width)

        # Update robot pose
        self.x += linear_velocity * dt * cos(self.theta)
        self.y += linear_velocity * dt * sin(self.theta)
        self.theta += angular_velocity * dt

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.theta)))
        odom_msg.twist.twist = Twist(Point(linear_velocity, 0, 0), Point(0, 0, angular_velocity))
        self.odom_pub.publish(odom_msg)

        # Publish tf transform
	"""
        self.tf_broadcaster.sendTransform((self.x, self.y, 0), tf.transformations.quaternion_from_euler(0, 0, self.theta),
                                         current_time, "base_link", "odom")

        self.tf_broadcaster_laser.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                                            current_time, "laser", "base_link")
	"""
	# Update timestamp and RPM values
        self.last_time = current_time
        self.last_L_rpm = L_rpm

    def right_rpm_callback(self, msg):
        self.last_R_rpm = msg.data

if __name__ == '__main__':
    try:
        WheelOdometry()
    except rospy.ROSInterruptException:
        pass

