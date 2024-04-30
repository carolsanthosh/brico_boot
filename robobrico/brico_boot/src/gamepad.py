#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Point, Twist
import struct
import sys

linear_vel = 0.5
angular_vel = 0.5
turbo = 0.0

rospy.init_node("gamepad")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(45)

infile_path = "/dev/input/event2"
FORMAT = 'llHHI'
EVENT_SIZE = struct.calcsize(FORMAT)

# Open the file in binary mode
in_file = open(infile_path, "rb")

# Read the first event
event = in_file.read(EVENT_SIZE)


def map(num, inMin, inMax, outMin, outMax):
    return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))


# map(5,0,10,0,100)
while event:
    (tv_sec, tv_usec, type, code, value) = struct.unpack(FORMAT, event)
    if type != 0 or code != 0 or value != 0:
        print(f"Event type {type}, code {code}, value {value} at {tv_sec}.{tv_usec}")

        if (code == 17):
            if (value == 1):
                speed.linear.x = (linear_vel+turbo)*-1
            elif (value > 1):
                speed.linear.x = linear_vel+turbo
            else:
                speed.linear.x = 0.0

        if (code == 16):
            if (value == 1):
                speed.angular.z = (angular_vel + turbo)*-1
            elif (value > 1):
                speed.angular.z = angular_vel + turbo
            else:
                speed.angular.z = 0.0

        if (code == 309):
            if (value == 1):
                turbo = -0.2
            else:
                turbo = 0.0
        if (code == 311):
            if (value == 1):
                turbo = 0.2
            else:
                turbo = 0.0

    else:
        # Events with code, type, and value == 0 are "separator" events
        print("===========================================")
    pub.publish(speed)
    event = in_file.read(EVENT_SIZE)
    r.sleep()

in_file.close()
