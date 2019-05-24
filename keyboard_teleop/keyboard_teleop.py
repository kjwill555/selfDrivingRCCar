#!/usr/bin/env python

""" Teleop node for controlling the car using the WASD keys """

from __future__ import print_function

import roslib; roslib.load_manifest('keyboard_teleop')
import rospy

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

# Map (x, y, z, th) values to each key
moveBindings = {
        'w':(0,1,0,0),
        's':(0,-1,0,0),
        'a':(0,0,0,1),
        'd':(0,0,0,-1),
}


"""
Reads a single key pressed by the user.

Returns:
str: The key that was pressed
"""
def getKey():
    # Open stdin in raw mode
    tty.setraw(sys.stdin.fileno())
    # Block until a key is pressed
    select.select([sys.stdin], [], [], 0)
    # Read the key
    key = sys.stdin.read(1)
    # Drain stdin
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    # Save the current stdin settings for exit time
    settings = termios.tcgetattr(sys.stdin)

    # Publish on the twist topic
    pub = rospy.Publisher('twist', Twist, queue_size = 1)
    rospy.init_node('keyboard_teleop')

    # Use y for throttle, th for steering
    y = 0
    th = 0

    try:
        print("Ready")
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                y = moveBindings[key][1]
                th = moveBindings[key][3]
            elif key == '\x03':
                # Terminate
                # Stop will be sent in finally block
                break
            else:
                # Stop
                y = 0
                th = 0

            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = y
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = th
            twist.angular.z = 0
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Stop the car
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        # Restore the settings for stdin
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
