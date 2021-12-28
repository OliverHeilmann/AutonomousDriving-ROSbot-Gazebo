#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
ROSbot_controller.py Publishing to Twist!

This is the main controller file for autonomous navigation
---------------------------
"""

# Publish movement commands for ROSbot
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    # publish new ROSbot state
    def setup(self, state):
        # Publish.
        self.setupPub.publish(state)

    def stop(self):
        print("PublishThread stopping...")
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


# Subscribe to teleop to check for setup messages
class SettingsThread(threading.Thread):
    def __init__(self):
        super(SettingsThread, self).__init__()
        self.status = "stop"
        self.done = False

    # return current status when function called
    def get_status(self):
        return self.status

    # ROSbot callback function for updating current status
    def moveBot_callback(self, msg, args):
        print(msg.data)
        self.status = msg.data

    def stop(self):
        print("SettingsThread stopping...")
        self.done = True
        self.join()
        rospy.signal_shutdown(None)

    def run(self):
        # create subscriber node with callback function
        sub = rospy.Subscriber('cmd_setup', String, self.moveBot_callback, (pub_thread))
        rospy.spin()


# Print velocity of ROSbot
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# Main code, will run first
if __name__=="__main__":
    rospy.init_node('ROSbot_controller')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    # Setup threads for publishing and subscribing
    pub_thread = PublishThread(repeat)
    set_thread = SettingsThread()
    set_thread.start()

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        
        while(1):
            # check teleop_twist_ROSbot current command
            curr_status = set_thread.get_status()

            if curr_status == "start":
                # let other nodes control robot actions
                pass

            elif curr_status == "stop":
                # set all values to zero, stop rosbot (NOTE: INCLUDE STOP CLOCK)
                pub_thread.update(0,0,0,0, speed, turn)

            elif curr_status == "reset":
                # placeholder for resetting values e.g. clock and distance travelled
                pass

            else:
                # not expected, but if unexpected result, set all values to zero...
                pub_thread.update(0,0,0,0, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        set_thread.stop()