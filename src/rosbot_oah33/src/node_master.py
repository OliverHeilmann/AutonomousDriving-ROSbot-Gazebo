#!/usr/bin/env python

"""
------- DESCRIPTION --------

--------- USAGE ------------

----- CONTACT DETAILS ------

"""

from __future__ import print_function
from thread_controller import PublishThread

import threading

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Duration

import sys, select, termios, tty

intro = """
node_master.py is the main controller file for autonomous navigation. 
-> See rqt_graph for more details on subscribers/ publishers
-> See code for details on thread script usage
---------------------------
"""

# Subscribe to teleop to check for setup messages
class SubscriberThread(threading.Thread):
    def __init__(self, pub_thread):
        super(SubscriberThread, self).__init__()
        self.pub_thread = pub_thread
        self.sub = None
        self.status = "stop"
        self.heading = 0.
        self.runtime = rospy.Duration.from_sec(0)
        self.total_dist = 0

    # return current status, heading and time when function called
    def get_data(self):
        return self.status, self.heading, self.runtime.to_sec # convert to seconds format before passing

    # ROSbot callback function for updating current status
    def callback_setup(self, msg, args):
        print("[INFO]: Setup Status: {}".format(msg.data))

        # stop rosbot from moving if stop or reset cmd
        if msg.data != "start":
            speed = rospy.get_param("~speed", 0.5)
            turn = rospy.get_param("~turn", 1.0)
            self.pub_thread.update(0,0,0,0, speed, turn)
        
        # reset should not change the rosbot plans
        if msg.data != "reset":
            self.status = msg.data

    # ROSbot callback function for updating heading (from braitenberg forward proxy pair)
    def callback_heading(self, msg, args):
        self.heading = msg.data

    # ROSbot callback function for updating elapsed runtime
    def callback_runtime(self, msg, args):
        self.runtime = msg.data
        print( "[INFO]: Total Run Time: {}".format( self.runtime.to_sec() ) )

    # ROSbot callback function for updating total distance
    def callback_total_dist(self, msg, args):
        self.total_dist = msg.data
        print( "[INFO]: Total Distance: {}".format( self.total_dist ) )
    
    def stop(self):
        print("SubscriberThread stopping...")
        self.sub.unregister() # unregister to subscription eleganty (must have access to sub i.e. see init self.sub)
        self.join()
        rospy.signal_shutdown(None)

    def run(self):
        # create subscriber node with callback function
        self.sub = rospy.Subscriber('/cmd_setup', String, self.callback_setup, ())
        self.sub = rospy.Subscriber('/heading/bberg', Float64, self.callback_heading, ())
        self.sub = rospy.Subscriber('/results/elapsed_time', Duration, self.callback_runtime, ())
        self.sub = rospy.Subscriber('/results/total_dist', Float64, self.callback_total_dist, ())
        rospy.spin()


# Print velocity of ROSbot
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# Main code, will run first
if __name__=="__main__":
    rospy.init_node('node_master')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    # Setup threads for publishing and subscribing
    pub_thread = PublishThread(repeat)
    sub_thread = SubscriberThread(pub_thread)
    sub_thread.start()

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(intro)
        print(vels(speed,turn))
        
        while(1):
            # check teleop_twist_ROSbot current command
            curr_status, curr_heading, curr_runtime = sub_thread.get_data()

            if curr_status != "stop":
                pub_thread.update(1,0,0,curr_heading, speed, turn)

            rospy.sleep(0.05)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        sub_thread.stop()