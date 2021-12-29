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
    def __init__(self):
        super(SubscriberThread, self).__init__()
        self.sub = None
        self.status = "stop"
        self.heading = 0.
        self.runtime = rospy.Duration.from_sec(0)

    # return current status when function called
    def get_status(self):
        return self.status

    # return current status when function called
    def get_heading(self):
        return self.heading

    # return current status when function called
    def get_runtime(self):
        return self.runtime.to_sec() # convert to seconds format before passing

    # ROSbot callback function for updating current status
    def callback_setup(self, msg, args):
        print("Setup Status: {}".format(msg.data))
        self.status = msg.data

    # ROSbot callback function for updating heading (from braitenberg forward proxy pair)
    def callback_heading(self, msg, args):
        self.heading = msg.data

    # ROSbot callback function for updating elapsed runtime
    def callback_runtime(self, msg, args):
        self.runtime = msg.data
        print( "[INFO]: Run Duration: {}".format( self.runtime.to_sec() ) )

    def stop(self):
        print("SubscriberThread stopping...")
        self.sub.unregister() # unregister to subscription eleganty (must have access to sub i.e. see init self.sub)
        self.join()
        rospy.signal_shutdown(None)

    def run(self):
        # create subscriber node with callback function
        self.sub = rospy.Subscriber('/cmd_setup', String, self.callback_setup, ())
        self.sub = rospy.Subscriber('/explore', Float64, self.callback_heading, ())
        self.sub = rospy.Subscriber('/elapsed_time', Duration, self.callback_runtime, ())
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
    sub_thread = SubscriberThread()
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
        
        """
        TO DO:

        Need to calcuate dDistance
        
        """
        
        while(1):
            # check teleop_twist_ROSbot current command
            curr_status = sub_thread.get_status()

            if curr_status == "start":
                heading = sub_thread.get_heading()
                pub_thread.update(1,0,0,heading, speed, turn)

            elif curr_status == "stop":
                # set all values to zero, stop rosbot
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
        sub_thread.stop()