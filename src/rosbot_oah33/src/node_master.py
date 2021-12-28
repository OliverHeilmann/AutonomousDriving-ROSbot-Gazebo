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

import sys, select, termios, tty

intro = """
node_master.py is the main controller file for autonomous navigation. 
-> See rqt_graph for more details on subscribers/ publishers
-> See code for details on thread script usage
---------------------------
"""

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
    rospy.init_node('node_master')

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

        print(intro)
        print(vels(speed,turn))
        
        """
        TO DO:
        
        ONCE BBERG NODE IS DONE AND THETA IS PUBLISHED, USE THIS SCRIPT
        TO SUBSCRIBE TO IT. IN THE CALLBACK THEN UPDATE THE VELOCITY 
        VALUES... 

        When done, then will start to include the bias approaches. Could
        do in the bberg node but keeping it here might be better for when
        other sensors are included e.g. lidar.

        Need to calcuate dTime and dDistance as well (based on trigger)
        
        """
        
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