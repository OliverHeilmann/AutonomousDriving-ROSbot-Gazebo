#!/usr/bin/env python3
# BEGIN ALL
#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int32


# BEGIN CALLBACK
def callback(msg):
    print(msg.data)
# END CALLBACK


rospy.init_node('topic_subscriber')

# BEGIN SUBSCRIBER
sub = rospy.Subscriber('counter', Int32, callback)
# END SUBSCRIBER

rospy.spin()
# END ALL
