#!/usr/bin/env python

import time
import rospy
import zmq
import threading
from std_msgs.msg import String

class SlackROSBridge():
    def __init__(self):
        self.pub = rospy.Publisher('from_slack_to_ros', String, queue_size=10)
        self.sub = rospy.Subscriber("from_ros_to_slack", String, self.callback)
        self.context = zmq.Context()
        self.socket_pub = self.context.socket(zmq.PUB)
        self.socket_sub = self.context.socket(zmq.SUB)

        self.socket_pub.bind('tcp://127.0.0.1:2001')
        self.socket_sub.connect('tcp://127.0.0.1:2000')

        self.socket_sub.setsockopt(zmq.SUBSCRIBE, '')
        
        self.zmq_thread = threading.Thread(target=self.read_from_zmq)
        self.zmq_thread.start()

    def callback(self, data):
        self.socket_pub.send_string(data.data)

    def read_from_zmq(self):
        while not rospy.is_shutdown():
            try:
                message = self.socket_sub.recv_string(flags=zmq.NOBLOCK)
                self.pub.publish(message)
            except:
                rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('slack_ros_bridge')
    bridge = SlackROSBridge()
    rospy.spin()
