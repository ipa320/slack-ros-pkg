#!/usr/bin/env python

#    Copyright (C) 2016  Kriegel Joffrey
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
# Import required Python code.
import time
import rospy
from std_msgs.msg import String

import requests

from slackclient import SlackClient

# Node example class.
class SlackROS():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        self.token = rospy.get_param('~token', 'xoxp-123456789')
        self.channel = rospy.get_param('~channel', 'G1234567Q')
        self.username = rospy.get_param('~username', 'ros-bot')
	
        # Create a publisher for our custom message.
        self.pub = rospy.Publisher('from_slack_to_ros', String, queue_size=10)
        rospy.Subscriber("from_ros_to_slack", String, self.message_callback)
        rospy.Subscriber("send_file_to_slack", String, self.file_callback)

        # Create the slack client
        self.slack_client = SlackClient(self.token)
        self.has_connection = False
        self.subscribed_msg_store = []
        self.make_connection()

    def make_connection(self):
        try:
            self.has_connection = self.slack_client.rtm_connect()
            if self.has_connection:
                rospy.loginfo("Slack connection stablished !")
            else:
                rospy.loginfo("Trying to establish slack connection ...")
        except Exception as e:
            rospy.logwarn("Error in establishing connection : %s", e)

    def publish_to_ros(self):
        # Main while loop.
        while not rospy.is_shutdown():
            for reply in self.slack_client.rtm_read():
                if "type" in reply and "user" in reply:
                    if reply["type"] == "message" and reply["channel"] == self.channel:
                        self.pub.publish(reply["text"])
            # Sleep for a while before publishing new messages. Division is so rate != period.
            rospy.sleep(2.0)

    def message_callback(self, data):
        try:
            self.msg_store.append(data)
            if self.has_connection:
                if len(self.subscribed_msg_store):
                    for data in self.subscribed_msg_store:
                        self.slack_client.api_call(
                        "chat.postMessage", channel=self.channel, text=data.data,
                        username=self.username, icon_emoji=':robot_face:')
                        #rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.data, self.channel)
                    self.subscribed_msg_store[:] = []
            else:
                rospy.sleep(2.0)
                self.make_connection()
        except Exception as e:
            rospy.logwarn("Error sending message, connection lost to server : %s", e)
            rospy.sleep(2.0)
            self.make_connection()

    def file_callback(self, data):
        try:
            with open(data.data, 'rb') as file:
                r = requests.post('https://slack.com/api/files.upload', files={'file': ['File '+data.data, file]}, params={
                'token': self.token, 'channels': self.channel})
        except Exception as e:
            rospy.logwarn("Error sending file, connection lost to server : %s", e)
            rospy.sleep(2.0)
            self.make_connection()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('slack_ros')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rospy.loginfo("ROS-Slack bot is running ...")
        slack_ros = SlackROS()
        slack_ros.publish_to_ros()
    except rospy.ROSInterruptException: pass
