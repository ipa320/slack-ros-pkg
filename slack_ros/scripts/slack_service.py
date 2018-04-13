#!/usr/bin/env python

import time
import sys
import os
import threading
import signal
import zmq

import rospy
from std_msgs.msg import String
from slack_interpreter.utils import unescape
from slack_interpreter.modules_system import *

import requests

from slackclient import SlackClient

SIGINT = False
ROSCORE_UP = False

def signal_handler(signal, frame):
    global SIGINT
    SIGINT = True

def is_roscore_up():
    return ROSCORE_UP

def set_roscore_up(value):
    global ROSCORE_UP
    ROSCORE_UP = value

class SlackService(object):
    def __init__(self, token, channel, username):
        self.token = token
        self.channel = channel
        self.username = username

        self.client = SlackClient(self.token)
        self.callbacks = []
        self.slack_thread = None

        self.context = zmq.Context()
        self.socket_pub = self.context.socket(zmq.PUB)
        self.socket_sub = self.context.socket(zmq.SUB)

        print "bind to socket"
        self.socket_pub.bind('tcp://127.0.0.1:2000')
        print "connect to socket"
        self.socket_sub.connect('tcp://127.0.0.1:2001')
        print "done"

        self.socket_sub.setsockopt(zmq.SUBSCRIBE, '')
        
        self.zmq_thread = threading.Thread(target=self.read_from_zmq)

        if self.client.rtm_connect():
            self.slack_thread = threading.Thread(target=self.read_from_slack)
            self.slack_thread.start()
            self.zmq_thread.start()

    def read_from_slack(self):
        while not SIGINT:
            for reply in self.client.rtm_read():
                if "type" in reply and "user" in reply:
                    if reply["type"] == "message" and reply["channel"] == self.channel:
                        for call in self.callbacks:
                            try:
                                call(reply["text"])
                            except Exception as e:
                                print "Exception calling callback: %s"%e.message
            time.sleep(0.2)

    def read_from_zmq(self):
        while not SIGINT:
            try:
                message = self.socket_sub.recv_string(zmq.NOBLOCK)
                self.send(message)
            except:
                time.sleep(0.1)

    def register_callback(self, fkt):
        if callable(fkt):
            self.callbacks.append(fkt)
        else:
            sys.stderr("register_callback: arg is not callable")

    def unregister_callback(self, fkt):
        self.callbacks.remove(fkt)

    def send_slack(self, data):
        self.client.api_call(
            "chat.postMessage", channel=self.channel, text=data,
            username=self.username, icon_emoji=':robot_face:'
        )

    def send_slack_file(self, data):
        with open(data.data, 'rb') as file:
            requests.post('https://slack.com/api/files.upload', 
                files={'file': ['File '+data.data, file]}, params={
                'token': self.token,
                'channels': self.channel
            })

    def send_zmq(self, data):
        self.socket_pub.send_string(data)

class SystemInterpreter(object):
    def __init__(self):
        self.system = System()
        self.bringup = Bringup()

    def slack_cb(self, msg):
        cmd = "self."+msg[2:]
        try:
            return eval(cmd)
        except Exception as e:
            print "SystemInterpreter: error eval '%s': %s"%(cmd, e.message)
            raise e

class InterpreterMediator(object):
    def __init__(self, slack_srv):
        self.sys_int = SystemInterpreter()
        self.slack_srv = slack_srv
        self.slack_srv.register_callback(self.slack_callback)

    def __del__(self):
        self.slack_srv.register_callback(self.slack_callback)

    def slack_callback(self, data):
        #first check if it is a command
        msg = unescape(data)

        #it is a interpreter command
        if msg.startswith('> '):
            #first pass it to system interpreter
            try:
                self.slack_srv.send_slack(self.sys_int.slack_cb(msg))
            except Exception as e:
                #if system interpreter issues an error
                #check if ros is running and send it to ros interpreter
                #since we do not know if ros interpreter was successfull at 
                #this stage, we just forward the ros interpreter output
                if is_roscore_up():
                    self.slack_srv.send_zmq(data)
                #if ros is not running
                else:
                    self.slack_srv.send_slack("Error exec command. Roscore not running and system reported error: %s"%e.message)
        #no interpreter command just send it to zmq socket
        else:
            self.slack_srv.send_zmq(data)

if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.stderr.write("Wrong num command line arguments!")
        sys.exit(os.EX_CONFIG)

    signal.signal(signal.SIGINT, signal_handler)

    slack_srv = SlackService(sys.argv[1], sys.argv[2], sys.argv[3])
    mediator = InterpreterMediator(slack_srv)
    
    while not SIGINT:

        try:
            rospy.get_master().getPid()
            set_roscore_up(True)
        except Exception as e:
            set_roscore_up(False)
        rospy.sleep(0.5)
