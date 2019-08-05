#!/usr/bin/env python
# coding: UTF-8
import rospkg
import sys

from flask import Flask, send_from_directory
import rospy
import signal

app = Flask(__name__)
HOST = "127.0.0.1"


def killer(signum, frame):
    print signum, frame
    sys.exit(1)


@app.route('/<path:path>')
def media(path):
    __model_path__ = rospkg.RosPack().get_path('ros_posenet') + "/etc/models"
    print "get:", path
    return send_from_directory(__model_path__, path)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, killer)
    rospy.init_node("model_server")
    app.run(debug=False, host=HOST, port=8000)
