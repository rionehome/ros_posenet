#!/usr/bin/env python
# coding: UTF-8
import rospkg
from flask import Flask, send_from_directory
import rospy

app = Flask(__name__)

HOST = "127.0.0.1"


@app.route('/<path:path>')
def media(path):
    __model_path__ = rospkg.RosPack().get_path('ros_posenet') + "/etc/models"
    print __model_path__
    return send_from_directory(__model_path__, path)


if __name__ == '__main__':
    rospy.init_node("model_server")
    app.run(debug=False, host=HOST, port=8000)