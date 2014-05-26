#! /usr/bin/env python

import roslib; roslib.load_manifest('dmp_action')
import rospy

import actionlib
import yaml

# import message, service types
from dmp_action.srv import *
from dmp.msg import *
from dmp.srv import *
from cartesian_trajectory_msgs.msg import *

from dmp_action.msg import *

rospy.init_node('test_dmp_server_node')
rospy.wait_for_service('dmp_action_server/load_file')
lf = rospy.ServiceProxy('dmp_action_server/load_file', LoadFile)
lf('data.yaml')


