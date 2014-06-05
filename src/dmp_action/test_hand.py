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
lf('sample_data/Hand_Motion_1.yaml')

client = actionlib.SimpleActionClient('dmp_action_server',
        dmp_action.msg.RequestMotionAction)

client.wait_for_server()

goal = dmp_action.msg.RequestMotionGoal()
goal.start.position.x = 0.4
goal.start.position.y =  0.312
goal.start.position.z = 0.0
goal.start.orientation.x = 0.753
goal.start.orientation.y = 0.658
goal.start.orientation.z = -0.018
goal.start.orientation.w = -0.021

goal.end.position.x = -0.4
goal.end.position.y =  0.362
goal.end.position.z = 0.1
goal.end.orientation.x = 0.753
goal.end.orientation.y = 0.658
goal.end.orientation.z = -0.018
goal.end.orientation.w = -0.021

client.send_goal(goal)

client.wait_for_result()

print client.get_result()
