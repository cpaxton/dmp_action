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

#Set a DMP as active for planning
#From active
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def PoseToVector(pose)
  vec = []
  vec[0] = pose.position.x
  vec[1] = pose.position.y
  vec[2] = pose.position.z
  vec[3] = pose.orientation.x
  vec[4] = pose.orientation.y
  vec[5] = pose.orientation.z
  vec[6] = pose.orientation.w

  return vec

'''
Class defining DMP action server settings
'''
class RequestActionServer(object):
  # create messages that are used to publish feedback/result
  _feedback = actionlib_tutorials.msg.RequestMotionFeedback()
  _result   = actionlib_tutorials.msg.RequestMotionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._publisher = rospy.Publisher('/output_trajectory/', cartesian_trajectory_msgs.msg.CartesianTrajectory)
        self._file_loaded = False

    def load_file_cb(self) :
        import yaml
        f = open('tree.yaml')
        # use safe_load instead load
        data = yaml.safe_load(f)
        f.close()
        print data
    
    def execute_cb(self, goal):

        if self._file_loaded :

            # helper variables
            r = rospy.Rate(1)
            success = True
    
            #Set it as the active DMP
            makeSetActiveRequest(resp.dmp_list)

            #Now, generate a plan
            x_0 = PoseToVector(goal.start)
            x_dot_0 = [0.0]*dims
            t_0 = 0                
            goal = PoseToVector(goal.end)
            goal_thresh = [0.01]*dims
            seg_length = -1          #Plan until convergence to goal
            tau = resp.tau # same time as the demo
            dt = 1
            integrate_iter = 5       #dt is rather large, so this is > 1  

            print dt
            print x_0
            print goal

            plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

            traj = cartesian_trajectory_msgs.msg.CartesianTrajectory()
    

            for i in range(len(plan.plan.points)) :
                pred_pt = plan.plan.points[i]

                print pred_pt # predicted point
                pose = geometry_msgs.msg.Pose()
                pt = cartesian_trajectory_msgs.msg.CartesianTrajectoryPoint()
                pose.position.x = pred_pt[0]
                pose.position.y = pred_pt[1]
                pose.position.z = pred_pt[2]
                pose.orientation.x = pred_pt[3]
                pose.orientation.x = pred_pt[4]
                pose.orientation.x = pred_pt[5]
                pose.orientation.x = pred_pt[6]

                pt.time_from_start = plan.plan.times[i]
                pt.poses = [pose]
            

            # publisher:
            # sends out cartesian trajectory message to the right topic

      
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else :
            rospy.logerror('%s: Failed! No DMP loaded.' % self._action_name)
      
if __name__ == '__main__':
    rospy.init_node('dmp_action_server_node')
    s = 

    RequestActionServer(rospy.get_name())
    rospy.spin()

