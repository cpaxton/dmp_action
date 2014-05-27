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

'''
verbosity
Controls the level of debug output.
'''
verbosity = 2

'''
dims
Number of dimensions to expect.
Used throughout this file.
'''
dims = 7

'''
makeSetActiveRequest()
Set a DMP as active for planning.
Based off of the DMP ROS package example.
'''
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

'''
makePlanRequest()
Generate a plan from a DMP.
Based off of the DMP ROS package example.
'''
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    if verbosity > 0:
        print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    if verbosity > 0:
        print "DMP planning done"   
            
    return resp

'''
PoseToVector()
A simple helper function.
Convert start/end poses to vectors
'''
def PoseToVector(pose) :
    vec = [0]*dims
    vec[0] = pose.position.x
    vec[1] = pose.position.y
    vec[2] = pose.position.z
    vec[3] = pose.orientation.x
    vec[4] = pose.orientation.y
    vec[5] = pose.orientation.z
    vec[6] = pose.orientation.w

    return vec

'''
Class defining DMP action server.
You can call the "load_file" service to pass a file containing a particular DMP to this server.
After a file has been loaded, the action server may be called any number of times.
'''
class RequestActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = dmp_action.msg.RequestMotionFeedback()
    _result   = dmp_action.msg.RequestMotionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, dmp_action.msg.RequestMotionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._publisher = rospy.Publisher('/output_trajectory/', cartesian_trajectory_msgs.msg.CartesianTrajectory)
        self._file_loaded = False

    def load_file_cb(self, req) :

        f = open(req.filename)
        # use safe_load instead load
        self._dmp = yaml.load(f)
        if verbosity > 1:
            print "Loaded new DMP object:"
            print self._dmp
        f.close()

        self._file_loaded = True

        return LoadFileResponse(1)

    '''
    start_load_service()
    Starts a service to load a YAML file containing DMP.
    '''
    def start_load_service(self) :
        return rospy.Service(self._action_name + '/load_file', LoadFile, self.load_file_cb)
   

    '''
    execute_cb()
    Main execute callback for replaying trajectories.
    Takes a goal with a start and end geometry_msgs/Pose object.
    '''
    def execute_cb(self, goal):

        if self._file_loaded :

            # helper variables
            r = rospy.Rate(1)
            success = True
    
            #Set it as the active DMP
            makeSetActiveRequest(self._dmp.dmp_list)

            #Now, generate a plan
            x_0 = PoseToVector(goal.start)
            x_dot_0 = [0.0]*dims
            t_0 = 0                
            goal = PoseToVector(goal.end)
            goal_thresh = [0.01]*dims
            seg_length = -1          #Plan until convergence to goal
            tau = self._dmp.tau # same time as the demo
            dt = 0.1
            integrate_iter = 5       #dt is rather large, so this is > 1  

            print dt
            print x_0
            print goal

            plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

            traj = cartesian_trajectory_msgs.msg.CartesianTrajectory()
    

            for i in range(len(plan.plan.points)) :
                pred_pt = plan.plan.points[i].positions

                if verbosity > 1:
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

                pt.time_from_start = rospy.Duration(plan.plan.times[i])
                pt.poses = [pose]
                traj.points += [pt]
            

            # publisher:
            # sends out cartesian trajectory message to the right topic
            self._publisher.publish(traj)
      
            rospy.loginfo('%s: Succeeded' % self._action_name)
            result = RequestMotionResult()
            result.status = 1
            self._as.set_succeeded(result)

        else :
            rospy.logerr('%s: Failed! No DMP loaded.' % self._action_name)
            result = RequestMotionResult()
            result.status = 0
            self._as.set_succeeded(result)
      
if __name__ == '__main__':
    try:
        rospy.init_node('dmp_action_server')

        req = RequestActionServer(rospy.get_name())
        s = req.start_load_service()

        rospy.spin()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
