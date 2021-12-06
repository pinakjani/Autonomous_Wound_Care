#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus#Array

Target_goal = [[-9.1,-1.2,1.0],[10.7,10.5,1.0],[12.6, -1.6,1.0],[18.2, -1.4,1.0],[-2.0, 4.0,1.0]]
count = 1
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

def done_cb(status,result):
    global Target_goal
    global count
    #rospy.loginfo(status)
    if status == 3:
	count+=1
	if count<len(Target_goal):
	    goal = MoveBaseGoal()
    	    goal.target_pose.header.frame_id = "map"
    	    goal.target_pose.header.stamp = rospy.Time.now() 
   	    goal.target_pose.pose.position.x = Target_goal[count][0]
    	    goal.target_pose.pose.position.y = Target_goal[count][1]
    	    goal.target_pose.pose.orientation.w = Target_goal[count][2]
    	    client.send_goal(goal,done_cb,active_cb,feedback_cb)	

def active_cb():
    return

def feedback_cb(feedback):
    return

def control_loop():
    global Target_goal
    global client
    global status
    global Target_goal
    global count
    rospy.init_node('ebot_controller')
    rate = rospy.Rate(10)
    wait = client.wait_for_server(rospy.Duration(5.0))
    if not wait:
        return
    #rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now() 
    goal.target_pose.pose.position.x = Target_goal[count][0]
    goal.target_pose.pose.position.y = Target_goal[count][1]
    goal.target_pose.pose.orientation.w = Target_goal[count][2]
    client.send_goal(goal,done_cb,active_cb,feedback_cb)
    rospy.spin()
     

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

