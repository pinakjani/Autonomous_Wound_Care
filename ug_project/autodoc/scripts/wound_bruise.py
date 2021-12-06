#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import quaternion_from_euler
import subprocess


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('eg4_go_to_pose', anonymous=True)

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
	self._gripper_group = moveit_commander.MoveGroupCommander("gripper_group")
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def move_gripper(self, pose_name):
	rospy.loginfo('\033[94m' + "Gripper going to Pose: {}".format(pose_name) + '\033[0m')
        self._gripper_group.set_named_target(pose_name)
	plan = self._gripper_group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
	self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Gripper now at Pose: {}".format(pose_name) + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
def main():

    ur5 = Ur5Moveit()

# #bruise
    # ur5.go_to_predefined_pose("view_left")
    # object_4 = geometry_msgs.msg.Pose()
    # object_4.position.x = -0.759
    # object_4.position.y = -0.267
    # object_4.position.z = 0.629
    # quaternion_4 = quaternion_from_euler(-1.573,0.04, 1.55)
    # object_4.orientation.x = quaternion_4[0]
    # object_4.orientation.y = quaternion_4[1]
    # object_4.orientation.z = quaternion_4[2]
    # object_4.orientation.w = quaternion_4[3]

    # ur5.go_to_pose(object_4)
    # object_4.position.z = 0.545
    # ur5.go_to_pose(object_4)

    ur5.go_to_predefined_pose("view_left")
    object_4 = geometry_msgs.msg.Pose()
    object_4.position.x = -0.776
    object_4.position.y = -0.244
    object_4.position.z = 0.629
    quaternion_4 = quaternion_from_euler(-1.571,0.0251, 1.55)
    object_4.orientation.x = quaternion_4[0]
    object_4.orientation.y = quaternion_4[1]
    object_4.orientation.z = quaternion_4[2]
    object_4.orientation.w = quaternion_4[3]

    ur5.go_to_pose(object_4)
    object_4.position.z = 0.521
    ur5.go_to_pose(object_4)
    del ur5  
    
    print("Running Bash")
    subprocess.call('~/catkin_ws/src/ug_project/autodoc/scripts/tf.sh',shell = True)  
if __name__ == '__main__':
    main()

