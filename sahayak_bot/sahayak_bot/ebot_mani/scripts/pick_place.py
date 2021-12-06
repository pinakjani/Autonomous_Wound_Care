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

    #ur5.go_to_predefined_pose("straightUp")

    object_1 = geometry_msgs.msg.Pose()
    object_1.position.x = 0.56
    object_1.position.y = -0.008
    object_1.position.z = 1.025
    quaternion_1 = quaternion_from_euler(-1.559, 0.0046, 2.322)
    object_1.orientation.x = quaternion_1[0]
    object_1.orientation.y = quaternion_1[1]
    object_1.orientation.z = quaternion_1[2]
    object_1.orientation.w = quaternion_1[3]

    object_2 = geometry_msgs.msg.Pose()
    object_2.position.x = 0.603
    object_2.position.y = 0.044
    object_2.position.z = 0.75
    quaternion_2 = quaternion_from_euler(3.128, 0.071, -2.476)
    object_2.orientation.x = quaternion_2[0]
    object_2.orientation.y = quaternion_2[1]
    object_2.orientation.z = quaternion_2[2]
    object_2.orientation.w = quaternion_2[3]

    object_3 = geometry_msgs.msg.Pose()
    object_3.position.x = 0.584
    object_3.position.y = -0.26
    object_3.position.z = 1.05
    quaternion_3 = quaternion_from_euler(-1.589, -0.08, -3.14)
    object_3.orientation.x = quaternion_3[0]
    object_3.orientation.y = quaternion_3[1]
    object_3.orientation.z = quaternion_3[2]
    object_3.orientation.w = quaternion_3[3]

    box_1 = geometry_msgs.msg.Pose()
    box_1.position.x = 0.007
    box_1.position.y = 0.7	
    box_1.position.z = 1.2
    box_1.orientation = object_1.orientation

    box_2 = geometry_msgs.msg.Pose()
    box_2.position.x = 0.0
    box_2.position.y = -0.69
    box_2.position.z = 1.35
    box_2.orientation = object_3.orientation

    ur5.go_to_pose(object_1)
    object_1.position.z = 0.93
    ur5.go_to_pose(object_1)
    ur5.move_gripper("closed_med")
    object_1.position.z = 1.1
    ur5.go_to_pose(object_1)
    ur5.go_to_pose(box_1)
    ur5.move_gripper("open")

    ur5.go_to_pose(object_3)
    object_3.position.z = 0.972
    ur5.go_to_pose(object_3)
    ur5.move_gripper("closed_med")
    ur5.go_to_pose(box_2)
    ur5.move_gripper("open")

    ur5.go_to_pose(object_2)
    object_2.position.z = 0.662
    ur5.go_to_pose(object_2)
    ur5.move_gripper("closed_high")
    box_1.position.y = 0.45	
    box_1.orientation = object_2.orientation
    ur5.go_to_pose(box_1)
    ur5.move_gripper("open")


    del ur5


if __name__ == '__main__':
    main()

