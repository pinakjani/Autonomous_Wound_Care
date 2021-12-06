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

    #ur5.go_to_predefined_pose("straightUp")
    #ur5.go_to_predefined_pose("arm_abrasion")
    
#cut
    # ur5.go_to_predefined_pose("view_wound")
    # object_1 = geometry_msgs.msg.Pose()
    # object_1.position.x =  0.226
    # object_1.position.y = 0.687
    # object_1.position.z = 0.672
    # quaternion_1 = quaternion_from_euler(-1.512,-0.028, 0.061)
    # object_1.orientation.x = quaternion_1[0]
    # object_1.orientation.y = quaternion_1[1]
    # object_1.orientation.z = quaternion_1[2]
    # object_1.orientation.w = quaternion_1[3]

    # ur5.go_to_pose(object_1)
# #abrasion
    #ur5.go_to_predefined_pose("view_wound")
    # object_2 = geometry_msgs.msg.Pose()
    # object_2.position.x =  -0.282
    # object_2.position.y = 0.655
    # object_2.position.z = 0.658
    # quaternion_2 = quaternion_from_euler(-1.512,-0.028,0.061)
    # object_2.orientation.x = quaternion_2[0]
    # object_2.orientation.y = quaternion_2[1]
    # object_2.orientation.z = quaternion_2[2]
    # object_2.orientation.w = quaternion_2[3]

    # ur5.go_to_pose(object_2)
    # object_2.position.z = 0.558
    # ur5.go_to_pose(object_2)
# #burn
    # object_3 = geometry_msgs.msg.Pose()
    # object_3.position.x =  -0.796
    # object_3.position.y = 0.262
    # object_3.position.z = 0.631
    # quaternion_3 = quaternion_from_euler(-1.601,-0.057,1.681)
    # object_3.orientation.x = quaternion_3[0]
    # object_3.orientation.y = quaternion_3[1]
    # object_3.orientation.z = quaternion_3[2]
    # object_3.orientation.w = quaternion_3[3]

    # ur5.go_to_pose(object_3)
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
#go above cotton
    ur5.go_to_predefined_pose("view_wound")
    object_6 = geometry_msgs.msg.Pose()
    object_6.position.x =  0.567
    object_6.position.y =  0.616
    object_6.position.z = 0.620
    quaternion_6 = quaternion_from_euler(-1.581,-0.028, 0.062)
    object_6.orientation.x = quaternion_6[0]
    object_6.orientation.y = quaternion_6[1]
    object_6.orientation.z = quaternion_6[2]
    object_6.orientation.w = quaternion_6[3]

    ur5.go_to_pose(object_6)
    object_6.position.z = 0.564900
    ur5.go_to_pose(object_6)
    ur5.move_gripper("closed_med")

    ur5.go_to_predefined_pose("view_wound")
    object_1 = geometry_msgs.msg.Pose()
    object_1.position.x =  0.272
    object_1.position.y = 0.691
    object_1.position.z = 0.636
    quaternion_1 = quaternion_from_euler(-1.582,-0.027, 0.060)
    object_1.orientation.x = quaternion_1[0]
    object_1.orientation.y = quaternion_1[1]
    object_1.orientation.z = quaternion_1[2]
    object_1.orientation.w = quaternion_1[3]
#cleaning the cut wound
    ur5.go_to_pose(object_1)
    object_1.position.x =  0.291
    object_1.position.y = 0.553
    ur5.go_to_pose(object_1)
    object_1.position.x =  0.272
    object_1.position.y = 0.691
    ur5.go_to_pose(object_1)
    object_1.position.x =  0.291
    object_1.position.y = 0.553
    ur5.go_to_pose(object_1)
    del ur5  
    
    #print("Running Bash")
    #subprocess.call('~/catkin_ws/src/ug_project/autodoc/scripts/tf.sh',shell = True)  
if __name__ == '__main__':
    main()

