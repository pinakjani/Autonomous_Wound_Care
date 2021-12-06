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
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
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

        list_joint_values = selfpython._group.get_current_joint_values()
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

class Detector:

    def __init__(self):

      self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.image_cb, queue_size=1, buff_size=2**24)

    def image_cb(self, data):
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
        print(e)
      
      print("Saving Image")
      cv2.imwrite('/home/kartik/catkin_ws/src/ug_project/autodoc/scripts/woundimage.png',cv_image)
      print("Done Saving")
      print("Running Bash")
      subprocess.call('~/catkin_ws/src/ug_project/autodoc/scripts/tf.sh',shell = True)
      return cv_image

    def __del__(self):
        rospy.loginfo(
            '\033[94m' + "Image Detector Deleted." + '\033[0m')
def main():

    ur5 = Ur5Moveit()

    #ur5.go_to_predefined_pose("straightUp")
   
    ur5.go_to_predefined_pose("view_wound")
    #object_1 = geometry_msgs.msg.Pose()
    #object_1.position.x = 2.59002
    #object_1.position.y = 0.436484
    #object_1.position.z = 1.771862
    #quaternion_1 = quaternion_from_euler(0.003907, 0.007019, -0.001113)
    #object_1.orientation.x = quaternion_1[0]
    #object_1.orientation.y = quaternion_1[1]
    #object_1.orientation.z = quaternion_1[2]
    #object_1.orientation.w = quaternion_1[3]

    #ur5.go_to_pose(object_1)

    del ur5
    img=Detector()

    del img
    try:
      rospy.sleep(10)
      
    except KeyboardInterrupt:
      print("ShutDown")
    cv2.destroyAllWindows()
    
    
if __name__ == '__main__':
    main()

