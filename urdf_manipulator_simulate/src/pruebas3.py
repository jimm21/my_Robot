#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('trajectory_control', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("robot_arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


wpose = group.get_current_pose().pose
print(wpose)

# We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()
joint_goal[0] = 1.5
joint_goal[1] = 0

group.go(joint_goal, wait=True)
group.stop()

wpose2 = group.get_current_pose().pose
print(wpose2)
joint_goal[0] = 0
joint_goal[1] = 0

group.go(joint_goal, wait=True)
group.stop()

group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
