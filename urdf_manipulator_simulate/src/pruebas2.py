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

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation = wpose.orientation
pose_goal.position.x = wpose.position.x
pose_goal.position.y = wpose.position.y
pose_goal.position.z = wpose.position.z
group.set_pose_target(pose_goal)
plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
