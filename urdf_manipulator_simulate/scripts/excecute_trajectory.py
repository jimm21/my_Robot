#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('trajectory_control', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("robot_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.384649216285
pose_target.position.y = -0.00062573300591
pose_target.position.z = 0.460302341874
group.set_pose_target(pose_target)

plan1 = group.plan()
rospy.sleep(5)
moveit_commander.roscpp_shutdown()

#/hand_ee_controller/command
#/robot_arm_controller/command

