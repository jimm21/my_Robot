#! /usr/bin/env python
import sys
from math import pi
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('trajectory_control', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("robot_arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


current_pose = group.get_current_pose().pose
print(current_pose)

pose_goal = geometry_msgs.msg.Pose()
# Pose Position
x_coordinate = 0.288622765468
y_coordinate = -0.199959113949
z_coordinate = 0.0743749161064
pose_goal.position.x = x_coordinate
pose_goal.position.y = y_coordinate
pose_goal.position.z = z_coordinate

# Pose Orientation
roll_angle = 1.59
pitch_angle = -pi/2
yaw_angle = pi
quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle, axes='szyz')
print(quaternion)

#pose_goal.orientation.w = quaternion[0]
#pose_goal.orientation.x = quaternion[1]
#pose_goal.orientation.y = quaternion[2]
#pose_goal.orientation.z = quaternion[3]

pose_goal.orientation.w = 0.481755087181
pose_goal.orientation.x = -0.516807041881
pose_goal.orientation.y = 0.482195311162
pose_goal.orientation.z = -0.51798667872

print(pose_goal)
group.set_pose_target(pose_goal)

plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
