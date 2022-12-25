#!/usr/bin/env python
import math
import rospy
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from urdf_manipulator_simulate.srv import ik, ikResponse
from urdf_manipulator_simulate.msg import my_msg
from urdf_manipulator_simulate.srv import parameters, parametersResponse
from geometry_msgs.msg import Point

def callback(request):

    q1 = 0.0
    q2 = 0.0
    q3 = 0.0

    L0 = request.L0
    L1 = request.L1
    L2 = request.L2
    L3 = request.L3

    the_msg = my_msg()
    node_list = the_msg.points

    with open('/home/user/theRobot_ws/src/urdf_manipulator_simulate/src/mesh_planned.txt', 'r') as filehandle:
        filecontents = filehandle.readlines()

        for line in filecontents:
            xx, yy, zz = line[:-1].split("\t")
            y = float(yy)
            z = float(zz)
            node_list.append(Point(0.0, y, z))
    
    print(node_list)
       
    y = 0.2
    z = 0.6

    z1 = z - L3
    z2 = z1 - L0

    #q2 = math.acos((z2**2 + y**2 - L1**2 - L2**2)/(2*L1*L2))
    #q1 = math.asin((y*(L1 + L2*math.cos(q2) + z2*L2*math.sin(q2))) /
    #               ((L1 + L2*math.cos(q2))**2 + (L2*math.sin(q2))**2))
    q3 = q1 + q2

    return the_msg


if __name__ == '__main__':

    rospy.init_node('inverse_kinematics')
    listener = tf.TransformListener()
    service = rospy.Service('ik_algorithm', ik, callback)
    rospy.spin()
