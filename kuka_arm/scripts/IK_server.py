#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import pickle
import os


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols dor DH parameter table
 
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # KUKA KR210
        # Create Modified DH parameters
        # alpha = twist angle
        # a = link length
        # d = link offset
        # q = joint angle
        s = {alpha0:     0 , a0:      0, d1:  0.75, q1:       q1,
             alpha1: -pi/2., a1:   0.35, d2:     0, q2: q2-pi/2.,
             alpha2:     0 , a2:   1.25, d3:     0, q3:       q3,
             alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:       q4,
             alpha4:  pi/2., a4:      0, d5:     0, q5:       q5,
             alpha5: -pi/2., a5:      0, d6:     0, q6:       q6,
             alpha6:     0 , a6:      0, d7: 0.303, q7:        0}

        # Define Modified DH Transformation matrix
        def Trans_Matrix(alpha, a, d, q):
            MAT = Matrix([[            cos(q),            -sin(q),            0,              a],
                          [ sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                          [ sin(q)*sin(alpha),  cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                          [                 0,                  0,            0,             1]])
            return MAT

        # Create individual transformation matrices
        T0_1 = Trans_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = Trans_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = Trans_Matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = Trans_Matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = Trans_Matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = Trans_Matrix(alpha5, a5, d6, q6).subs(s)
        T6_G = Trans_Matrix(alpha6, a6, d7, q7).subs(s)
        
        # Transform from base_link to gripper
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G 

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            # Step 1: find the rotation of the gripper
            # Generate generic rotation matrices
            if not os.path.exists("R_G.p"):
                r, p, y = symbols('r p y')
        
                R_z = Matrix([[   cos(y),   -sin(y),         0],
                              [   sin(y),    cos(y),         0],
                              [        0,         0,         1]]) # YAW
    
                R_y = Matrix([[   cos(p),         0,    sin(p)],
                              [        0,         1,         0],
                              [  -sin(p),         0,    cos(p)]]) # PITCH
    
                R_x = Matrix([[        1,         0,         0],
                              [        0,    cos(r),   -sin(r)],
                              [        0,    sin(r),    cos(r)]]) # ROLL

                # Extrinsically multiply the rotation matrices so that gripper rotation can be
                # calculated from yaw, pitch and roll values provided by Gazebo
                R_G = R_z * R_y * R_x

                # Finally, a further rotation must be applied to account for the difference
                # between the URDF and Gazebo (world) frames
                Rot_correction = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
                R_G = R_G * Rot_correction

                pickle.dump(R_G, open("R_G.p", "wb"))

            else:
                R_G = pickle.load(open("R_G.p", "rb"))

            # Calculate rotation of gripper by substituing roll, pitch and yaw values
            # from Gazebo
            R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Step 2: find position of Wrist Centre
            # Find position of gripper
            POS_G = Matrix([[px],
                            [py],
                            [pz]])
    
            # To calculated wrist centre (Lesson 2: 18) find the gripper position (G_POS)
            # then subtract the gripper link from the position by multiplying
            # the length of the link (0.303) scaled by the rotation matrix
            # this ensures the subtraction goes in the correct direction
            # back toward the wrist centre            
            WC = POS_G - 0.303 * R_G[:,2] # WC should now be a 3 row column-matrix: x, y, z

            # With the Wrist Centre known the theta angles (from base to wrist centre)
            # must be calculated
            # firstly store the x, y and z coordinates of the WC in their own variables
            # so that the following formula are more human readable
            WCx = WC[0]
            WCy = WC[1]
            WCz = WC[2]

            # Theta1 can be calculated by projecting px, py and pz on to the x-y plane
            # example of this is in Lesson 2(19) - IK Examples
            theta1 = atan2(WCy, WCx)

            # Theta2 and Theta3 are equal to each other Project Robotic Arm: Pick & Place (15)
            A = sqrt((a3)**2 + (d4)**2).subs(s) # link length from joint 3 to wrist centre 
            B = sqrt((sqrt(WCx**2 + WCy**2) - a1)**2 + (WCz - d1)**2).subs(s)
            C = a2.subs(s) # link length from join 2 to joint 3

            a_1 = acos((B**2 + C**2 - A**2)/(2*B*C)) # angle between B and C
            b_1 = acos((A**2 + C**2 - B**2)/(2*A*C)) # angle between A and C
            c_1 = acos((A**2 + B**2 - C**2)/(2*A*B)) # angle between A and B

            theta2 = pi/2 - a_1 - atan2(WCz - d1, sqrt(WCx**2 + WCy**2) - a1).subs(s)
            theta3 = pi/2 - (b_1 + asin(0.054/A)) # using the angle generated by the sag in link4

            # Create rotation matrices to use with the theta values being calculate in this section
            # these rotation matrices can be extracted from the transformation matrices
            # for example, the rotation matrix from Joint_0 to Joint_1 is the 3x3 matrix in the top left
            # of the 4x4 transformation matrix T0_1
            #
            # To translate between joints, once again multiple the rotation matrices together
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Using instructions from 'Inverse Kinematics with Kuka KR210' (Project s.15) we can
            # find the rotation matrix from joint 3 to 6
            # To do this we multiply the rotation matrix from the base to the gripper (R_G) by the inverse of the
            # rotaiton matrix from joint_0 to joint_3
            R3_6 = R0_3.transpose() * R_G

            # Euler angles are then extracted from the rotation matrices
            # Reference to the project walkthrough was required for this section
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	    
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
