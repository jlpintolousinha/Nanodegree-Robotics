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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')		#considering the gripper at the end
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
        rot = symbols('rot')
        # Create non-zero constant DH parameters
        s = { 
            alpha0:			0, a0:		0,	d1:		0.75, q1:   q1,
            alpha1:		-pi/2, a1:	 0.35,	d2:		   0, q2:	q2-pi/2,
            alpha2:			0, a2:	 1.25, 	d3: 	   0, q3:   q3,
            alpha3:		-pi/2, a3: -0.054,	d4:		1.50, q4:   q4,
            alpha4:		 pi/2, a4:		0,  d5:		   0, q5:   q5,
            alpha5:		-pi/2, a5:		0, 	d6:		   0, q6:   q6,
            alpha6:		    0, a6:		0,  d7:	   0.303, q7: 		  0}

        # Define DH Transformation matrix
        def DH_T(theta_i, ai_1, alphai_1, di):
            DH = Matrix([[			 cos(theta_i), 		   		-sin(theta_i),				 0, 			    ai_1],
                      [sin(theta_i)*cos(alphai_1), cos(theta_i)*cos(alphai_1),	-sin(alphai_1),    -sin(alphai_1)*di],
                      [sin(theta_i)*sin(alphai_1), cos(theta_i)*sin(alphai_1),   cos(alphai_1), 	cos(alphai_1)*di],
                      [							0,							0,				 0,						1]])
            return DH

        #Define Rotation Matrices:
        R_z = Matrix([[cos(rot), 	-sin(rot), 		0],
                     [sin(rot), 	 cos(rot),		0],
                     [	      0,	        0,		1]])
    	R_y = Matrix([[cos(rot), 		0, 	  sin(rot)],
                     [	     0, 		1, 		     0],
                     [-sin(rot),		0,	   cos(rot)]])
    	R_x = Matrix([[		  1,		    0,		      0],
                     [		  0,	 cos(rot),	   sin(rot)],
                     [		  0,	 sin(rot),	   cos(rot)]])

        # Create individual transformation matrices
        T0_1 = DH_T(q1, a0, alpha0, d1).subs(s)
        T1_2 = DH_T(q2, a1, alpha1, d2).subs(s)
        T2_3 = DH_T(q3, a2, alpha2, d3).subs(s)
        T3_4 = DH_T(q4, a3, alpha3, d4).subs(s)
        T4_5 = DH_T(q5, a4, alpha4, d5).subs(s)
        T5_6 = DH_T(q6, a5, alpha5, d6).subs(s)
        T6_G = DH_T(q7, a6, alpha6, d7).subs(s)

        #Composition of homogeneous transforms, from the base link:
        T0_2 = T0_1 * T1_2  #base-link to link 2
        T0_3 = T0_2 * T2_3	#base-link to link 3			   
        T0_4 = T0_3 * T3_4	#base-link to link 4	
        T0_5 = T0_4 * T4_5 	#base-link to link 5
        T0_6 = T0_5 * T5_6  #base-link to link 6
        T0_G = T0_6 * T6_G  #base-link to gripper

        # Extract rotation matrices from the transformation matrices
        R0_1 = T0_1[0:3,0:3]
        R0_2 = T0_2[0:3,0:3]					   
        R0_3 = T0_3[0:3,0:3]
        R0_4 = T0_4[0:3,0:3]
        R0_5 = T0_5[0:3,0:3]
        R0_6 = T0_6[0:3,0:3]
        R0_G = T0_G[0:3,0:3]

        #Correction matrix for the DH convention
        R_z_corr = R_z.evalf(subs={rot:pi})
        R_y_corr = R_y.evalf(subs={rot:-pi/2})
        R_corr = R_z_corr * R_y_corr

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

            #Position of end-effector	
            EE_pos =Matrix([[px],[py],[pz]])		   

            #Inverse position
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = R_z.evalf(subs={rot:yaw}) * R_y.evalf(subs={rot:pitch}) * R_x.evalf(subs={rot:roll}) * R_corr

            #For the wrist position
            wx = EE_pos[0] - (0.303)*Rrpy[0,2]
            wy = EE_pos[1] - (0.303)*Rrpy[1,2]
            wz = EE_pos[2] - (0.303)*Rrpy[2,2]

            # to calculate the joint angles, a top-down view of the kuka arm helpful. 
			#Take into account that the walk-through video was used to check how these angles were calculated:
            theta1 = atan2(wy,wx) 

            #checking the triangle formed by joints 2 and 3 plus the wrist center position:
            A = 1.5
            B = sqrt(pow(sqrt(wx*wx + wy*wy)-0.35,2) + pow(wz-0.75,2))
            C = 1.25

            #from the cosine law
            angle_a = acos((B**2 + C**2 - A**2)/(2*B*C))
            angle_b = acos((A**2 + C**2 - B**2)/(2*A*C))
            angle_c = acos((A**2 + B**2 - C**2)/(2*A*B))		 

            theta2 = pi/2 - angle_a - atan2((wz-0.75),sqrt(wx*wx + wy*wy)-0.35)
            theta3 = pi/2 - angle_b - 0.036  #due to the -0.054 offset in link 4
            
			#Inverse orientation
            #for the last 3 joints, is necessary to substitube the previously calculated values:
            R0_3_new = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3_new = R0_3_new.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_G = R0_3_new.transpose() * Rrpy

            #Last joint angles
            theta4 = atan2(R3_G[2,2],-R3_G[0,2]) 
            theta5 = atan2(sqrt(R3_G[0,2]*R3_G[0,2] + R3_G[2,2]*R3_G[2,2]),R3_G[1,2])
            theta6 = atan2(-R3_G[1,1], R3_G[1,0])

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
