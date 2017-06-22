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
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import pickle
import os.path

# helper function to change a 3x3 rotational matrix to a 4x4 matrix
def quad_from_matrix(M):
            f = np.zeros((4,4))
            f[0,0] = M[0,0]
            f[0,1] = M[0,1]
            f[0,2] = M[0,2]
            f[1,0] = M[1,0]
            f[1,1] = M[1,1]
            f[1,2] = M[1,2]
            f[2,0] = M[2,0]
            f[2,1] = M[2,1]
            f[2,2] = M[2,2]
            f[3,3] = 1

            quad_final = tf.transformations.quaternion_from_matrix(f)
            print("Quad = " + "{:.3f}".format(quad_final[0]) + " , " + "{:.3f}".format(quad_final[1]) + " , " + "{:.3f}".format(quad_final[2]) + " , " + "{:.3f}".format(quad_final[3]))

            return quad_final

# convert yaw pitch row to rotational matrix
def ypr_to_rotation_matrix(q1, q2, q3):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,        cos(q1), -sin(q1)],
                  [ 0,        sin(q1),  cos(q1)]])

    R_y = Matrix([[ cos(q2),        0,  sin(q2)],
                  [       0,        1,        0],
                  [-sin(q2),        0,  cos(q2)]])

    R_z = Matrix([[ cos(q3), -sin(q3),        0],
                  [ sin(q3),  cos(q3),        0],
                  [ 0,              0,        1]])

    R = R_x * R_y * R_z
    return R

# convert rotational matrix to yaw pitch row
def rotation_matrix_to_ypr(R_XYZ):
    alpha = atan2(R_XYZ[1,0], R_XYZ[0,0]) # rotation about Z-axis
    beta  = atan2(-R_XYZ[2,0], sqrt(R_XYZ[0,0] * R_XYZ[0,0] + R_XYZ[1,0] * R_XYZ[1,0]))  # rotation about Y-axis
    gamma = atan2(R_XYZ[2,1], R_XYZ[2,2])  # rotation about X-axis
    return alpha, beta, gamma

def rotation_correction(R):
            # correction for URDF and DH Convention for Gripper
            # 90 degrees on the Z axis
            R_z = Matrix([[           cos(np.pi/2),         -sin(np.pi/2),            0],
                           [          sin(np.pi/2),          cos(np.pi/2),            0],
                           [                   0,                   0,            1]])
            # 90 degrees on the Y axis
            R_y = Matrix([[        cos(-np.pi/2),                   0,sin(-np.pi/2)],
                           [                   0,                   1,            0],
                           [      -sin(-np.pi/2),                   0,cos(-np.pi/2)]])
            R_correction = R_z # * R_y
            return  R * R_correction

# conversion from DH convention to URDF convention
def dh_to_urdf_rotation_correction(R):
            # correction for URDF and DH Convention for Gripper
            # 180 degrees on the Z axis
            R_z = Matrix([[           cos(np.pi),         -sin(np.pi),            0],
                           [          sin(np.pi),          cos(np.pi),            0],
                           [                   0,                   0,            1]])
            # 90 degrees on the Y axis
            R_y = Matrix([[        cos(-np.pi/2),                   0,sin(-np.pi/2)],
                           [                   0,                   1,            0],
                           [      -sin(-np.pi/2),                   0,cos(-np.pi/2)]])
            R_correction = R_z * R_y
            return  R * R_correction

# this generates the forward functions and save them into pickle files
# this is only run once if program cannot find a previously saved pickle file
def generate_forward():
            # Define DH param symbols
            # alpha0: Z0 // Z1 : 0
            # alpha1: Z1 |_ Z2 : -90 + alpha1
            # alpha2: Z2 // Z3 : 0 + alpha2
            # alpha3: Z3 |_ Z4 : -90
            # alpha4: Z4 |_ Z5 :  90 + alpha4
            # alpha5: Z5 |_ Z6 : -90
            # alpha6: Z6 // Z7 :   0

            # a0: Orig1 - Orig0 along X1: 0
            # a1: Orig2 - Orig1 along X2: 0.75
            # a2: Orig3 - Orig2 along X3: 2 - 0.75 = 1.25
            # a3: Orig4 - Orig3 along X4: 1.946 - 2 = -0.054
            # a4: Orig5 - Orig4 along X5: 0
            # a5: Orig6 - Orig5 along X6: 0
            # a6: Orig7 - Orig6 along X7: 0

            # d1: Orig1 - Orig0 along Z1: 0.33
            # d2: Orig2 - Orig1 along Z2: 0
            # d3: Orig3 - Orig2 along Z3: 0
            # d4: Orig4 - Orig3 along Z4: 1.85 - 0.35 = 1.5
            # d5: Orig5 - Orig4 along Z5: 0
            # d6: Orig6 - Orig5 along Z6: 0
            # d7: Orig7 - Orig6 along Z7: 2.153 - 1.85 = 0.303

            # q1: X0 // X1 : q1
            # q2: X1 |_ X2 : 90
            # q3: X2 // X3 : 0
            # q4: X3 // X4 : q4
            # q5: X4 // X5 : 0
            # q6: X5 // X6 : q6
            # q7: X6 // X7 : 0


            
            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

            s = {alpha0:        0,  a0:      0, d1: 0.75, 
                 alpha1: -np.pi/2,  a1:   0.35, d2:    0, q2: q2-np.pi/2,
                 alpha2:        0,  a2:   1.25, d3:    0, 
                 alpha3: -np.pi/2,  a3: -0.054, d4:  1.5, 
                 alpha4:  np.pi/2,  a4:      0, d5:    0, 
                 alpha5: -np.pi/2,  a5:      0, d6:    0, 
                 alpha6:        0,  a6:      0, d7:0.303, q7:          0}

            # Modified DH params


            
            # Define Modified DH Transformation matrix
            T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                           [                   0,                   0,            0,               1]])
            T0_1 = T0_1.subs(s)

            T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                           [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                           [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                           [                   0,                   0,            0,               1]])
            T1_2 = T1_2.subs(s)

            T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                           [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                           [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                           [                   0,                   0,            0,               1]])
            T2_3 = T2_3.subs(s)

            T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                           [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                           [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                           [                   0,                   0,            0,               1]])
            T3_4 = T3_4.subs(s)

            T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                           [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                           [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                           [                   0,                   0,            0,               1]])
            T4_5 = T4_5.subs(s)

            T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                           [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                           [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                           [                   0,                   0,            0,               1]])
            T5_6 = T5_6.subs(s)

            T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                           [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                           [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                           [                   0,                   0,            0,               1]])
            T6_7 = T6_7.subs(s)

            # Create individual transformation matrices
            T0_2 = simplify(T0_1 * T1_2)
            print("done T0_2")
            T0_3 = simplify(T0_2 * T2_3)
            print("done T0_3")
            T0_4 = simplify(T0_3 * T3_4)
            print("done T0_4")
            T0_5 = simplify(T0_4 * T4_5)
            print("done T0_5")
            T0_6 = simplify(T0_5 * T5_6)
            print("done T0_6")
            T0_7 = simplify(T0_6 * T6_7)
            print("done T0_7")
            
            # correction for URDF and DH Convention for Gripper
            # 180 degrees on the Z axis
            R_z = Matrix([[           cos(np.pi),         -sin(np.pi),            0,              0],
                           [          sin(np.pi),          cos(np.pi),            0,              0],
                           [                   0,                   0,            1,              0],
                           [                   0,                   0,            0,              1]])
            # 90 degrees on the Y axis
            R_y = Matrix([[        cos(-np.pi/2),                   0,sin(-np.pi/2),              0],
                           [                   0,                   1,            0,              0],
                           [      -sin(-np.pi/2),                   0,cos(-np.pi/2),              0],
                           [                   0,                   0,            0,              1]])
            R_correction = simplify(R_z * R_y)

            T_final = simplify(T0_7 * R_correction)


            pickle.dump( T0_1, open( "T0_1.p", "wb" ) )
            pickle.dump( T0_2, open( "T0_2.p", "wb" ) )
            pickle.dump( T0_3, open( "T0_3.p", "wb" ) )
            pickle.dump( T0_4, open( "T0_4.p", "wb" ) )
            pickle.dump( T0_5, open( "T0_5.p", "wb" ) )
            pickle.dump( T0_6, open( "T0_6.p", "wb" ) )
            pickle.dump( T0_7, open( "T0_7.p", "wb" ) )
            pickle.dump( T_final, open( "T_final.p", "wb" ) )
            
            return None

# loads the forward functions from pickle files
def load_forward():
            if not os.path.exists("T_final.p"):
                generate_forward()

            T0_1 = pickle.load( open( "T0_1.p", "rb" ) )
            T0_2 = pickle.load( open( "T0_2.p", "rb" ) )
            T0_3 = pickle.load( open( "T0_3.p", "rb" ) )
            T0_4 = pickle.load( open( "T0_4.p", "rb" ) )
            T0_5 = pickle.load( open( "T0_5.p", "rb" ) )
            T0_6 = pickle.load( open( "T0_6.p", "rb" ) )
            T0_7 = pickle.load( open( "T0_7.p", "rb" ) )
            T_final = pickle.load( open( "T_final.p", "rb" ) )
            
            return None

# Forward Kinematics calculation
# only calculate T0_4 and T_final because that is all we need
def k_cal(cq1, cq2, cq3, cq4, cq5, cq6):
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

            s = {alpha0:        0,  a0:      0, d1: 0.75, 
                 alpha1: -np.pi/2,  a1:   0.35, d2:    0, q2: q2-np.pi/2,
                 alpha2:        0,  a2:   1.25, d3:    0, 
                 alpha3: -np.pi/2,  a3: -0.054, d4:  1.5, 
                 alpha4:  np.pi/2,  a4:      0, d5:    0, 
                 alpha5: -np.pi/2,  a5:      0, d6:    0, 
                 alpha6:        0,  a6:      0, d7:0.303, q7:          0}
            
            if not os.path.exists("T_final.p"):
                generate_forward()

            T0_1 = pickle.load( open( "T0_1.p", "rb" ) )
            T0_2 = pickle.load( open( "T0_2.p", "rb" ) )
            T0_3 = pickle.load( open( "T0_3.p", "rb" ) )
            T0_4 = pickle.load( open( "T0_4.p", "rb" ) )
            T0_5 = pickle.load( open( "T0_5.p", "rb" ) )
            T0_6 = pickle.load( open( "T0_6.p", "rb" ) )
            T0_7 = pickle.load( open( "T0_7.p", "rb" ) )
            T_final = pickle.load( open( "T_final.p", "rb" ) )


            final = T_final.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6})
            Till4 = T0_4.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6})

            #print("T0_1 = ",T0_1.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T0_2 = ",T0_2.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T0_3 = ",T0_3.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T0_4 = ",T0_4.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T0_5 = ",T0_5.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T0_6 = ",T0_6.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T0_7 = ",T0_7.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))
            #print("T_final = ",T_final.evalf(subs={q1: cq1, q2: cq2, q3: cq3, q4: cq4, q5: cq5, q6: cq6}))

            #print ("x = ",final[0,3])
            #print ("y = ",final[1,3])
            #print ("z = ",final[2,3])
            #print ("Till3 = ",Till3)

            R0_4 = Till4[0:3,0:3]
            #print ("R0_3 = ",R0_3)
            R0_4_c = dh_to_urdf_rotation_correction(R0_4)

            Inv_R0_4 = R0_4.pinv()
            #print ("Inv_R0_3 = ",Inv_R0_3)
            Inv_R0_4_c = R0_4_c.pinv()

            #RR03 = R0_3 *Inv_R0_3 
            #print ("RR03 = ",RR03)

            # R3_6 = inv(R0_3) * Rrpy

            return final[0,3], final[1,3], final[2,3], Inv_R0_4_c, final

# IK for joint 1,2,3 (theta1, theta2, theta3) using simple geometry
# Using the End-effector position (setting theta4, theta5, theta6 = 0) to calculate (theta1, theta2, theta3)
#   is the same as using the Wrist Center to calculate (theta1, theta2, theta3)
def ik_cal_xyz(px, py, pz):
            # Find theta1 base on end-effector position project to ground plane (pz = 0)
            # joint1 limit: -185 degree to 185 degree
            theta1 = atan2(py, px)

            # joint2 limit: -45 degree to 85 degree
            #               -0.78 to 1.48
            j2_lower_rad = np.deg2rad(-45.0)
            j2_upper_rad = np.deg2rad(85.0)

			# joint3 limit: -210 degree to 155-90 = 65 degree
            #               -3.66 to 1.13
            j3_lower_rad = np.deg2rad(-210.0)
            j3_upper_rad = np.deg2rad(65.0)

            # distance from Orig2 to P
            orig2z = 0.75  # d1
            orig2x = 0.35 * cos(theta1)  # a1
            orig2y = 0.35 * sin(theta1)

            #print("orig2x = ",orig2x)
            #print("orig2y = ",orig2y)
            #print("orig2z = ",orig2z)

            O2P = sqrt( np.square(px - orig2x) + np.square(py - orig2y) + np.square(pz - orig2z))

            O2Pz = orig2z - pz

            # distance from Orig3 to P is fixed
            # a1 = 0.35, d4 = 1.5, d7 = 0.303 => 2.153
            O3P = sqrt( np.square(2.153 - 0.35) + np.square(1.946 - 2.0) )


            # angle g1 is the angle drop below X-Y plane from O2
            g1 = asin(O2Pz / O2P)
            #print("g1 = ",g1)

            # a2 = 1.25
            f1 = acos( (np.square(1.25) + np.square(O2P) - np.square(O3P)) / (2 * 1.25 * O2P))
            #print("f1 = ",f1)

            theta2 = np.pi/2 + g1 - f1


            # a2 = 1.25
            orig3z = orig2z + (1.25 * cos(theta2))

            #print("orig3z = ",orig3z)
            #print("O3P = ",O3P)

            # need to offset by a3 = -0.054
            s = orig3z - 0.054 - pz

            #theta3 = atan2(s, O3P)
            theta3 = asin(s / O3P) - theta2

            #print("j2 limit = ",j2_lower_rad, " ", j2_upper_rad)
            #print("j3 limit = ",j3_lower_rad, " ", j3_upper_rad)

            #print("theta1 = ",theta1)
            #print("theta2 = ",theta2)
            #print("theta3 = ",theta3)
			
            return theta1, theta2, theta3


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        logfile = open("ik-log.txt", "a")
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            print('Pose ',x,' / ',len(req.poses))

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            target_quad = [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w]

            # Calculate joint angles using Geometric IK method
            # Step 1: calculate IK for joint 1,2,3 (theta1, theta2, theta3) using simple geometry
            theta1, theta2, theta3 = ik_cal_xyz(px,py,pz)

            theta4, theta5, theta6 = 0.0,0.0,0.0

            # for performance purpose, we only calculate (theta4, theta5, theta6) in the last pose for the Pick phase
            # drop the if check to calculate for every step
            if (x >= (len(req.poses) - 1) and (px > 0)):
                # Step 2: get the orientation of sperical wrist given (theta1, theta2, theta3), setting (theta4, theta5, theta6) = 0
                x1, y1, z1, Inv_R0_4_c, final = k_cal(theta1, theta2, theta3, theta4, theta5, theta6)
                # Step 3: get the target orientation given by the target pose
                (target_euler4, target_euler5, target_euler6) = tf.transformations.euler_from_quaternion(target_quad)
                print("Pre adjust XY Result : ")

                fquad = quad_from_matrix(final)
                (ct4, ct5, ct6) = tf.transformations.euler_from_quaternion(fquad)

                print("Angles Cal: ", ct4, ct5, ct6)
                print("Target Angles: ", target_euler4, target_euler5, target_euler6)

                # Step 4: find the rotation we need to move the wrist to the target orientation
                dt4 = ct4 - target_euler4
                dt5 = ct5 - target_euler5
                dt6 = ct6 - target_euler6

                err = dt4*dt4 + dt5*dt5 + dt6*dt6

                # Step 5: move joints 4,5,6 towards the target orientation
                theta4 -= dt4
                # since joints 4,5,6 is in the Z-X-Z orientation, we need some complex calculation to 
                # find a solution to match all 3 target rotation
                # for simplicity, we adjust to only the larger angle of dt5 or dt6
                if (abs(dt5) > abs(dt6)):
                    theta5 -= dt5
                else:
                    # turn theta4 90 degrees and theta6 -90 degress so that theta5 rotates along Z axis
                    theta4 += pi/2
                    theta6 -= pi/2
                    theta5 -= dt6

                # Step 6: after rotation, the End-Effector position is changed
                #  we do translation adjust again by using forward kinematics to find the current x,y,z 
                #  and try to adjust it back to the given px, py, pz
                x1, y1, z1, Inv_R0_4_c, final = k_cal(theta1, theta2, theta3, theta4, theta5, theta6)
                dx = px - x1
                dy = py - y1
                dz = pz - z1
                adjust_x = px + dx
                adjust_y = py + dy
                adjust_z = pz + dz
                theta1, theta2, theta3 = ik_cal_xyz(adjust_x,adjust_y,adjust_z)

                # For more accurate result, we can keep repeating Step 2 to 6

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]

            str1 = 'Pose ' + str(x) + '/' + str(len(req.poses)) + "\n"
            str1 = str1 + "Pose : " + "{:.3f}".format(px) + " , " + "{:.3f}".format(py) + " , " + "{:.3f}".format(pz) + " , " + "{:.3f}".format(req.poses[x].orientation.x) + " , " + "{:.3f}".format(req.poses[x].orientation.y) + " , " + "{:.3f}".format(req.poses[x].orientation.z) + " , " + "{:.3f}".format(req.poses[x].orientation.w) + "\n"
            str1 = str1 + "Joint angles : " + str(N(theta1)) + " , " + str(N(theta2)) + " , " + str(N(theta3)) + " , " + str(N(theta4)) + " , " + str(N(theta5)) + " , " + str(N(theta6))  + "\n"
            print(str1)
            logfile.write(str1)
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
