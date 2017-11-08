from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')		#considering the gripper at the end
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
    rot = symbols('rot')
    # Create non-zero constant DH parameters
    s = { 
        alpha0:			0, a0:		0,	d1:		0.75, q1:   q1,
        alpha1:		-pi/2, a1:	 0.35,	d2:		   0, q2:   q2-pi/2,
        alpha2:			0, a2:	 1.25, 	d3: 	   0, q3:   q3,
        alpha3:		-pi/2, a3: -0.054,	d4:		1.50, q4:   q4,
        alpha4:		 pi/2, a4:		0,  d5:		   0, q5:   q5,
        alpha5:		-pi/2, a5:		0, 	d6:		   0, q6:   q6,
        alpha6:		    0, a6:		0,  d7:	   0.303, q7:   0}


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
                  [	  0,	        0,		1]])
    R_y = Matrix([[cos(rot), 		0, 	  sin(rot)],
                  [	  0, 		1, 		 0],
                  [-sin(rot),		0,	   cos(rot)]])
    R_x = Matrix([[		  1,		0,		  0],
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
    EE_pos = Matrix([[px],[py],[pz]])		   

    #Inverse position
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    Rrpy = R_z.evalf(subs={rot:yaw}) * R_y.evalf(subs={rot:pitch}) * R_x.evalf(subs={rot:roll}) * R_corr

    #For the wrist position
    wx = EE_pos[0] - (0.303)*Rrpy[0,2]
    wy = EE_pos[1] - (0.303)*Rrpy[1,2]
    wz = EE_pos[2] - (0.303)*Rrpy[2,2]

    # to calculate the joint angles, a top-down view of the kuka arm helpful. Take into account that the walk-through video was used to check how these angles were calculated:
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

    #for the last 3 joints, is necessary to substitube the previously calculated values:
    R0_3_new = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3_new = R0_3_new.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_G = R0_3_new.transpose() * Rrpy
    print(R3_G)
    
    #now, for a symbolic form of R3_EE
    R3_G_sym = (T3_4[0:3,0:3] * T4_5[0:3,0:3] * T5_6[0:3,0:3] * T6_G[0:3,0:3])
    print(R3_G_sym)
    
    #Last joint angles
    theta4 = atan2(R3_G[2,2],-R3_G[0,2]) 
    theta5 = atan2(sqrt(R3_G[0,2]*R3_G[0,2] + R3_G[2,2]*R3_G[2,2]),R3_G[1,2])
    theta6 = atan2(-R3_G[1,1], R3_G[1,0])

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    Forward_k = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [Forward_k[0,3],Forward_k[1,3],Forward_k[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinematics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)


if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
