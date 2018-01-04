## Project: 3D Perception

**The goals / steps to complete this project were the following:**  


1. Exercise 1 steps: Implementation of data filtering and RANSAC plane fitting.
2. Exercise 2 steps: Include clustering technique for segmentation.
3. Exercise 3 steps: Objects' features extracted and SVM trained. 
4. Implement object recognition via the exercises. 

[//]: # (Image References)
[image1]: ./joint_axes.png
[image2]: ./figure_theta123.png
[image3]: ./image1.png
[image4]: ./image2.png
[image5]: ./image3.png
[image6]: ./image4.png

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points

Summary

The proposed project was developed following these steps:
1. All required software and packages were installed in Linux VM. 
2. ROS packages were tested and ran under Linux enviroment.
3. A ROS enviroment was setup for Gazebo, RViz and Moveit! to properly execute. 
4. A kinematics analysis was performed for the Kuka Arm KR210; `kr210.urdf.xacro` file was also studied to determine the relationship between each of the arm's joints.
5. Coding was included in `IK_debug.py` to check Forward Kinematics calculation. 
6. `IK_server.py` was modified and positevely tested afterwards. 

### Forward Kinematics
Before the analysis, two checks were performed": static configuration and dynamic behavior. That is, observing the robot's model via `load_urdf.launch` in RViz (see image below) allowed to better understand its composition of 6 joints plus 5 links and a gripper or end-effector, while running safe_spawner.sh under `Demo mode` helped to determine the robot's space of operation and limits. I

![image1]

Keeping in mind these two factors, the Denavit-Hartenberg (DH) convention was used to identify the reference frames of each joint while using the parameters included in `kr210.urdf.xacro` to get the links' lengths (a's) and offsets (d's). As the base link (at the origin) is of interest in determining the proper position of the gripper, the element was included in the table. The DH table resulted as follows, including both the twist (**Alpha**) and joint (**Theta** or **q**) angles of the elements:  

Joint|Alpha | A | D | Theta
--- | --- | --- | --- | ---
0 | 0     | 0      | 0.75 | 0
1 | -pi/2 | 0.35   | 0    | -pi/2
2 | 0     | 1.25   | 0    | 0
3 | -pi/2 | -0.054 | 1.5  | 0
4 | pi/2  | 0      | 0    | 0
5 | -pi/2 | 0      | 0    | 0
6 | 0     | 0      | 0.303| 0

To properly read the table, each row should be accounted as the relationship between the lower and higher index joint (i.e., row 0 translates the relationship between the base frame and the first joint). On the other hand, the disposition of the joints 4, 5 and 6 allowed to consider the case of spherical wrist, and define the intersection point of such axes (at joint 5) as a `Wrist-Center` that would express how the base link and the gripper position are linked. As a matter of fact, the reference frames of the `Wrist-Center` can be translated to the that of the gripper by includding a link offset of 0.303 meters (see row 6 of the table).  

Keeping these parameters, a generalized homogeneous transform `T` between base link and gripper could be constructed by concatenating translations and rotations from the origin all the way through to the gripper, hence establishing the Forward Kinematics analysis of the robot. That been said, knowing the general form of a homogeneous transformation as

                    cos(theta_i)       |       -sin(theta_i)        |        0       |        ai_1
            sin(theta_i)*cos(alphai_1) | cos(theta_i)*cos(alphai_1) | -sin(alphai_1) |  -sin(alphai_1)*di
            sin(theta_i)*sin(alphai_1) | cos(theta_i)*sin(alphai_1) |  cos(alphai_1) |   cos(alphai_1)*di
                           0           |               0            |	  0      |          1
                           
It is possible to substitute the values of the DH table and concatenate each transformation so that

                T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

With zero being the base link and G the gripper. Take into account that rotation matrices can be extracted from the different homogeneous transforms as shown in lines 82 to 88 of `IK_server.py`.   

#### Inverse Kinematics 
At this point, it becomes necessary to determine all individual joint angles given a certain position (otherwise, the robot won't know which configuration to take for that). In order to derive these equations, the problem should be divided into inverse position kinematics and inverse orientation kinematics.

For the **inverse position** part, as the pose of the gripper is provided by a ROS service message in the form of pitch, roll and yaw, it is necessary to translate this coordinates in terms of the `Wrist-Center` reference frame. The reason is apparent when we realized it is the only link we have between the gripper and the base link of the robot. However, a simple translation is not enough: as the gripper reference frame is rotated 180° over the Z-axis plus another -90° over the y-axis (see image above), a low level correction is required before translating such frame to that of the `Wrist-Center` (lines 91 to 93 of `IK_server.py`).

Once the correction is calculated, and the gripper's pose assigned to the variables px, py and pz (lines 104 to 106 of `IK_server.py`), the `Wrist-Center` position can be calculated as:

            wx = EE_pos[0] - (0.303)*Rrpy[0,2]
            wy = EE_pos[1] - (0.303)*Rrpy[1,2]
            wz = EE_pos[2] - (0.303)*Rrpy[2,2]

Where `EE_pos` is the vector containind current gripper's pose; `Rrpy` stands as the total rotation of the gripper around the axes X (for roll), Y (for pitch) and Z (for yaw), with the mentioned correction included, referenced from the base link; and 0.303 meters is the distance between the gripper and the `Wrist-Center`. 

Finally, keeping in mind that the `Wrist-Center`pose is known and all links' lengths have been registered in the DH table and are also included in the `kr210.urdf.xacro` file, it was possible to get the angles associated to joints 1, 2 and 3 as:
1. `theta_1`,  calculated by projecting `Wrist-Center` position on the XY plane, for which `theta1 = atan2(wy,wx)`
2. `theta_2` and `theta_3`, determined via cosine law while using the image below as a reference:

![image2]

The resulting equations `theta2 = pi/2 - angle_a - atan2((wz-0.75),sqrt(wx*wx + wy*wy)-0.35)` and `theta3 = pi/2 - angle_b - 0.036` are included in lines 168 and 169 of the `IK_server.py` file. It is important to point out that the image provided for such calculations was found via the `Slack Robotics Nanodegree` channel and is referenced for such objective. 

For the **inverse orientation** side, considering the angles of the first three joints were available, and that the rotation matrix from the base link to the gripper was also available via `Rrpy`, the last joint angles were determined based on the expression `R3_G = R0_3_new.transpose() * Rrpy`. As a matter of fact, a symbolic evaluation of the resulting matrix was necessary to determine which terms could be used to extract the Euler Angles associated to it (the expression is included in lines 179 and 180 of the file `IK_debug.py`):

            -sin(q4)sin(q6) + cos(q4)cos(q5)cos(q6) |-sin(q4)cos(q6) - sin(q6)cos(q4)cos(q5) | -sin(q5)cos(q4)
                            sin(q5)cos(q6)          |             -sin(q5)sin(q6)            |     cos(q5)
            -sin(q4)cos(q5)cos(q6) - sin(q6)cos(q4) |-sin(q4)sin(q6)cos(q5) - cos(q4)cos(q6) | sin(q4)sin(q5)

The accounted to make the following relations for such Euler Angles as

                            theta4 = atan2(R3_G[2,2],-R3_G[0,2]) 
                            theta5 = atan2(sqrt(R3_G[0,2]*R3_G[0,2] + R3_G[2,2]*R3_G[2,2]),R3_G[1,2])
                            theta6 = atan2(-R3_G[1,1], R3_G[1,0])


### Project Implementation
As instructed, the `IK_server.py` file was filled with code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. The next impressions could be gathered about the project and its results:

1. The project as it is, performs the expect actions without deviating too much from the initially planned trajectory. However, problems regarding the time it may take to complete the task of grabbing and dropping the spawn may arise. The image below portrays the predicted trajectories from the robot's initial position and that after grabbing the spawn. 

![image3]  ![image4]

2. The transition from the robot's initial position to the spawn's location, although smooth, may present rotations of 180° or 360° around joint 6's Z-axis before reaching the spawn (see imgaes below). This could be the result of singularities (or changes of sign) encountered by atan2 at specific robot's positions (as a matter of fact, the test cases contained in `IK_debug.py` provided errors different than zero and higher than one at joint 6 even though the forward kinematic estimated errors were small)... Nonetheless, improvements could be included so that at such positions, in order to avoid unnecessary turnings of joint 6 around Z-axis, the latest valid value of atan2 is kept and used by the robot rather than the singularity itself, thus improving both performance and time to complete the task.

![image5]  ![image6]

3. The coding of Forward Kinematics' calculation was initially thought to consist of one matrix object per homogeneus transform. Althpough usefull in terms of understanding the process, it proved to be very unoptimal. The best solution was the one provided by the walkthrough video, were a function to determine the transform was defined (see lines 46 to 51 of `IK_server.py`). 

4. Much of the matrices that did not need to be included in the `for` loop for the inverse kinematics process were left outside of it for optimization objectives. Moreover, placing such matrices within the loop would be resource consuming and contrary to what was expected from the robot. 

### Final Thoughts

This project was not an easy task. Much of the code was grabbed from the walkthrough solution provided in the course due to lack of understanding regarding the Inverse Kinematics process and its implementation. I would personally thank the Slack channel because of the insight it provided regarding the calculation of the joint angles and how the cosine law was used for such purpose. 

I consider the lectures lack the contents necessary to complete this project in terms of the tools I could've used to better grip the assignment. However, I consider this as a lack of experience from my side and just another part of the learning process.  


