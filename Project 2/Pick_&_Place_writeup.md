## Project: Kinematics Pick & Place

**The goals / steps to complete this project were the following:**  


1. Set up a ROS Workspace.
2. Download or clone the project repository contained in GitHub
3. Experiment with the forward_kinematics environment and familiarize with the Gazebo and Rviz environment.
4. Launch safe_spawner.sh in demo mode and observe KR210's behavior
5. Perform Kinematic Analysis for the  KR210, using the methodology described in the lectures. 
6. Fill in the `IK_server.py` with code to perform Inverse Kinematics analysis. 


[//]: # (Image References)
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

Summary

The proposed project was developed following these steps:
1. All required software and packages were installed in Linux VM. 
2. ROS packages were tested and ran under Linux enviroment.
3. A ROS enviroment was setup for Gazebo, RViz and Moveit! to properly execute. 
4. A kinematics analysis was performed for the Kuka Arm KR210; `kr210.urdf.xacro` file was also studied to determine the relationship between each of the arm's joints.
5. Coding was included in `IK_debug.py` to check Forward Kinematics calculation. 
6. `IK_server.py` was modified and positevely tested afterwards. 

### Forward Kinematics
Before the analysis, two checks were performed": static configuration and dynamic behavior. That is, observing the robot's model via `load_urdf.launch` in RViz allowed to better understand its composition of 6 joints plus 5 links and a gripper or end-effector, while running safe_spawner.sh under `Demo mode` helped to determine the robot's space of operation and limits. 

![alt text][image1]

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

To properly read the table, each row should be accounted as the relationship between the lower and higher index joint (i.e., row 0 translates the relationship between the base frame and the first joint). On the other hand, the disposition of the frames at joints 4, 5 and 6 allowed to define a single point as a `Wrist Center` that would ease the relationship between the base link and the gripper position. As a matter of fact, the reference frames of the `Wrist Center` can be translated to the that of the gripper by includding a link offset of 0.303 meters (see row 6 of the table).  

Keeping these parameters, a generalized homogeneous transform `T` between base link and gripper could be constructed by concatenating translations and rotations from the origin all the way through to the gripper, hence establishing the Forward Kinematics analysis of the robot. That been said, knowing the general form of a homogeneous transformation as

cos(theta_i)| -sin(theta_i)| 0 | ai_1
--- | --- | --- | --- 
sin(theta_i)\*cos(alphai_1) | cos(theta_i)\*cos(alphai_1) |	-sin(alphai_1) |    -sin(alphai_1)\*di
sin(theta_i)\*sin(alphai_1) | cos(theta_i)\*sin(alphai_1) | cos(alphai_1) | 	cos(alphai_1)\*di
0                           |						                	0|		     		 0|			        			1

It is possible to substitute the values of the DH table and concatenate each transformation so that

T0_G = T0_1 \* T1_2 \* T2_3 \* T3_4 \* T4_5 * T5_6 * T6_G

With zero being the base link and G the gripper. 

#### Inverse Kinematics 
At this point, it becomes necessary to determine all individual joint angles. the problem dividdes into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


