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

### Kinematic Analysis
Before the analysis, two checks were performed": static configuration and dynamic behavior. That is, observing the robot's model via `load_urdf.launch` in RViz allowed to better understand its composition of 6 joints plus 5 links and a gripper or end-effector, while running safe_spawner.sh under `Demo mode` helped to determine the robot's space of operation and limits. 

![alt text][image1]

Keeping in mind these two factors, the Denavit-Hartenberg convention was used to identify the reference frames of each joint while using the parameters included in `kr210.urdf.xacro` to get the links' lengths (a's) and offsets (d's). The generated table 

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

 also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


