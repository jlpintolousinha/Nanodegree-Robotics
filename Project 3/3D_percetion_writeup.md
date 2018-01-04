## Project: 3D Perception

**The goals / steps to complete this project were the following:**  

1. Get familiar with the concept of Point Cloud Library.  
2. Implement data filtering techniques for image recognition from a RGB camera. 
3. Understand Random Sample Consensus (RANSAC) plane fitting for identifying the points of an identified model.
4. Perform clustering segmentation in the generated Point Cloud Library.
5. Extract object's features extracted and train a Support Vector Machine (SVM). 
6. Recognixe objects detected by the RGB camera.

[//]: # (Image References)
[image3]: ./image1.png
[image4]: ./image2.png
[image5]: ./image3.png
[image6]: ./image4.png

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points

Summary

The proposed project was developed by:
1. The implementation of data filtering and RANSAC plane fitting.
2. The clustering or segmentation of detected objects.
3. The extraction of features from the object and training of SVM. 
4. The object's recognition via project implementation.  

#### Data filtering / RANSAC plane fitting

#### Clustering

#### Object Features's Extraction

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


