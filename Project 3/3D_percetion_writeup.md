## Project: 3D Perception

**The goals / steps to complete this project were the following:**  

1. Get familiar with the concept of Point Cloud Library (PCL).  
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

### Summary

The proposed project was developed by:
1. The implementation of data filtering and RANSAC plane fitting.
2. The clustering or segmentation of detected objects.
3. The extraction of features from the object and training of SVM. 
4. The object's recognition via project implementation.  

Keep in mind that most of the techniques herein described were applied to data coming out of a RGB camera. Such data had to be trasmitted via ROS messages over specified topics and converted to PCL (and XYZ positions), and then back into ROS messages for its transmission in RViz. In general, the creation of ROS publishers and further instructions is implicit all over the project. 

#### Data filtering / RANSAC plane fitting
This section was part of Exercise 1. The `pcl_callback()` function had to include instructions to properly filter out the data coming from the RGB camera. Threen different techniques were used in this part before the PCL could be clustered:
1. Voxel Grid Downsampling to reduce the incoming amount of data. 
2. Passthrough filter to separate the table from the detectable objects. 
3. Noise filtering to eliminate outlier points (e.g., not belonging to any object)

On the other hand, the RANSAC technique was also used and it allowed to narrow down the recorded points by determining whether they belonged to a specific model or shape. This way, it was possible to separate the objects from the table itself by defining a plane as segmentation shape (line 91 of `project_template.py`) and whether any points were within a threshold distance. 

#### Clustering
This section was part of Exercise 2. After the filtering, it was necessary to create a point cloud for any different object present on the table. The way to do so was by defining clusters of data (or points) over the XYZ space, for which different methods exists in the literature (K-means clustering, DBSCAN algorithm, etc). 

In our case, a 'k-d tree' was used as a data structure to organize a number of points in a k-dimensions space. Such struture allows to determine the set of point neighbors to a specific location (or radius). The Eucliden-clustering technique follows such this principle and it was used in `project_template.py` to reduce the amount of processing time of the streamed point cloud (lines 105 to 117). A color was assigned to each point cloud afterwards to differentiate the detected objects (line 121 to 134). 

#### Object Features's Extraction
This section was part of Exercise 3. Once all of the objects were extracted and clustered, instructions to create recognizable objects from the generated point clouds were included in the `pcl_callback()` function (see `project_template.py`, lines 147 to 180).  The funtions `compute_color_histograms()` and `get_nomarls()` stand out as a way to get the feature vector necessary to get the Support Vector Machine algorithm implemented (lines 166 to 168 in `project_template.py`). 

In a nutshell, the algorithm allows to characterize the parameter space (PCL) of the detected objects into discrete classes by usign a 'trained' feature vector and label (e.g., the SVM is trained to recognize whether an image contains some specific "based on an input feature vector composed of color histograms" (slide 12, Lesson 19:Object Recognition)). That been said, once the training is applied to the whole parameter space, it is possible to characterize new objects for which only features exists and afterwards assign it to a specific class. After this point, if the SVM predictions are good, such training can be used for the object recognition phase.

The latest paragraph accounts for a series of functions distributed among the files `features.py`, `capture_features.py` and `train_svm.py`. Take into account however that the mentioned SVM training set maybe not be accurate if the characteristics from which such feature vector was extracted are not enough. In order to improve such model, more and better features could to be extracted for instance. 

### Project Implementation
As instructed,  .The next impressions could be gathered about the project and its results:

1. The project as it is, performs the expect actions without deviating too much from the initially planned trajectory. However, problems regarding the time it may take to complete the task of grabbing and dropping the spawn may arise. The image below portrays the predicted trajectories from the robot's initial position and that after grabbing the spawn. 

![image3]  ![image4]

2. The transition from the robot's initial position to the spawn's location, although smooth, may present rotations of 180° or 360° around joint 6's Z-axis before reaching the spawn (see imgaes below). This could be the result of singularities (or changes of sign) encountered by atan2 at specific robot's positions (as a matter of fact, the test cases contained in `IK_debug.py` provided errors different than zero and higher than one at joint 6 even though the forward kinematic estimated errors were small)... Nonetheless, improvements could be included so that at such positions, in order to avoid unnecessary turnings of joint 6 around Z-axis, the latest valid value of atan2 is kept and used by the robot rather than the singularity itself, thus improving both performance and time to complete the task.

![image5]  ![image6]

3. The coding of Forward Kinematics' calculation was initially thought to consist of one matrix object per homogeneus transform. Althpough usefull in terms of understanding the process, it proved to be very unoptimal. The best solution was the one provided by the walkthrough video, were a function to determine the transform was defined (see lines 46 to 51 of `IK_server.py`). 

4. Much of the matrices that did not need to be included in the `for` loop for the inverse kinematics process were left outside of it for optimization objectives. Moreover, placing such matrices within the loop would be resource consuming and contrary to what was expected from the robot. 

### Final Thoughts

This project was not an easy task. Much of the code was grabbed from the walkthrough solution provided in the course due to lack of understanding regarding the Inverse Kinematics process and its implementation. I would personally thank the Slack channel because of the insight it provided regarding the calculation of the joint angles and how the cosine law was used for such purpose. 

I consider the lectures lack the contents necessary to complete this project in terms of the tools I could've used to better grip the assignment. However, I consider this as a lack of experience from my side and just another part of the learning process.  


