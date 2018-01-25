## Project: Follow Me

**The goals / steps to complete this project were the following:**  

1. Get familiar with the concept of Neural Networks and its uses on image processing.
2. Learn about Deep Learning functions defined in the TensorFlow® library .
3. Understand the features of Fully Connected Networks (FCN) and its applications. 
4. Grasp the 'Scene Understanding' concept via Semantic Segmentation of pixels. 
6. Use the RoboND Quad Sim application as an insight on when to use the abovesaid techniques and what information can be extracted from them. 

[//]: # (Image References)
[image1]: ./image_batch_size_60_256.png
[image2]: ./image_batch_size_70_160.png
[image3]: ./image3_while_following_the_target.png
[image4]: ./image4_while_at_patrol_without_target.png
[image5]: ./image5_while_at_patrol_with_target.png

## [Rubric](https://review.udacity.com/#!/rubrics/1155/view) Points

### Summary

The proposed project was developed by:
1. The implementation of separable convolution layers. 
2. The definition of a network architecture.
3. The training of the model, assuming different values for the involved hyperparameters. 
5. The predictions provided by the model as based on a validation dataset.
5. The calculation of Intersection over Union metric (IOU) for model's performance evaluation. 

Keep in mind that these steps were performed through the use of an existing dataset of training/validation images. A different set of images could of course be constructed via the simulator's Spawn Crown function, but the simulator crashed in many occasions during the recording, and for that reason no new data was collected for train/validation purposes.  

In addition, many of the concepts herein treated were implemented as functions in `model_training.html` that were used as necessary all along the way. 

#### Separable Convolution Layers

As defined in the lessons, Separable Convolution consists of "a convolution performed over each channel of an input layer, followed by a 1x1 convolution that takes the output channels from the previous step, then combining them into another output layer" (Udacity, 2018). The layers are implemented in line 6 of `model_training.html` and allow to reduce the number of parameters to be traversed by the kernels, thus improving the performance of the network and to some extent, any overfitting. The data already provided was used as an input to the functions in line 10 of `model_training.html` after instantiating the model object. 

In addition, batch normalization was implemented as well in line 6 of `model_training.html` as an additional way to scale down the number of parameters to analize, further optimizing the network's training. As explained in the lessons, "the inputs to layers within the network are normalized" while using both mean and variance of the values in the current selected batch of data.

#### Network Architecture

This section was

#### Model Training/Hyperparameters

This section 

![image1]
![image2]

### Predictions/IOU
The next impressions can be gathered from the project's implementation:

1. There's a high risk of overfitting the model if number of features N > 50. This comes out after several attempts were made for N (10, 30, 50, 100, 250). As the number increased, the reported precision while runnning `train_svm.py` increased as well (even 96% reported), although no improvement was observed in the number of objects detected (as reported in RViz). Information in Slack helped in determinining under which conditions this situation would appear.  

2. Changing the number of bins from 32 to 16 improved the overfitting situation as more items could be detected in RViz. However, a number below such limit reduced considerably the reported precision of the model while running `train_svm.py`.

3. Setting `using_hsv=True` in `compute_color_histograms()` improved both the reported precision of the trained model and the number of detected objects in RViz. As a matter of fact, the setting had to be included in both `features.py` and `project_template.py` in order to get consistent results while running `pick_place_project.launch`. 

4. Nevertheless, none of the results observed in Rviz (for any of the test worlds) provided a so called 'passing submission'. As a matter of fact, during `test1.world` case, several objects were detected at the begining (over 20) and decayed to detect 2 out of 3 objects (in the image, 'biscuits' and 'soap2' were detected). This behavior could be due to hardware limitations, but it is unknown until which point. 

`Images while following the target`

![image3]

`Images while at patrol without target` 

![image4]

`Images while at patrol with target`

![image5]

5. Can't personally say 

### Final Thoughts

I consider this project to be manageable in the proposed time. Information provided via the Slack channel helped also in determining the errors that the code could have. In addition, the lectures provided good information regarding the CNN and FCN methods for image processing making this experience nicer than others. 


