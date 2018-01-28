## Project: Follow Me

**The goals / steps to complete this project were the following:**  

1. Get familiar with the concept of Neural Networks and its uses on image processing.
2. Learn about Deep Learning functions defined in the TensorFlow® library .
3. Understand the features of Fully Convolutional Networks (FCN) and its applications. 
4. Grasp the 'Scene Understanding' concept via Semantic Segmentation of pixels. 
6. Use the RoboND Quad Sim application as an insight on when to use the abovesaid techniques and what information can be extracted from them. 

[//]: # (Image References)
[image1]: ./image_batch_size_60_256.png
[image2]: ./image_batch_size_70_160.png
[image3]: ./image3_while_following_the_target.png
[image4]: ./image4_while_at_patrol_without_target.png
[image5]: ./image5_while_at_patrol_with_target.png
[image6]: ./fcn.png
[image7]: ./architecture.png

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

The project's tasks were to search for a specific target, save its location and follow it through its path as time ran over. One way to do it was by using fully connected layers. However, as they don't preserve spatial information through the network (as we need a 4D tensor), the replacement of them by convolutional layers presents an advantage during image inference: it preserves spatial information along the different convolution layers.

As defined in the lessons, the architecture of a FCN's (see image below. Udacity, 2018) it is comprised by one or more encoders and one or more decoders (the more the elements, the more deep a network is). A depthwise separable convolution, or `Separable Convolution`, was performed in this case properly address the abovementioned tasks without impacting the eficiciency of the network. 

![image6]

The separable convolution technique consists of "a spatial convolution performed independently over each channel of an input, followed by a 1x1 convolution (or `pointwise convolution`), projecting the channels output by the depthwise convolution onto a new channel space" (Chollet, 2016). In other words, it takes the output channels from the previous steps and combines them upstream into another output layer. This way, the number of parameters to be traversed by the patches (or kernels) is reduced hence improving the performance of the network and to some extent any overfitting. 

The method is implemented in lines 6 and 7 of `model_training.html`, where different functions in Keras library has been used for instantiating different layers: separable, batch normalization and bilinear upsampling. As a side note, the last two layers where included to improve the performance of the layer (by normalizing the inputs) and to increase the input pixels to higher resolutions. 

#### Network Architecture

The encoders and decoders were defined in lines 8 and 9 of `model_training.html`, whereas the whole model was assembled in line 10 as has the scheme portrayed in the image below. This design was the result of a number of trials with a different number or encoders/decoders (as well as filter sizes) which provided global precision values from 20% to 25%

As a matter of fact, an initial proposal of 2 encoders & 2 decoders was instantiated in the notebook, but it did not reached a precision above 24% even after modifying the provided hyperparameters. In many cases, overfitting was observed after the model finished its validation phase, making it unadequate for the tasks in ind. It was necessary to go further deep the neural network: a 5-by-5 encoders-decoders scheme (see image below) was proposed to increase the depth of the filters and get the images' pixels properly detected and classified.

![image7]

#### Model Training/Hyperparameters

Different combinations of the provided hyperparameters were tested. At the beginning it was though that by using a high value of learning rate, it would possible to quickly reach an optimum. But that proved wrong giving the low global IOU calculated at the end of the process. As a matter of fact, assuming 0.1 as an initial value proved worng after the first 5 epochs where the model crearly started to overfit the data. 

On the other hand, the number of epochs was progressively modified from 5 to 15 while keeping the rest of the hyperparameters constant, but this was proven wrong once more after the model showed signs of overfitting. That been said, it was not possible to get a precision above 25% with a 2-by-2 encoders-decoders combination, which marked the time for the modified model of a 5-by-5 encoders-decoders to  further deepen the network.

The number of steps-per-epoch was also modified from the default value of 50. The images contained in the address `data/train/images` accounted for a total of 4131, which is why assuming a batch size of 60, the steps-per-epoch hyperparameter was calculated to be 69. In general, the modification of either batch-size or steps-per-epoch numbers did have a proportional effect on the time required for training: the higher the numbers, the more time it was required for the model to train. 

Finally, the combination shown in line 13 of `model_training.html` (including the 5-by-5 encoders-decoders model) was the one that after many trials reached an improved global IOU of 35%, although well below the required passing threshold. Once more, the batch size was modified to 30 (plus the corresponding steps-per-epoch) in other to improve the training time, but the model got terribly overfitted after the 10th epoch (see image below). From this point onwards, as the batch-size needed to be increased, training started to be performed in AWS servers in order to speed-up the process while testing another combination of parameters.

![image2]

After the increasing the batch-size again to 60, it was not clear if modifying any other hyperparameter would get a better result. Much of the abovesaid numbers were tuned without any positive change whatsoever, thus questioning if more images would be required to improve the image recognition step; However, in line 11 of `model_training.html` the statement `For this project, the helper code in data_iterator.py will resize the copter images to 160x160x3 to speed up training` provided the best hint: although the training was speeded up by limiting the size of the images before training, it hinderend the capacity of the model to adequately reach a good score, especially during detection of the target from far away (see next section).

Therefore, changing such number from 160 to 256 improved both global precision and overfitting as the target could be better detected in the different evaluated scenarios. Accounting for 15 epochs and the other hyperparameters fixed the validity loss oscillated as shown in the graph below, with line `val_loss` with a much lesser distance between the peaks than the previous graph. The last was prove that the model pointed to the right direction given its closeness to the `train_loss` and the improved `val_loss` average along the epochs. 

![image1]

It could also be that the model was unable to reach an adequate global precision value in the first trials due to hardware limitations, but it is unknown if that's a plausible reason. 

### Predictions/IOU
The next impressions can be gathered from the project's implementation:
 

`Images while following the target`

![image3]

`Images while at patrol without target` 

![image4]

`Images while at patrol with target`

![image5]

1. There's a high risk of overfitting the model if number of features N > 50. This comes out after several attempts were made for N (10, 30, 50, 100, 250). As the number increased, the reported precision while runnning `train_svm.py` increased as well (even 96% reported), although no improvement was observed in the number of objects detected (as reported in RViz). Information in Slack helped in determinining under which conditions this situation would appear.  


### Final Thoughts

I consider this project to be manageable in the proposed time. Information provided via the Slack channel helped also in determining the errors that the code could have. In addition, the lectures provided good information regarding the CNN and FCN methods for image processing making this experience nicer than others. 
 

