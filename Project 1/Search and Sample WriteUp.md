## Project: Search and Sample Return

**The goals / steps of this project were the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points 

#### Summary

The proposed project was developed following these steps:
1. All required software to properly deploy The Roversim simulator was installed and tested.  
2. I familiarized myself with the simulator commands and the steps required to save data in a specific folder.
3. The generated images were analized in order to identify terrain, obstacles and rocks in terms of a color threshold. 
4. Changes were implemented in the coding so perspective transformation, color thresholding, coordinates transformation and processing of images could be performed smoothly. 
5. The files `perception.py`, `drive_rover.py`, `decision.py` and `supporting_functions.py` were analized so the modifications previously performed on the `Rover_Project_Test_Notebook` were translated and adapted. 
6. Functions `perception_step()` and `decision_step()` were modified in order to generate an adequate response on the autonomous mode of the rover model.
7. It was possible to accomplish the project's minimum requirements and a logic for the rover to succesfully pick all the localized rocks.  Nevertheless, the design of appropiate loops for allowing the rover to choose a non-explored area and returning to its initial position after picking all the rocks, are pending in the attached documents. 
8. Conclusions are reocmmendations for improvement are included in the respective sections. 

### Notebook Analysis

The functions provided in the notebook were initially tested with test data provided in the repository. Path was also modified to match the that of the personal PC in order to properly random pick an image. 

For the identification of navigable terrain versus obstacles, a pick of (160, 160, 160) RGB pixels was initially considered. Other ranges were properly tested, but last numbers provided the best resolution in terms of light intensity reflection. Therefore, all pixels above this threshold were determined to be ground or navigable terrain, while the obstacles were all those below. 

Regarding rocks' identification, an image including a rock (e.g. `example_rock1.jpg`) was analized in jupyter notebook so the RGB pixel values could be visualized. Due to the different set of yellow combinations, the ranges that proved to be the most effective  for the detection of rocks were:
1. Between 110 and 215 pixels for the Red channel
2. Between 110 and 200 pixels for the Green channel
3. Between 5 and 50 pixels for the Blue channel. 

Keeping the above values in mind, and considering that the `color_thresh()` function had already some code written for ground detection, both obstacles' and rocks' detection included the same steps. That is, a matrix of zeros accounting for the size of the image was initally created; a boolean operation was then performed to define `True` or `1` values for the pixels whose position followed the mentioned criteria; and such positions would finally be considered for indexing the `True` values onto the zeros-filled matrices, detemining therefore which pixels would contribute for the generation of each color channel. 

The `color_thresh()` function returned a tuple of three elements all contained in `Threshed[0]`, `Threshed[1]` and `Threshed[2]`, for which a warped image was passed as an argument. The results are three images configured to display in white the results of applying the above thresholds to the three color channels of the example image. 

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And another! 

![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  



![alt text][image3]


