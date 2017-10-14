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

[image1]: ./image_1.jpg
[image2]: ./image_2.jpg 
[image3]: ./image_3.jpg 
[image4]: ./image_4.jpg
[image5]: ./rock_img_warped.jpg

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

#### 1. The `Color_thresh()` funtion. 
The functions provided in the notebook were initially tested with data provided in the repository; initilially specified file path was modified to match that of the personal PC before attempting to pick an image at random. Because of the Notebook formatting, input and output line numbers will be provided throughout this section. 

For the identification of navigable terrain versus obstacles, a set of (160, 160, 160) RGB pixels was initially considered. Other ranges were properly tested, but last numbers provided the best resolution in terms of light intensity reflection. Therefore, all pixels above this threshold were determined to be ground or navigable terrain, while the obstacles were all those below. 

Regarding rocks' identification, an image including a rock (e.g. `example_rock1.jpg`) was analized in jupyter notebook so the RGB pixel values could be visualized. Due to the different set of yellow combinations, the ranges that proved to be the most effective  for the detection of rocks were:
1. Between 110 and 215 pixels for the Red channel
2. Between 110 and 200 pixels for the Green channel
3. Between 5 and 50 pixels for the Blue channel. 

Keeping the above values in mind, and considering that the `color_thresh()` function had already some code written for ground detection, both obstacles' and rocks' detection included the same steps. That is, a matrix of zeros accounting for the size of the image was initally created; a boolean operation was then performed to define `True` or `1` values for the pixels whose position followed the mentioned criteria; and such positions would finally be considered for indexing the `True` values onto the zeros-filled matrices, detemining therefore which pixels would contribute for the generation of each color channel. 

The `color_thresh()` function returned a tuple of three elements all contained in `Threshed[0]`, `Threshed[1]` and `Threshed[2]`, for which a warped image was passed as an argument. The results are three images configured to display in white the results of applying the above thresholds to the three color channels of the example image.

These profiles (ground + obstacles + rocks) all together created the filter to be applied to the images perceived/generated by the Rover's camera so all three elements could be processed in the `process_image()` function. On the other hand, a so called 'mask' was provided to limit the obstacles' displayed area to that perceived by the Rover. As a matter of fact, if it is not used, the image perceived by the Rover's camera will include whole red background that is not a real depiction of the visualized environment. 

![alt text][image1]

#### 2. The `process_image()` function  
Provided that the `color_thresh()` function filtered all the perceived images properly, the idea of where the Rover could drive to was considered next. Assume that for instance all the pixels were filtered and a continuous image of `navigable terrain` was generated. This data would then have to be tranlated into a worldmap such that ground, obstacles and rocks are visually available to determine which path the rover followe din the long run. 

I such sense of ideas, the `process_image()` function was populated with several functions that:
1. Defined both source and destination for the perspective transform to be generated.
2. Applied the perspective transform to the saved images while keeping both source and destination as working parameters, further saving its results to a local variable in line 108 of perception.py file. 
3. `color_threshold()` function received this local variable as argument to identify navigable terrain, obstacles and rocks, which was achievable due to the ranges imposed for the 3 different areas. Recall that the function was modified so a tuple of three elements was returned by the function after it ran. As an example, the next figuras include navigable terrain, obstacles and a rock sample to be picked.   

![alt text][image2]

4. The filtered images pixels were then translated into Rover coordinates. As the Rover is the only source of information, each pixel position needs to be properly calculated, and this applies to the three mentoned areas: navigable space, obstacles and rocks.  
5. All pixels were then rotated/translated into world's frame of reference. That is, we know where they're from Rover's perspective, but in order to get them into the real world image, a conversion of coordinates is accomplished by using Rover's stored parameters of `(x,y)` posion and orientation (yaw).  
6. `Worldmap` was then updated as the Rover moved along its path. That is, knowing the pixel's position at each time step (at each stored image) allows to associate the intensity to each of the color channels to such positions in the map. Blue (navigable) areas were defined to be more bright than the red ones (obstacles) per each time step. 

A video of around eight seconds was constructed with the data gathered during the recording session. As expected, colored areas in the map changed as per the Rover's direction, due to the population of such regions on the map with those of the output data.

### Autonomous Navigation and Mapping

#### 1. The `perception_step()` and `decision_step()`
These functions respectively included in perception.py and decision.py were modified to implement the autonomous mode in the simulation. The code described in the provious section was included in perception.py, while a decision-making process was written in decision.py in the form of a flow-control methodology. 

Regarding perception.py, two differences stand out:

1. Rover's position is provided at every time step by Rover.pos. This class element is included from lines 124 to 126 when both rotation and translation of rover-centric pixel values to world coordinates are calculated. This process is carried for all three elements present in the Rover's path (navigable terrain, obstacles and rocks) so both distance and angle relative to the Rover is calulated per every time step. 

2. Current Rover's angle is calculated and updated in line 135 every time step. Because this value is the main trigger for the flow-control methodology to start (as included in line 15 of decision.py file), it is observable from the very beginning that, even though `decision_step()` has not been modified, the Rover will start to navigate through the brightest pixels while avoiding obstacles (i.e. walls).

Once the flow-control triggers, the modifications introduced in decision.py play a major role: `decision_step()` function allows to differentiate whether the Rover's moving forward (`forward mode`in line 17), or if it finds a rock to pick (`rock mode`in line 59), or if can't move forward anymore (`stop mode` in line 92). 

In `forward` mode, the Rover will identify (according to image perception) whether the length of the navigation angles' array is higher than a defined threshold (`Rover.stop_forward`). It will compare the Rover's velocity to zero in order to set accelerate (via `Rover.set_throttle_set`) and set the bakes to zero. If the number of pixels for ground or navigable terrain are less than the mentioned threshold, the code is set for the Rover to stop accelerating, apply the brakes and switch to `stop` mode. In addtion, three intermediate decision logics were included:
1. To detect sparsed obstacles during navigation (see line 26), the Rover checks if its trajectory is blocked and it is not longer possible to move forward, in which case it switches to `stop` mode. The reporting variable `Rover.rock_in_the_middle` was defined to account for this scenario in order to keep a better control over the simulation. 
2. If rocks are available to pick up, the Rover's camera will show them on the screen (see line 145 in perception.py file), the Rover will stop accelerating, make a stop and then switch to `rock mode`. 
3. If all the samples have been collected and there're no more amples to find, the Rover will steer towards the angle fed by `Rover.starting_point_angle` (defined in line 165 of perception.py). Subsequently, once Rover's position matches the point recorded at the beginning of the simulation, the Rover will switch to `stop` and no further action will occur. 

In `rock` mode, the Rover will add one to the `Rover.samples_located` counter and steer towards the rock shown in the screen (the relative angle of the rock is calculated in line 146 of perception.py). As the Rover gets closer to the sample, the `picking_up` command (line 131) required for the Rover's arm to move will be sent once `Rover.near_sample` is True. However, it may happen that while getting closer to the sample, its angle is not small enough to make the variable True, in which case the Rover will steer 15 or -15 degrees depending on the angle's sign. Once the rock is picked, the Rover will accelerate, steer to the navigable zone and switch to `forward mode` (nonetheles, if all the rock samples have been collected, the Rover will be set to `forward` mode in order to reach the point were the simulation started (defined as `Rover.starting_point` in line 87 of `drive_rover.py` file) and eventually stop once the point is reached).  

In `stop`mode, the speed value will be compared to 0.2 to either determine if the Rover's still moving. If True, brakes will be further applied (0.5 gain in line 96) in order for the Rover to stop. Once stopped, image processing will determine if navigation angles' array length is lower than the `Rover.stop_forward` thershold (line 101), or if on the contrary it is higher than the `Rover.go_forward` element (line 108). The former statement allows to figure out if the Rover reached a dead end (see image below), while the latter provides feedback whether it is possible to continue its journey (and this applies also to the case where `Rover.rock_in_the_middle` was previously set to True). 

![alt text][image3]

#### 2. Results
Autonomous mode was ran using a 800 x 600 screen resolution (windowed) with good graphics quality, at a rate of 15 FPS. The simulation turned out to accomplish the requirements of mapping and fidelity. The rover is also capable of finding samples and pick them up as it gets close ot it. However, the model include certain drawbacks that are worth to mention:

1. Rock samples that although observed in eyes of the user, may not be detected by the Rover's camera. That been said, scaling the rocks so the are bigger in relation to the perpective image would solve the issue in the long run so the Rover doesn't spend much time looking for the samples. Current perpective transform configuration provides an image of the rock like the one below:

![alt text][image5]

2. Although the Rover steer to according to the mean angle formed by the navigable pixels, there's no logic implemented for discerning whether a zone in the map has been previously explored. For instance, in case two zones were equally navigable, a human operator would have to determine which zone is better for the Rover to go given certain conditions like number of samples found, zones in the map left, etc. 

![alt text][image4]

3. It may not possible for the Rover to keep moving in case a big obstacle blocks its path (i.e. a big terrain rock). The logic implemented on line 26 of decision.py will work 50% to 60% of the time it if is obvious that the Rover being block from going forward; however, if the edges of these obstacles are no smooth enough, the perspective transform won't be adequate to actually determine that the `Rover.stop_forward` thershold has been outstripped. In this case, the operator would have to manually take the Rover out the blockage for it to move. 

4. The logic implemented to return to the original point of departure (line 36 of decision.py) will not work if the Rover is very far from it. That is, the Rover will steer to the angle formed by the middle point of the map (which is fixed per definition) and the origin (0,0), and it may loop indefinitely between the `forward` and the `stop` mode if it hits an obstacle during its way. As it is now, the Rover

Different techniques could be useful to improve this model even further:

1. Define a cost function to differentiate the 'unknown' navigable terrain from that already explored. In this case, the Rover would follow the path where such function is mimimized in time, as the `Rover.worldmap` elements gets increased in the Red channel per explored pixels.

2. Steering/thrust values could be adaptable in terms of the Rover's current situation. That is, if the Rover gets stuck, it could easily accelerate further and return to its original value after the situation is solved. Adaptative control techniques could be used for this model, but it may require an effort that is out of the scope for this project. 

3. A GPS could be implemented as well in order to better compare Rover's current position regarding the worldmap, with the original point of departure. In other words, position of the rocks samples could be regarded as waypoints to be followed in terms of the Rover's current position.    