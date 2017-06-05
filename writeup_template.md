## Project: Search and Sample Return


**The goals / steps of this project are the following:**  

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
[image4]: ./output/example_rock1.jpg 
[image5]: ./output/example_obstacle1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
I change the `color_thresh()` to set up the acceptable maximum RGB value. Here is an example output of thresholded rock image using the code below:
```
rock = color_thresh(warped, (90, 90, 0), (255, 255,72))
```

 ![alt text][image3] ![alt text][image4]
 
 And  for the obstacles:

```
 obstacles = color_thresh(warped, (0, 0, 0), (110, 110, 110))
```
  ![alt text][image3] ![alt text][image5]
  
  To better deal with the value 0 in RGB, I use the greater-than-and-equal sign in `color_thresh()` like below:
 
  ```
  img[:,:,1] >= rgb_min_thresh[1]
  ```
  
 But it introduces another problem that I will include unnecessary balck point for obstacle image if I do the transforming first.
 As a result, I choose to threshold the original image before the transforming.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And here are two example videos I made after changing the `process_image()`! 

* [Test mapping video](./output/test_mapping.mp4)
* [My test mapping video](./output/my_test_mapping.mp4)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
##### perception_step()
The `perception_step()` (line 85 to line 163) contains the perception pipeline which is developed from jupyter notebook. The generall steps to process navigable/obstacle/rock images include:
1. Perspect transform
2. Color threshold
3. Establish rover coordinate
4. Establish world coordinate
5. Highlight the sampled pixels in world map
6. Derive navigable angle

As metioned above, the `color_thresh()` is changed to accept value 0 in RGB and there is problem dealing with pure black points. As a result, I do a color threshold berfore perspect transform for obstacle images.
~~~
    obstacles = color_thresh(Rover.img, (0, 0, 0), (110, 110, 110))
    obstacles = perspect_transform(obstacles, source, destination)
~~~

##### decision_step()
The original logic inside the `decision_step()`  is enough to pass the basic rubics, but I want to try picking up some rock samples. So I set up a new state in the decision for seeing the rock sample and approach the sample slowly until I get near:
~~~
#When rock sample is saw, try to approach
elif Rover.mode == 'saw_sample':
    if len(Rover.rock_angles >= Rover.go_sample):
        if not Rover.near_sample:
            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -Rover.camera_vision, Rover.camera_vision)

            # Approaching slowly
            if Rover.vel > 5.0:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            else:
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
        else:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set

    elif not Rover.near_sample and not Rover.picking_up:
        Rover.throttle = Rover.throttle_set
        Rover.brake = 0
        Rover.steer = 0
        Rover.mode = 'forward'
~~~
The rover will pick the sample after then:
~~~
if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
	Rover.send_pickup = True
~~~

In some cases, I find out that the rover get stuck after picking up the rock sample. As a result, I introudce a 90-degree-turn in stop mode for struggling out these cases. The main idea is first to detect if the rover is being stuck for a long time:
~~~
# Detect if we are stuck in one place, try to struggle out by 90 degree turn in stop mode.
if Rover.vel <= Rover.stop_velocity_thresh / 2 and Rover.throttle_set == Rover.throttle_set:
    if Rover.stuck_count < Rover.stuck_thres:
        Rover.stuck_count += 1
    else:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.stuck_count = 0
        Rover.mode = 'stop'
~~~
And then the rover will try to turn around in stop mode:
~~~
def is_rotated_90(now_yaw, from_yaw):
    rotated = now_yaw - from_yaw
    if rotated >= 0:
        return rotated >= 90.0
    else:
        return rotated + 360.0 >= 90
        
if len(Rover.nav_angles) < Rover.go_forward or not is_rotated_90(Rover.yaw, Rover.stop_init_yaw):
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.steer = -15 # Could be more clever here about which way to turn
~~~

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

The simulator in my environment has following configuration:

* Resolution: 1024 X 768
* Quality: Good
* FPS: 17 - 20

For most of the scenarios, my rover can steer quite well and it can pick up several rock samples. But I find out that my rover will circle around a big plains in some cases. I think I can introduce a bias toward the obstacle on the left to sovle this, and this should give me the ability to navigate the whole map.