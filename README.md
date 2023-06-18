#PROJECT: SEARCH AND SAMPLE RETURN
-----


[//]: # (Image References)

[image1]: ./misc/image_n_warped.png
[image2]: ./misc/navi_bw.png
[image3]: ./misc/rock_bw.png
[image4]: ./misc/navi_arrow.png
[image5]: ./misc/rock_arrow.png 
[image6]: ./misc/auto_002.jpg
[image7]: ./misc/auto_800x600_001.jpg
[image8]: ./misc/auto_640x480_001.jpg
[image9]: ./misc/manual_mapped.jpg


# Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

* **Color Thresholding:** To identify what was of interest, the color was limited using ranges, from the central RGB to the maximum and minimum acceptable values. The same function is called giving the necessary parameters for its purpose each time.

Variables also used in perception.py
````
navi_range = 55
rock_range = 50
obst_range = 45
navi_thresh = (255-navi_range, 255-navi_range, 245-navi_range)
rock_thresh = (250-rock_range , 250-rock_range, 15+rock_range)
obst_thresh = (obst_range, obst_range, obst_range)
````
````
def color_thresh(img, rgb_thresh = rock_thresh, color_range = rock_range):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (np.logical_and(img[:,:,0] > rgb_thresh[0]-color_range, img[:,:,0] < rgb_thresh[0]+color_range))\
                & (np.logical_and(img[:,:,1] > rgb_thresh[1]-color_range, img[:,:,1] < rgb_thresh[1]+color_range)) \
                & (np.logical_and(img[:,:,2] > rgb_thresh[2]-color_range, img[:,:,2] < rgb_thresh[2]+color_range))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
threshed = color_thresh(warped)
plt.imshow(threshed, cmap='gray')
````

![alt text][image1] 

Selected image and Warped.

![alt text][image2] ![alt text][image3]

Thresholded images, navigable terrain and rock sample.

![alt text][image4] ![alt text][image5]

Navigable Terrain and Rock Sample

----

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

###process_image(img):

* Define source and destination points for perspective transform
````
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
````

* Apply perspective transform
````
    warped = perspect_transform(image, source, destination)
````
    
* Apply color threshold to identify navigable terrain/obstacles/rock samples.
````# RGB center values for ground, rocks and obstacles (walls)
    navi_thresh = (180, 180, 150)
    rock_thresh = (185, 185, 50)
    obst_thresh = (75, 60, 60)
    # Range for colors
    thresh_tolerance = 40
    # Applying color threshold
    navi_colorsel = color_thresh(warped, navi_thresh, thresh_tolerance)
    obst_colorsel = color_thresh(warped, obst_thresh, thresh_tolerance)
    rock_colorsel = color_thresh(warped, rock_thresh, thresh_tolerance)
````
    
* Convert thresholded image pixel values to rover-centric coords. 
````navixpix, naviypix = rover_coords(navi_colorsel)
    rockxpix, rockypix = rover_coords(rock_colorsel)
    obstxpix, obstypix = rover_coords(obst_colorsel)````

* Convert rover-centric pixel values to world coords. 
````scale = 10
    # Get navigable pixel positions in world coords
    navi_x_world, navi_y_world = pix_to_world(navixpix, naviypix, data.xpos[data.count], 
                                          data.ypos[data.count], 
                                          data.yaw[data.count], 
                                          data.worldmap.shape[0], 
                                          scale)
    rock_x_world, rock_y_world = pix_to_world(rockxpix, rockypix, data.xpos[data.count], 
                                          data.ypos[data.count], 
                                          data.yaw[data.count], 
                                          data.worldmap.shape[0], 
                                          scale)
    obst_x_world, obst_y_world = pix_to_world(obstxpix, obstypix, data.xpos[data.count], 
                                          data.ypos[data.count], 
                                          data.yaw[data.count], 
                                          data.worldmap.shape[0], 
                                          scale)
````   
    
* Update worldmap (to be displayed on right side of screen)
````data.worldmap[obst_y_world, obst_x_world, 0] = 255 #+= 1 #red
    data.worldmap[rock_y_world, rock_x_world, 1] = 255 #+= 1 #green
    data.worldmap[navi_y_world, navi_x_world, 2] = 255 #+= 1 #blue
````

* Make a mosaic image, below is some example code. 
````# First create a blank image (can be whatever shape you like)
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
        # Next you can populate regions of the image with various output
        # Here I'm putting the original image in the upper left hand corner        
    output_image[0:img.shape[0], 0:img.shape[1]] = img
        # Let's create more images to add to the mosaic, first a warped image
    warped = perspect_transform(img, source, destination)
        # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:] = warped
        # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
        # Flip map overlay so y-axis points upward and add to output_image 
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)
        # Then putting some text over the image
    cv2.putText(output_image,"trying text", (10, 10), 
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    if data.count < len(data.images) - 1:
        data.count += 1 # Keep track of the index in the Databucket()
    return output_image
```` 

### Video Capture

![alt text][image9]


---

# Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

###Perception Step
The same as the notebook, color thresholding is made by ranges. The same function `color_thresh(,,)` is used for terrain, walls and rock samples.
````
navi_range = 55
rock_range = 50
obst_range = 45
navi_thresh = (255-navi_range, 255-navi_range, 245-navi_range)
rock_thresh = (250-rock_range , 250-rock_range, 15+rock_range)
obst_thresh = (obst_range, obst_range, obst_range)
````

###Desicion Step

There are three modes: forward, stop and stumble.

* **Forward mode:** In the forward mode, the possibility of finding a rock is contemplated, taking into account the distance to which it is located and the speed of the rover. To avoid infinite loops and donuts in the course, the values of the steer are conditioned according to the speed, the rocks will always have priority in the desicion of the steer, except to enter stumble mode.
````
 if Rover.mode == 'forward':
            # Check stumble or dead end
            if Rover.vel <= 0 and len(Rover.nav_angles) > Rover.stop_forward  and Rover.throttle > 0:
                Rover.throttle = 0
                stumbletime = Rover.total_time
                Rover.mode = 'stumble'
            # Check navigable terrain without rock sample to Stop mode
            elif len(Rover.nav_angles) < Rover.stop_forward and len(Rover.rock_angles) == 0:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.lastangle = Rover.steer
                Rover.lastpos = Rover.pos
                Rover.mode = 'stop'
            # Checking rock sample exists
            elif len(Rover.rock_angles) >= 1:
                # Check distance to pick
                if rock_dist <= pick_dist:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.mode = 'stop'
                # Check distance to start approach
                elif pick_dist < rock_dist <= decelerate:
                    Rover.steer = rock_angle
                    # Check speed of the Rover
                    if Rover.vel <= limit_aprox:
                        Rover.throttle = Rover.throttle_set
                        Rover.brake = 0
                    elif limit_aprox < Rover.vel < Rover.max_vel:
                        Rover.throttle = 0
                        Rover.brake = Rover.vel
                    else:
                        Rover.throttle = 0 #rock_dist / decelerate
                        Rover.brake = Rover.brake_set
                # Check rock in sigth
                elif rock_dist > decelerate:
                    Rover.steer = np.clip(navi_angle + rock_angle, -Rover.steerlimit, Rover.steerlimit)
                    # Check speed of the Rover
                    if Rover.vel <= limit_far:
                        Rover.throttle = Rover.throttle_set
                        Rover.brake = 0
                    elif limit_far < Rover.vel < Rover.max_vel:
                        Rover.throttle = 0
                        Rover.brake = (Rover.vel * abs(rock_angle)) / Rover.steerlimit
                    else:
                        Rover.throttle = 0
                        Rover.brake = Rover.vel / rock_dist
            # End of Rock mode
            elif Rover.vel >= Rover.max_vel and len(Rover.rock_angles) < 1:
                Rover.throttle = 0
                Rover.steer = np.clip(navi_angle / (0.1 + Rover.vel), -caselimit, caselimit)
                Rover.lastangle = Rover.steer
                # saved to check stumble
                Rover.lastpos = Rover.pos
                if Rover.stop_forward < len(Rover.nav_angles) < Rover.go_forward:
                    Rover.brake = (Rover.vel / Rover.max_vel) * coef
                elif len(Rover.nav_angles) < Rover.stop_forward:
                    Rover.brake = Rover.brake_set
                else:
                    Rover.brake = 0
            else:
                Rover.steer = np.clip(((navi_angle+Rover.lastangle))/2, -caselimit, caselimit) #/ Rover.vel
                Rover.throttle = Rover.throttle_set + ((caselimit - abs(Rover.steer)) / (3* caselimit * coef))
                Rover.lastangle = Rover.steer
                Rover.brake = 0
````

* **Stop mode:** This comes when it finds a wall, when a rock is near or when leaving the stumble mode. As soon as there is a rock within reach, it is collected. When there is a wall, the rover turns to recover the path, just as when leaving the stumble mode.
````
        elif Rover.mode == 'stop':
            Rover.brake = 0
            # It's time to pick up rock sample
            if Rover.near_sample:
                # Check speed
                if Rover.vel <= 0:
                    # Check is picking up
                    if not Rover.picking_up:
                        Rover.send_pickup = True
                else:
                    Rover.brake = Rover.brake_set
            # Check walls or stumble        
            elif Rover.vel <= 0.2:
                # There is not navigable terrain
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    if len(Rover.rock_angles) > 0:
                        if navi_angle > 7:
                            Rover.steer = rock_angle
                        else:
                            Rover.throttle = -Rover.throttle_set
                            Rover.lastangle = -Rover.steer
                            Rover.mode = 'stumble'
                        Rover.lastangle = Rover.steer
                        Rover.throttle = Rover.throttle_set
                    elif Rover.lastangle != 0:
                        Rover.steer = 15 * np.sign(Rover.lastangle)
                    else:
                        Rover.steer = 15 * np.sign(navi_angle)
                        Rover.lastangle = Rover.steer
                # There is navigable terrain
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.steer = np.clip(navi_angle, -Rover.steerlimit, Rover.steerlimit)
                    Rover.throttle = Rover.throttle_set
                    # saved to check stumble
                    Rover.lastpos = Rover.pos
                    Rover.lastangle = Rover.steer
                    Rover.mode = 'forward'
            else:
                Rover.brake = Rover.brake_set
````

* **Stumble mode:** Stumble mode
````
        elif Rover.mode == 'stumble':
            Rover.throttle = 0
            Rover.steer = 15 * np.sign(navi_angle)
            sleep(1)
            Rover.mode = 'stop'
````

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

Code was generated just using image from center camera of the rover, other data like map, time, or state are no taked into account. 

There are no functions defined in this development,  decision is made in an only conditional block in decision_step(), some variables were created to help in decision and to make calculation just once at iteration.

**Some examples of the performance:**

* 1024x768 Performance

![alt text][image6] 

* 800x600 Performance

![alt text][image7]

* 640x480 Performance 

![alt text][image8]



**TODO List**

* Filter glitches on detecting rocks to steer the rover
* More soft braking, better formulas.
* Improve Stumble mode behavior
* Collect data for making decisions.
* Take advantage of more available data
* Use mapped
* Modules
