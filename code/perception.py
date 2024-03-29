import numpy as np
import cv2


dst_size = 5 
bottom_offset = 6
scale = 20
navi_range = 55
rock_range = 50
obst_range = 45
navi_thresh = (255-navi_range, 255-navi_range, 245-navi_range)
rock_thresh = (250-rock_range , 250-rock_range, 15+rock_range)
obst_thresh = (obst_range, obst_range, obst_range)

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh, color_range):
    # Create an array of zeros same xy size as img, but single channel
    #color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    #above_thresh = (img[:,:,0] > rgb_thresh[0]) \
    #            & (img[:,:,1] > rgb_thresh[1]) \
    #            & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    #color_select[above_thresh] = 1
    # Return the binary image
    color_select = np.zeros_like(img[:,:,0])
    
    above_thresh = (np.logical_and(img[:,:,0] > rgb_thresh[0]-color_range, img[:,:,0] < rgb_thresh[0]+color_range))\
                & (np.logical_and(img[:,:,1] > rgb_thresh[1]-color_range, img[:,:,1] < rgb_thresh[1]+color_range)) \
                & (np.logical_and(img[:,:,2] > rgb_thresh[2]-color_range, img[:,:,2] < rgb_thresh[2]+color_range))
    color_select[above_thresh] = 1
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

    # 1) Define source and destination points for perspective transform

    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])    

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navi_colorsel = color_thresh(warped, navi_thresh, navi_range)
    obst_colorsel = color_thresh(warped, obst_thresh, obst_range)
    rock_colorsel = color_thresh(warped, rock_thresh, rock_range)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obst_colorsel * 255 #-thresholded binary image
    Rover.vision_image[:,:,1] = rock_colorsel * 255 #-thresholded binary image
    Rover.vision_image[:,:,2] = navi_colorsel * 255 #-thresholded binary image
    
    # 5) Convert map image pixel values to rover-centric coords
    navixpix, naviypix = rover_coords(navi_colorsel)
    rockxpix, rockypix = rover_coords(rock_colorsel)
    obstxpix, obstypix = rover_coords(obst_colorsel)
   
    # 6) Convert rover-centric pixel values to world coordinates
    navi_x_world, navi_y_world = pix_to_world(navixpix, naviypix, 
                                          Rover.pos[0], Rover.pos[1], 
                                          Rover.yaw, Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(rockxpix, rockypix, 
                                          Rover.pos[0], Rover.pos[1], 
                                          Rover.yaw, Rover.worldmap.shape[0], scale)
    obst_x_world, obst_y_world = pix_to_world(obstxpix, obstypix, 
                                          Rover.pos[0], Rover.pos[1], 
                                          Rover.yaw, Rover.worldmap.shape[0], scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obst_y_world, obst_x_world, 0] = 200 #red
    Rover.worldmap[rock_y_world, rock_x_world, 1] = 255 #green
    Rover.worldmap[navi_y_world, navi_x_world, 2] = 255 #blue

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    rover_polarcoords = to_polar_coords(navixpix, naviypix)
    Rover.nav_dists = rover_polarcoords[0]
    Rover.nav_angles = rover_polarcoords[1]
    rock_rel_polar = to_polar_coords(rockxpix, rockypix)
    Rover.rock_dists = rock_rel_polar[0]
    Rover.rock_angles = rock_rel_polar[1]
    obst_rel_polar = to_polar_coords(obstxpix, obstypix)
    Rover.obst_dists = obst_rel_polar[0]
    Rover.obst_angles = obst_rel_polar[1]

    return Rover