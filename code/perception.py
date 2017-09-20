import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
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
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image

    return warped


# Identify yellow rock sample
def color_thresh_rock(img):
    # yellow_hsv = [30,255,255] # H is 0-179 degree not 360
    # convert RGB image to HSV image
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # define lowerbound and upperbound for yellow color
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    # detect color in image by masking pixels
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    result = cv2.bitwise_and(img, img, mask=mask)
    # convert result to binary
    binary_result = color_thresh(result, (0, 0, 0))
    # return binary result
    return binary_result


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles


    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])

    # 2) Apply perspective transform to input image
    warped_navigable = perspect_transform(Rover.img, source, destination)
    warped_rock = warped_navigable

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed_navigable = color_thresh(warped_navigable)
    threshed_obstacle = 1 - threshed_navigable  # binary invert
    threshed_rock = color_thresh_rock(warped_rock)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] = threshed_obstacle * 255  # full red
    Rover.vision_image[:, :, 1] = threshed_rock * 255
    Rover.vision_image[:, :, 2] = threshed_navigable * 255  # full blue

    # -----------------------------------------------------------------------------------
    # Perspective transform is distorted when the distance (respect to the rover) is far away.
    # So only near vision will be added to world map (crop for inside vision)
    # This help increase Fidelity
    threshed_navigable_crop = np.zeros_like(threshed_navigable)
    threshed_obstacle_crop = np.zeros_like(threshed_obstacle)
    x1 = np.int(threshed_navigable.shape[0] / 2)  # index of start row
    x2 = np.int(threshed_navigable.shape[0])  # index of end row
    y1 = np.int(threshed_navigable.shape[1] / 3)  # index of start column
    y2 = np.int(threshed_navigable.shape[1] * 2 / 3)  # index of end column
    # crop from start to end row/column
    threshed_navigable_crop[x1:x2, y1:y2] = threshed_navigable[x1:x2, y1:y2]
    threshed_obstacle_crop[x1:x2, y1:y2] = threshed_obstacle[x1:x2, y1:y2]
    # -----------------------------------------------------------------------------------

    # 5) Convert map image pixel values to rover-centric coords
    # Full coordinates will use to find steering direction
    xpix_nav, ypix_nav = rover_coords(threshed_navigable)
    xpix_obs, ypix_obs = rover_coords(threshed_obstacle)
    xpix_rock, ypix_rock = rover_coords(threshed_rock)
    # Only the near-vision coordinates will be added to the world map
    # Adding full coordinate pixels will decrease Fidelity
    xpix_nav_crop, ypix_nav_crop = rover_coords(threshed_navigable_crop)
    xpix_obs_crop, ypix_obs_crop = rover_coords(threshed_obstacle_crop)

    # 6) Convert rover-centric pixel values to world coordinates
    xpix_world_nav, ypix_world_nav = pix_to_world(xpix_nav_crop, ypix_nav_crop, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                  Rover.worldmap.shape[0], scale=10)
    xpix_world_obs, ypix_world_obs = pix_to_world(xpix_obs_crop, ypix_obs_crop, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                  Rover.worldmap.shape[0], scale=10)
    xpix_world_rock, ypix_world_rock = pix_to_world(xpix_rock, ypix_rock, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                    Rover.worldmap.shape[0], scale=10)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # Update world map only if the rover is flat to the ground (pitch and roll ~ 0 +- 3 degrees) to have a correct vision
    if (np.float(np.abs(Rover.roll) % 360) <= 3) and (np.float(np.abs(Rover.pitch) % 360) <= 3):
        Rover.worldmap[ypix_world_obs, xpix_world_obs, 0] += 1
        Rover.worldmap[ypix_world_rock, xpix_world_rock, 1] += 1
        Rover.worldmap[ypix_world_nav, xpix_world_nav, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles

    # Set priority of picking up sample to be higher than navigating
    # if the sample is in vision, go pick it up first
    if (xpix_rock.any() or Rover.mode == 'goto_rock'):
        # Entering 'go-to-sample mode'
        if (Rover.mode != 'goto_rock'):
            Rover.mode = 'goto_rock'
        # if the sameple is in vision, set perception for navigating to the sample
        if (xpix_rock.any()):
            Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_rock, ypix_rock)
            Rover.see_rock_error = 0
        # sometimes might mistakenly see the rock
        # Rover.see_rock_error is a commulative frame counter that rover might mistakenly see the sample
        else:
            Rover.see_rock_error += 1;
        # if mistakenly enter 'goto_rock' mode, and no longer see the sample, exit this mode
        if Rover.see_rock_error > 100:
            Rover.mode = 'stop'
    # if do not see any rock, set perception for a normal navigation
    else:
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_nav, ypix_nav)

    return Rover
