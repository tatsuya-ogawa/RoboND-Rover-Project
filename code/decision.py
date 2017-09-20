import numpy as np
import time
import random


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.mode != 'stop':
            # --------------------------------------------------------------------------
            # This will determine if the rover get stuck
            # Rover.stuck_time is a commulative frame counter that rover get stuck
            if (np.abs(Rover.vel) <= 0.05):
                Rover.stuck_time += 1
            # if it get can get out then fine, reset counter
            else:
                Rover.stuck_time = 0
            # if the rover get stuck, move backward in oppospite direction
            if Rover.stuck_time > Rover.error_limit:
                sign = random.choice([-1, -1, -1, -1, 1])
                Rover.throttle = sign * Rover.throttle_set
                if random.choice([True, False]):
                    Rover.steer = sign * np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                else:
                    Rover.steer = random.gauss(0, 15.0)
                time.sleep(random.uniform(1.0, 2.0))
                Rover.stuck_time = 0
        else:
            Rover.stuck_time = 0

        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if (len(Rover.nav_angles) >= Rover.stop_forward) and \
                    (np.max(Rover.nav_dists) >= 20):
                # If mode is forward, navigable terrain looks good
                # and velocity is below max and not turning, then throttle
                if (Rover.vel < Rover.max_vel) and (Rover.steer < 5):
                    # Set throttle value to throttle setting
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif (len(Rover.nav_angles) < Rover.stop_forward) or \
                    (np.max(Rover.nav_dists) < 20):
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                Rover.steer = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.5:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving then do something else
            elif Rover.vel <= 0.5:
                # Now we're stopped and there is no path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'forward'
        # If any sample detected, go to sample
        elif Rover.mode == 'goto_rock':
            # if the rover can pick up sample, stop and pick it up
            if Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                # if cannot pick up, but the rover is getting close to the sample, start to brake
            else:
                # indicator function
                indicator = int((Rover.max_vel / 2) >= Rover.vel and np.mean(Rover.nav_dists) >= Rover.vel)
                Rover.throttle = indicator * Rover.throttle_set / 2
                Rover.brake = (1 - indicator) * Rover.brake_set / 3

            # --------------------------------------------------------------------------
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            # if the sample is picked up, exit this mode
            if Rover.picking_up:
                Rover.mode = 'stop'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        # Enter mode 'stop' after picking up
        Rover.mode = 'stop'

    return Rover
