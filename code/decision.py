import numpy as np

def is_rotated_90(now_yaw, from_yaw):
    rotated = now_yaw - from_yaw
    if rotated >= 0:
        return rotated >= 90.0
    else:
        return rotated + 360.0 >= 90

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            if len(Rover.rock_angles >= Rover.go_sample):
                # Steering to rock sample and clipped to the camera vision range.
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -Rover.camera_vision, Rover.camera_vision)
                Rover.mode = 'saw_sample'

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -Rover.camera_vision, Rover.camera_vision)

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

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            if Rover.entering_stop:
                Rover.stop_init_yaw = Rover.yaw
                Rover.entering_stop = False

            # If we're in stop mode but still moving keep braking
            if Rover.vel > Rover.stop_velocity_thresh:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= Rover.stop_velocity_thresh:
                # Now we're stopped and we have vision data to see if there's a path forward
                # Introduce 90 degree turn in case that we are stuck.
                if len(Rover.nav_angles) < Rover.go_forward or not is_rotated_90(Rover.yaw, Rover.stop_init_yaw):
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -Rover.camera_vision, Rover.camera_vision)

                    Rover.entering_stop = True
                    Rover.mode = 'forward'

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

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover

