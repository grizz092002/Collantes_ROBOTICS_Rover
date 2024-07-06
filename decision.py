import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # offset in rad used to hug the left wall.
    offset = 0
    # Only apply left wall hugging when out of the starting point (after 10s)
    # to avoid getting stuck in a circle
    if Rover.total_time > 10:
        # Steering proportional to the deviation results in
        # small offsets on straight lines and
        # large values in turns and open areas
        offset = 0.8 * np.std(Rover.nav_angles)

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status. I made Rover.mode a stack
        if Rover.mode[-1] == 'forward':
            # if sample rock on sight (in the left side only) and relatively close
            if Rover.samples_angles is not None and np.mean(Rover.samples_angles) > -0.2 and np.min(
                    Rover.samples_dists) < 30:
                # Rover.steer = np.clip(np.mean(Rover.samples_angles * 180 / np.pi), -15, 15)
                Rover.rock_time = Rover.total_time
                Rover.mode.append('rock')

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # Except for start, if stopped means stuck.
                # Alternates between stuck and forward modes
                if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 4:
                    # Set mode to "stuck" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode.append('stuck')
                    Rover.stuck_time = Rover.total_time
                # if velocity is below max, then throttle
                elif Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                # Hug left wall by setting the steer angle slightly to the left
                Rover.steer = np.clip(np.mean((Rover.nav_angles + offset) * 180 / np.pi), -15, 15)

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward or Rover.vel <= 0:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode.append('stop')

            # If we're already in "stuck". Stay here for 1 sec
        elif Rover.mode[-1] == 'stuck':
            # if 1 sec passed go back to previous mode
            if Rover.total_time - Rover.stuck_time > 1:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                # Hug left wall by setting the steer angle slightly to the left
                Rover.steer = np.clip(np.mean((Rover.nav_angles + offset) * 180 / np.pi), -15, 15)
                Rover.mode.pop()  # returns to previous mode
            # Now we're stopped and we have vision data to see if there's a path forward
            else:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                # Since hugging left wall steering should be to the right:
                Rover.steer = -15

        elif Rover.mode[-1] == 'rock':
            # Steer torwards the sample
            mean = np.mean(Rover.samples_angles * 180 / np.pi)
            if not np.isnan(mean):
                Rover.steer = np.clip(mean, -15, 15)
            else:
                Rover.mode.pop()  # no rock in sight anymore. Go back to previous state

            # if 20 sec passed gives up and goes back to previous mode
            if Rover.total_time - Rover.rock_time > 20:
                Rover.mode.pop()  # returns to previous mode

            # if close to the sample stop
            if Rover.near_sample:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set

            # if got stuck go to stuck mode
            elif Rover.vel <= 0 and Rover.total_time - Rover.stuck_time > 10:
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode.append('stuck')
                Rover.stuck_time = Rover.total_time
            else:
                # Approach slowly
                slow_speed = Rover.max_vel / 2
                if Rover.vel < slow_speed:
                    Rover.throttle = 0.2
                    Rover.brake = 0
                else:  # Else break
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode[-1] == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    # Since hugging left wall steering should be to the right:
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    # Hug left wall by setting the steer angle slightly to the left
                    offset = 12
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi) + offset, -15, 15)
                    Rover.mode.pop()  # returns to previous mode

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    return Rover