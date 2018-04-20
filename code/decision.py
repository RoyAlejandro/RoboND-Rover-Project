import numpy as np
from time import sleep


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Setting regions for dependent behaviour
    # Distance of the rock sample
    navi_angle = 0
    pick_dist = 10
    decelerate = 80
    
    limit_aprox = 0.8
    limit_far = Rover.max_vel/2
    # Calculating distance and angle of the rock sample
    rock_dist = np.mean(Rover.rock_dists)
    rock_angle = np.mean(Rover.rock_angles * 180/np.pi)
    obst_angle = np.mean(Rover.obst_angles * 180/np.pi)
    navi_angle = np.mean(Rover.nav_angles * 180/np.pi)
    coef = (len(Rover.nav_angles) / (1 * Rover.go_forward))
    stumbletime = Rover.total_time
    caselimit = Rover.steerlimit / (0.1+abs(Rover.vel))
    # Check if we have vision data to make decisions with
    Rover.lastpos = Rover.pos

    if Rover.nav_angles is not None: 
        # Check for Rover.mode status
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
        #Stop mode
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
        # Stumble mode
        elif Rover.mode == 'stumble':
            Rover.throttle = 0
            Rover.steer = 15 * np.sign(navi_angle)
            sleep(1)
            Rover.mode = 'stop'
        else:
            Rover.throttle = Rover.throttle_set
            Rover.mode = 'forward'
    # Just to make the rover do something 
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        Rover.roll=0

    
    return Rover

