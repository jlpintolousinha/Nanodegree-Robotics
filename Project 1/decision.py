import numpy as np
import time


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with 
    if (Rover.nav_angles is not None):
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward: #Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set

                    if (Rover.throttle != 0 and Rover.vel == 0.0 and Rover.steer !=0) and Rover.vision_image[:,:,1].any() == False:
                        Rover.rock_in_the_middle = True
                    else:
                        Rover.rock_in_the_middle = False
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip((np.mean(Rover.nav_angles * 180/np.pi))*0.5, -15, 15)
                
                if Rover.samples_collected == 6 and Rover.samples_to_find == 0:
                    Rover.steer = np.clip(np.mean(Rover.starting_point_angle * 180/np.pi), -15, 15)     
					if Rover.pos == Rover.starting_point:
						Rover.throttle = 0
						Rover.brake = Rover.brake_set*0.5
						Rover.mode = 'stop'                                                 
            else:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set*0.5
                Rover.mode = 'stop'          
                
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            if (len(Rover.nav_angles) < Rover.stop_forward) or Rover.rock_in_the_middle == True:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.mode = 'stop' 
                                
            if Rover.rock_angles != 0 and Rover.rock_dists != 0 and Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set*0.1
                Rover.mode = 'rock' 

        elif Rover.mode =='rock':
            Rover.samples_located += 1
            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
            Rover.brake = 0
            Rover.throttle = Rover.throttle_set*0.5
            Rover.near = Rover.near_sample
                 
            if Rover.near == True:
                Rover.brake = Rover.brake_set*2
                Rover.throttle = 0
            else:
                if Rover.rock_angles > 0 :
                    Rover.brake = 0
                    Rover.steer = 15
                else:
                    Rover.brake = 0
                    Rover.steer = -15
                    
            if Rover.vision_image[:,:,1].any() == False:
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set
                Rover.steer = np.clip((np.mean(Rover.nav_angles * 180/np.pi))*0.85, -15, 15)  
                Rover.mode = 'forward'                 
                    
            #after collecting the rock, return to the initial state
            if Rover.samples_collected > 0 and Rover.picked_rock == True:
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set
                Rover.steer = np.clip((np.mean(Rover.nav_angles * 180/np.pi))*1.5, -15, 15)
                Rover.mode = 'forward'
                Rover.picked_rock = False
                               
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set*0.5
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if (len(Rover.nav_angles) < Rover.stop_forward) or Rover.rock_in_the_middle == True: 
                    Rover.throttle = Rover.throttle = Rover.throttle_set*-1
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!              
                if (len(Rover.nav_angles) >= Rover.go_forward):
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    Rover.rock_in_the_middle = False  
                
                if Rover.samples_collected == 6:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set*0.5
                    Rover.steer = 0 

					
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.picked_rock = True
                  
    print(Rover.picked_rock)
    print(Rover.rock_in_the_middle)
    print(Rover.pos)
    print(Rover.starting_point_dist,Rover.starting_point_angle)
    return Rover



