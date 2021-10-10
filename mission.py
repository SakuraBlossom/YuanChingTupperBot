### Conventions ###
# Object Type - Our notation to determine the obstacle based on its colour
#
# 0 - Unknown Obstacle
# 1 - Obstacle (Red Colour Pole)
# 2 - Obstacle (Yellow Colour Pole)
# 3 - Obstacle (Green Colour Pole)
# 4 - Obstacle (Blue Colour Pole)
# 5 - Obstacle (Orange Colour Pole)

def getMyMissionPlans():

    '''
    Instead of write a single and (very) complex mission plan.
    We can break up the challenge into a sequence of simple task
    '''
    return [MyMissionPlanDemo1, MyMissionPlanDemo2], myDepthPID

def MyMissionPlanDemo1(my_auv):
    # EDIT ME!
    '''
    Brief: Scan left and right till we see the blue obstacle 
    '''
    
    # Not a good idea to start the mission immediately
    
    print("Waiting 5 secs before starting...")
    my_auv.sleep_ms(5000)           # sleep 5 secs
    
    # Must call this function everytime before we can read our sensor detections
    # This includes front camera detections, the auv's pose (X, Y and yaw)
    my_auv.updateSensorData()  
    
    # Loops the program until some bad happens or we time out
    # Let's repeat the following loop until the time_limit is reached
    # In this case, we keep looping for 10 secs. (current time + 10000 ms)
    time_limit = my_auv.time_ms() + 10000 
    while my_auv.is_ok and time_limit > my_auv.time_ms():
        
        # Scan from left to right for the blue obstacle
        
        print("Scan Left to Right")
        while not (235 < my_auv.yaw < 270):
        
            # Must call this function
            my_auv.updateSensorData()  
            
            # Reads detections from the front camera,
            # which consist of the obj_type and the theta angle (on the horizontal plane) to the object
            for obj_type, obj_theta_degs in my_auv.detections:
                # Only interested in finding a Blue pole!
                if obj_type == 4 and -20 < obj_theta_degs < 20: 
                    print("Found the blue pole! Current theta: " + str(obj_theta_degs) + " degs")
                    
                    # Stop XY thrustors and move onto the next mission plan by leaving this function
                    my_auv.setSpeedXY(0.0, 0.0)
                    
                    # we set the attribute 'task_success' to True to indicate means this part of the mission was successful
                    my_auv.task_success = True
                    return

            my_auv.setSpeedXY(0.15, -0.15)    # Turn Right
            pass
            
        # Same as above, but now we scan from right to left
        print("Scan Right to Left")
        while not (90 < my_auv.yaw < 135):
            my_auv.updateSensorData()  # Must call this function everytime before we can read our sensor detections
            for obj_type, obj_theta_degs in my_auv.detections:
                if obj_type == 4 and -20 < obj_theta_degs < 20: 
                    print("Found the blue pole! Current theta: " + str(obj_theta_degs) + " degs")
                    my_auv.setSpeedXY(0.0, 0.0)
                    my_auv.task_success = True
                    return

            my_auv.setSpeedXY(-0.15, 0.15)    # Turn Left
            pass
        

    # we set the attribute 'task_success' is false to indicate that our mission has failed.
    # Essentailly, we admit defeat here.
    my_auv.task_success = False
    pass


def MyMissionPlanDemo2(my_auv):
    # EDIT ME!
    '''
    Brief: Drives the AUV forwards and backwards
    '''
    
    # Not a good idea to start the mission immediately
    my_auv.sleep_ms(1000)           # sleep 1 sec
    
    print("Drive forwards")
    my_auv.setSpeedXY(1.0, 1.0)     # Drive forwards
    my_auv.sleep_ms(2000)           # sleep 2 secs
    
    print("Drive Backwards")
    my_auv.setSpeedXY(-1.0, -1.0)   # Drive backwards
    my_auv.sleep_ms(2000)           # sleep 2 secs

    my_auv.setSpeedXY(0.0, 0.0)     # Stop thrustors
    # End of our mission

    my_auv.task_success = True
    pass


### PID Settings for depth controller ###
# EDIT this to tune your depth movement
DEPTH_SET_POINT = 1.2
DEPTH_KP = 0.002
DEPTH_KI = 0      #No need for Integral as the simulation is ideal, no noise
DEPTH_KD = 0.01   #Improves stability
DEPTH_bias = -0.1 #To prevent the output being zero
DEPTH_error_prior = 0
DEPTH_integral = 0

def myDepthPID(auv_altitude, iteration_time):
    # EDIT ME!
    global DEPTH_error_prior, DEPTH_integral

    error = DEPTH_SET_POINT - auv_altitude
    DEPTH_integral = DEPTH_integral + (error*iteration_time)
    derivative = (error - DEPTH_error_prior)/iteration_time
    thrust = DEPTH_KP*error + DEPTH_KI*DEPTH_integral + DEPTH_KD*derivative + DEPTH_bias
    DEPTH_error_prior = error
    
    return thrust

