import sys, math, time, threading, random
import numpy as np
import argparse
import pygame



### World settings ###
# 0 - Unknown Obstacle
# 1 - Obstacle (Red Colour Pole)
# 2 - Obstacle (Yellow Colour Pole)
# 3 - Obstacle (Green Colour Pole)
# 4 - Obstacle (Blue Colour Pole)
# 5 - Obstacle (Orange Colour Pole)

GAME_RANDOM_OFFSET_GATE     = random.randrange(-10, 10) * 0.5
GAME_RANDOM_OFFSET_OBSTACLE = random.randrange(-6, 6) * 0.5

GAME_WORLDS =[

    # World 1 - Actual Gate Task
    ("YAY! We passed through the gate!",[
        {"type" : None, "goal_id": 0, "rel" : None, "colour" : None,        "world" : (-0.1, 9.85) },
        {"type" : None, "goal_id": 0, "rel" : None, "colour" : None,        "world" : (0.1, 9.85) },
        {"type" : None, "goal_id": 1, "rel" : None, "colour" : None,        "world" : (-0.1, 10.15) },
        {"type" : None, "goal_id": 1, "rel" : None, "colour" : None,        "world" : (0.1, 10.15) },
    
        {"type" : 1, "goal_id": None, "rel" : None, "colour" : (255,0,0),   "world" : (0, 5) },
        #{"type" : 1, "goal_id": None, "rel" : None, "colour" : (218,165,32), "world" : (0, 10)},
        {"type" : 1, "goal_id": None, "rel" : None, "colour" : (255,0,0),   "world" : (-0.75, 10) },
        {"type" : 3, "goal_id": None, "rel" : None, "colour" : (0,255,0),   "world" : (0.75, 10) },
    ]),

    # World 2 - Qualifiers
    ("YAY! We passed qualifiers!",[
        {"type" : None, "goal_id": 0, "rel" : None, "colour" : None,        "world" : (-0.1, 9.85) },
        {"type" : None, "goal_id": 0, "rel" : None, "colour" : None,        "world" : (0.1, 9.85) },
        {"type" : None, "goal_id": 1, "rel" : None, "colour" : None,        "world" : (-0.1, 10.15) },
        {"type" : None, "goal_id": 1, "rel" : None, "colour" : None,        "world" : (0.1, 10.15) },
    
        {"type" : 1, "goal_id": None, "rel" : None, "colour" : (255,0,0),   "world" : (-0.75, 10) },
        {"type" : 3, "goal_id": None, "rel" : None, "colour" : (0,255,0),   "world" : (0.75, 10) },
    ]),
    
    # World 3 - Training Ground 1
    ("YAY! We hit the target!",[
        {"type" : 4, "goal_id": 1, "rel" : None, "colour" : (0,0,255),   "world" : (-4, 0) }
    ]),
    
    # World 4 - Training Ground 2 with Random positions
    ("YAY! We hit the target!",[
        {"type" : 4, "goal_id": 1,    "rel" : None, "colour" : (0,0,255),   "world" : (GAME_RANDOM_OFFSET_GATE, 10) },
        {"type" : 1, "goal_id": None, "rel" : None, "colour" : (255,0,0),   "world" : (GAME_RANDOM_OFFSET_OBSTACLE, 5) },
    ]),
    
    # World 5 - Actual Gate Task with Random positions
    ("YAY! We passed through the gate!",[
        {"type" : None, "goal_id": 0, "rel" : None, "colour" : None,        "world" : (GAME_RANDOM_OFFSET_GATE-0.1, 9.85) },
        {"type" : None, "goal_id": 0, "rel" : None, "colour" : None,        "world" : (GAME_RANDOM_OFFSET_GATE+0.1, 9.85) },
        {"type" : None, "goal_id": 1, "rel" : None, "colour" : None,        "world" : (GAME_RANDOM_OFFSET_GATE-0.1, 10.15) },
        {"type" : None, "goal_id": 1, "rel" : None, "colour" : None,        "world" : (GAME_RANDOM_OFFSET_GATE+0.1, 10.15) },
    
        {"type" : 1, "goal_id": None, "rel" : None, "colour" : (255,0,0),   "world" : (GAME_RANDOM_OFFSET_GATE-0.75, 10) },
        {"type" : 3, "goal_id": None, "rel" : None, "colour" : (0,255,0),   "world" : (GAME_RANDOM_OFFSET_GATE+0.75, 10) },
        
        {"type" : 1, "goal_id": None, "rel" : None, "colour" : (255,0,0),   "world" : (GAME_RANDOM_OFFSET_OBSTACLE, 5) },
    ]),
]


###=============== Game settings ===============###

COLLISION_THRESHOLD = 0.2

# Create width and height constants
WINDOW_WIDTH  = 600
WINDOW_HEIGHT = 800
WINDOW_RATIO_AERIAL_FRONT_VIEW = 0.6

AERIAL_RADIUS = 7
CAMERA_FOV_RADS = np.deg2rad(90)
AERIAL_FOV_RADIUS = 5

PLAYER_FWD_STEP_ENU  = 0.1
PLAYER_SIDE_STEP_ENU = 0.1
PLAYER_YAW_STEP_ENU  = 1 # in degs

GAME_AREA_POOL_DEPTH  = 1.8 # in metres
GAME_AREA_POOL_CRASH  = 0.8 # in metres
GAME_AREA_POOL_LIMIT  = 1.6 # in metres

GAME_TILE_SCALE = 6
GAME_AREA_TILE_HEIGHT = GAME_TILE_SCALE * 0.240
GAME_AREA_TILE_WIDTH  = GAME_TILE_SCALE * 0.115

GAME_CURRENT_WORLD = 0

GAME_CONTROL_DEPTH_RESPONSE = 0.5
GAME_CONTROL_FWD_RESPONSE  = 0.02
GAME_CONTROL_SIDE_RESPONSE = 0.02
GAME_CONTROL_YAW_RESPONSE  = 0.02

###=============== Mission Interface Functions ===============###

class AUVInterface:
    def __init__(self, gama_sim):
        self.gama_sim = gama_sim
        
        self.is_ok = True
        self.depthController = None
        self.last_depth_update_tick = 0

        # Our AUV position
        self.x = 0        # Current X position
        self.y = 0        # Current Y position
        self.yaw = 0      # Current Yaw angle. North is 0 degs; +ve is clockwise
        self.alitude = 0  # Current altitude from the ground
        self.numTiles = 0 # Number of tiles observed

        # Thrustor Settings
        self.thrustor_down_id  = 0
        self.thrustor_left_id  = 1
        self.thrustor_right_id = 2
        self.thrustor_down_fwd_max  = 255
        self.thrustor_left_fwd_max  = 255
        self.thrustor_right_fwd_max = 255

        self.detections = list()


    def updateSensorData(self):
        '''
        Brief: Decodes the latest sensor information obtained from the bottom and front camera.
               Always call this function to decode the latest sensor readings
        '''
        self.x = self.gama_sim.global_x
        self.y = self.gama_sim.global_y
        self.yaw = self.gama_sim.global_yaw
        self.altitude = self.gama_sim.global_altitude
        
        self.detections.clear()
        for obj in self.gama_sim.objs:
            if obj["type"] is None:
                continue

            obj_theta = math.atan2(obj["rel"][0], obj["rel"][1])
            if (0 < obj["rel"][1] <= AERIAL_FOV_RADIUS and 2*abs(obj_theta) < CAMERA_FOV_RADS):
                self.detections.append((obj["type"], int(np.rad2deg(obj_theta))))


    def setSpeedXY(self, thrust_left : float, thrust_right : float):
        '''
        Brief: Sets the thrustor speeds of the AUV. The thrustors run asynchorously from the program
        Param thrust_left:  The normalised thrust speed (-1.0 to 1.0) for the left thrustor. +ve is forwards
        Param thrust_right: The normalised thrust speed (-1.0 to 1.0) for the right thrustor. +ve is forwards
        '''
        thrust_left  = np.clip(thrust_left, -1.0, 1.0)
        thrust_right = np.clip(thrust_right, -1.0, 1.0)

        thrust_side_enu = 0
        thrust_fwd_enu  = 0.5*(thrust_left+thrust_right)*PLAYER_FWD_STEP_ENU
        thrust_turn_enu = (thrust_right-thrust_left)*PLAYER_YAW_STEP_ENU

        self.gama_sim.updateCmdContinuous(thrust_fwd_enu, thrust_side_enu, thrust_turn_enu)
        pass

    def blinkLed(self, wait_time : float):
        '''
        Brief: Turns on and Off Led for wait_time
        Param wait_time: Sleep duration in seconds
        '''
        time.sleep(wait_time)
        pass

    def time_ms(self):
        '''
        Brief: Gets the time elapsed from the initialisation of the mission plan.
        Returns: An integer representing the time elapsed (in milli-seconds) 
        '''
        return int(self.gama_sim.time_diff * 1000)

    def sleep_ms(self, time_ms : int):
        '''
        Brief: Sleeps for the given time duration
        Param time_ms: Sleep duration in milli-seconds
        '''
        time.sleep(time_ms / 1000)
        pass


###=============== Drawing helper Functions ===============###

def modulus(numerator, divisor):
    val = numerator / divisor
    return val - int(val)

def denotation(numerator, divisor):
    val = int(numerator / divisor)
    return val*divisor

def draw_rect_alpha(surface, color, rect):
    shape_surf = pygame.Surface(pygame.Rect(rect).size, pygame.SRCALPHA)
    pygame.draw.rect(shape_surf, color, shape_surf.get_rect())
    surface.blit(shape_surf, rect)

def draw_circle_alpha(surface, color, center, radius):
    target_rect = pygame.Rect(center, (0, 0)).inflate((radius * 2, radius * 2))
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.circle(shape_surf, color, (radius, radius), radius)
    surface.blit(shape_surf, target_rect)

def draw_polygon_alpha(surface, color, points):
    lx, ly = zip(*points)
    min_x, min_y, max_x, max_y = min(lx), min(ly), max(lx), max(ly)
    target_rect = pygame.Rect(min_x, min_y, max_x - min_x, max_y - min_y)
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.polygon(shape_surf, color, [(x - min_x, y - min_y) for x, y in points])
    surface.blit(shape_surf, target_rect)

def draw_vectical_gradient_rect(window, top_colour, bottom_colour, target_rect):
    """ Draw a horizontal-gradient filled rectangle covering <target_rect> """
    colour_rect = pygame.Surface(( 2, 2 ))                                      # tiny! 2x2 bitmap
    pygame.draw.line(colour_rect, top_colour,  ( 0,0 ), ( 1,0 ) )               # left colour line
    pygame.draw.line(colour_rect, bottom_colour, ( 0,1 ), ( 1,1 ) )             # right colour line
    colour_rect = pygame.transform.smoothscale(colour_rect, (target_rect.width, target_rect.height))  # stretch!
    window.blit(colour_rect, target_rect)


###=============== PyGame Simulator Functions ===============###

class SimpleAUVSimulatorGame:
    def __init__(self, game_window, depthPIDController, isPerfectControl=False):
        self.aerial_radius = AERIAL_RADIUS

        self.restart()

        self.grid = list()
        self.camera_tan_val = math.tan(CAMERA_FOV_RADS / 2.0)
        self.depth_PID_controller = depthPIDController
        self.depth_controller_last_tick = time.time()

        aerial_window_h = int(WINDOW_RATIO_AERIAL_FRONT_VIEW * WINDOW_HEIGHT)
        sub_window_h = WINDOW_HEIGHT - aerial_window_h - 10
        sub_window_w = (WINDOW_WIDTH // 2) - 5

        self.view_main_window = game_window
        self.view_aerial  = pygame.Surface((WINDOW_WIDTH, aerial_window_h))
        self.view_frontal = pygame.Surface((sub_window_w, sub_window_h))
        self.view_plot    = pygame.Surface((sub_window_w, sub_window_h))

        # init variables for front view
        self.view_frontal_base = None
        self.font_small = pygame.font.Font(pygame.font.get_default_font(), 16)
        self.font_med = pygame.font.Font(pygame.font.get_default_font(), 25)
        self.font_large = pygame.font.Font(pygame.font.get_default_font(), 35)
        
        # Features
        self.isPerfectControl = isPerfectControl

    def restart(self):
        global GAME_CURRENT_WORLD

        self.global_x = 0
        self.global_y = 0
        self.global_yaw = 0
        self.global_altitude = GAME_AREA_POOL_DEPTH

        self.plot_path = [(0,0), (0,0)]

        self.game_win_phrase = GAME_WORLDS[GAME_CURRENT_WORLD][0]
        self.objs = GAME_WORLDS[GAME_CURRENT_WORLD][1]
        type_id_max = [obj["goal_id"] for obj in self.objs if obj["goal_id"] is not None]
        assert len(type_id_max) > 0, "GAME World must have atleast one goal"
        
        self.goals_achieved = [False for _ in range(max(type_id_max)+1)]

        self.time_diff = 0
        self.start_time = time.time()
        self.view_display_text = None
        self.view_display_subtext = None
        
        self.last_cmd_fwd   = 0
        self.last_cmd_side  = 0
        self.last_cmd_yaw   = 0
        self.last_cmd_depth = 0

        # For Imperfect controls
        self.vec_fwd = 0
        self.vec_side = 0
        self.vec_yaw = 0


    ###=============== Game Logic Functions ===============###
    def updateObjectPositions(self, auv_tf : np.array):
        # Update Obj
        for obj in self.objs:
            obj_pos = auv_tf @ np.array([obj["world"][0], obj["world"][1], 1])
            obj["rel"] = list(obj_pos[:2])

        # Update grid
        rot_tf = np.zeros((6,6))
        rot_tf[0:3,0:3] = auv_tf
        rot_tf[3:6,3:6] = auv_tf

        pos_x_start = self.global_x - 1.8 * self.aerial_radius
        pos_y_start = self.global_y - 1.8 * self.aerial_radius

        pos_x_end = self.global_x + 1.8 * self.aerial_radius
        pos_y_end = self.global_y + 1.8 * self.aerial_radius

        pos_x_start = denotation(pos_x_start, GAME_AREA_TILE_WIDTH)
        pos_x_end   = denotation(pos_x_end,   GAME_AREA_TILE_WIDTH)
        pos_y_start = denotation(pos_y_start, GAME_AREA_TILE_HEIGHT)
        pos_y_end   = denotation(pos_y_end,   GAME_AREA_TILE_HEIGHT)

        grid_temp = [[px, pos_y_start, 1, px, pos_y_end, 1] for px in np.arange(pos_x_start, pos_x_end, GAME_AREA_TILE_WIDTH)]
        grid_temp.extend([[pos_x_start, py, 1, pos_x_end, py, 1] for py in np.arange(pos_y_start, pos_y_end, GAME_AREA_TILE_HEIGHT)]) 
       
        self.grid = (rot_tf @ np.array(grid_temp).T).T # Numpy hack
 

    def updateMovement(self, rel_fwd_enu : float, rel_side_enu : float, rel_turn_enu : float):

        # Calculate global XY pose
        rel_fwd_enu     = np.clip(rel_fwd_enu,  -PLAYER_FWD_STEP_ENU,  PLAYER_FWD_STEP_ENU)
        rel_side_enu    = np.clip(rel_side_enu, -PLAYER_SIDE_STEP_ENU, PLAYER_SIDE_STEP_ENU)
        rel_yaw_enu     = np.clip(rel_turn_enu, -PLAYER_YAW_STEP_ENU,  PLAYER_YAW_STEP_ENU)
        
        if not self.isPerfectControl:
            self.vec_fwd  = GAME_CONTROL_FWD_RESPONSE*rel_fwd_enu   + (1-GAME_CONTROL_FWD_RESPONSE)*self.vec_fwd
            self.vec_side = GAME_CONTROL_SIDE_RESPONSE*rel_side_enu + (1-GAME_CONTROL_SIDE_RESPONSE)*self.vec_side
            self.vec_yaw  = GAME_CONTROL_YAW_RESPONSE*rel_yaw_enu   + (1-GAME_CONTROL_YAW_RESPONSE)*self.vec_yaw
            
            rel_fwd_enu  = self.vec_fwd
            rel_side_enu = self.vec_side
            rel_yaw_enu  = self.vec_yaw


        self.global_yaw = (self.global_yaw+rel_yaw_enu) % 360

        yaw_ang_rads = np.deg2rad(self.global_yaw)
        cos_val = math.cos(yaw_ang_rads)
        sin_val = math.sin(yaw_ang_rads)

        self.global_x += cos_val * rel_side_enu - sin_val * rel_fwd_enu
        self.global_y += sin_val * rel_side_enu + cos_val * rel_fwd_enu

        invert_x = cos_val  * -self.global_x + sin_val * -self.global_y
        invert_y = -sin_val * -self.global_x + cos_val * -self.global_y
        
        # Update Depth
        if self.depth_PID_controller:
            tick_diff = (time.time() - self.depth_controller_last_tick)
            thrust_depth = np.clip(self.depth_PID_controller(self.global_altitude, tick_diff), -1.0, 1.0)
            self.depth_controller_last_tick = time.time()
            

        # Update movement plot
        prev_pos = self.plot_path[-1]
        if ((prev_pos[0]-self.global_x)**2+(prev_pos[1]-self.global_y)**2) > 0.04:

            self.plot_path.append((self.global_x, self.global_y))

            if len(self.plot_path) > 2 and abs(self.plot_path[-1][0] - self.plot_path[-2][0]) < 0.01 and abs(self.plot_path[-1][0] - self.plot_path[-3][0]) < 0.01 \
                and abs(self.plot_path[-1][1] - self.plot_path[-3][1]) < abs(self.plot_path[-1][1] - self.plot_path[-2][1]):
                del self.plot_path[-2]

            if len(self.plot_path) > 2 and abs(self.plot_path[-1][1] - self.plot_path[-2][1]) < 0.01 and abs(self.plot_path[-1][1] - self.plot_path[-3][1]) < 0.01 \
                and abs(self.plot_path[-1][0] - self.plot_path[-3][0]) < abs(self.plot_path[-1][0] - self.plot_path[-2][0]):
                del self.plot_path[-2]



        # Return world in baselink frame
        return np.array([[cos_val, sin_val, invert_x],
                        [-sin_val, cos_val, invert_y],
                        [0, 0, 1]])


    def checkEndCondition(self):
        if self.hasEpochEnded():
            return

        # Check for any collision with obstacles
        for obj in self.objs:
            if ((obj["world"][0] - self.global_x)**2 + (obj["world"][1] - self.global_y)**2) > (COLLISION_THRESHOLD*COLLISION_THRESHOLD):
                continue

            if obj["goal_id"] is not None:
                for my_id in range(obj["goal_id"]+1, len(self.goals_achieved)):
                    self.goals_achieved[my_id] = False
                self.goals_achieved[obj["goal_id"]] = True
                continue

            self.displayResult("OH NO. We crashed!", (255, 75, 56))

        # Check if goal is achieved
        if all(self.goals_achieved):
            self.displayResult(self.game_win_phrase, (22, 160, 133))


    def getTimeElapsed(self):
        time_mins = int(self.time_diff // 60)
        time_secs = self.time_diff - time_mins*60
        return f"Time: {time_mins:02d}:{time_secs:0>5.2f}"
        
    def hasEpochEnded(self):
        return (self.view_display_text is not None)

    def displayResult(self, text, text_colour, subTitle="Press 'R' to restart."):
        if self.view_display_text is not None:
            return

        print(text)
        print(self.getTimeElapsed())
        self.view_display_text = self.font_large.render(text, True, text_colour, (149, 165, 166))
        self.view_display_subtext = self.font_med.render(subTitle, True, (0,0,0))
        pass



    ###=============== Sub Window Drawing Functions ===============###
    def updateAerialView(self, auv_tf : np.array) -> None:
        win_width  = self.view_aerial.get_width()
        win_height = self.view_aerial.get_height()
        half_w = (win_width // 2)
        half_h = (win_height // 2)
        win_cx = half_w
        win_cy = half_h

        scale_factor = float(half_h) / self.aerial_radius

        # Draw Window
        pygame.draw.rect(self.view_aerial, (173, 216, 230), self.view_aerial.get_rect(), 0, 10)
       
        # Draw Grid (Vertical)
        grid_temp = scale_factor * self.grid
        for px1, py1, _, px2, py2, _ in grid_temp:
            pygame.draw.line(self.view_aerial, (0,0,0), (win_cx+px1, win_cy-py1), (win_cx+px2, win_cy-py2), 2)

        # Draw View Cone
        camera_fov_half_rads = CAMERA_FOV_RADS / 2.0
        camera_fov_adj = (half_h * AERIAL_FOV_RADIUS) // self.aerial_radius
        camera_fov_opp = camera_fov_adj * math.tan(camera_fov_half_rads)

        camera_fov_opp = min(camera_fov_opp, half_w)
        camera_fov_adj = min(camera_fov_adj, half_h)

        camera_fov_cone = [(win_cx, win_cy),
                            (win_cx+camera_fov_opp, win_cy-camera_fov_adj),
                            (win_cx-camera_fov_opp, win_cy-camera_fov_adj)]
        draw_polygon_alpha(self.view_aerial, (255, 255, 0, 100), camera_fov_cone)
        
        # Draw Obstacles
        obstacle_scale = scale_factor * COLLISION_THRESHOLD
        for obj in self.objs:
            if obj["colour"] is not None and obj["rel"] is not None:
                obj_pos = (win_cx+obj["rel"][0]*scale_factor, win_cy-obj["rel"][1]*scale_factor)
                pygame.draw.circle(self.view_aerial, obj["colour"], obj_pos, obstacle_scale)
        
        # Draw vehicle
        vehicle_scale = 1.2*obstacle_scale
        pygame.draw.polygon(self.view_aerial, (255,0,0), [(win_cx, win_cy-vehicle_scale), (win_cx-vehicle_scale, win_cy+vehicle_scale), (win_cx+vehicle_scale, win_cy+vehicle_scale)])

        legend_colour=(41, 128, 185)

        # Draw North Arrow
        compass_offset = np.array([25, win_height-35])
        compass_pt1 = (auv_tf[:2,:2].T @ np.array([0,-15])) + compass_offset
        pygame.draw.circle(self.view_aerial, (255,255,255), (compass_offset[0], compass_offset[1]), 15)
        pygame.draw.line(self.view_aerial, (255,0,0), compass_offset, compass_pt1, 3)
        text_surface = self.font_small.render("North", True, legend_colour)
        self.view_aerial.blit(text_surface, (compass_offset[0]-12, win_height-20))

        # Draw Scale
        reference_scale = 1 if self.aerial_radius < 6 else 5
        line_pt1 = (win_width-10, win_height-35)
        line_pt2 = (win_width-10-reference_scale*scale_factor, win_height-35)
        pygame.draw.line(self.view_aerial, legend_colour, line_pt1, line_pt2, 3)
        pygame.draw.line(self.view_aerial, legend_colour, (line_pt1[0], line_pt1[1]-7), (line_pt1[0], line_pt1[1]+7), 3)
        pygame.draw.line(self.view_aerial, legend_colour, (line_pt2[0], line_pt2[1]-7), (line_pt2[0], line_pt2[1]+7), 3)
        text_surface = self.font_small.render(f"{reference_scale}m", True, legend_colour)
        self.view_aerial.blit(text_surface, (win_width-30, win_height-25))

        # Print Clock
        if self.hasEpochEnded():
            legend_colour = (192, 57, 43)
        else:
            self.time_diff = time.time() - self.start_time
        
        text_surface = self.font_med.render(self.getTimeElapsed(), True, legend_colour, (149, 165, 166))
        self.view_aerial.blit(text_surface, (win_width-180, win_height-75))


    def updateFrontView(self) -> None:
        win_width  = self.view_frontal.get_width()
        win_height = self.view_frontal.get_height()
        half_w = (win_width // 2)
        half_h = (win_height // 2)

        if self.view_frontal_base is None:
            self.view_frontal_base = pygame.Surface(self.view_frontal.get_size())

            # Draw Window
            pygame.draw.rect(self.view_frontal_base, (255,255,255), self.view_frontal.get_rect(), 0, 10)

            # Draw ground
            draw_vectical_gradient_rect(self.view_frontal_base, (41, 109, 152), (69, 182, 255), pygame.Rect(0,half_h,win_width,half_h))

        else:
            self.view_frontal.blit(self.view_frontal_base, self.view_frontal.get_rect())

        '''
        # Draw Grid
        rot_angle = np.deg2rad(50) 
        cos_val = math.cos(rot_angle)
        sin_val = math.sin(rot_angle)
        rot_mat = np.array([[  1, -8,     1],
                             [ 0,  1,     1],
                             [ 0, -0.01, 1]])
       
        
        rot_tf = np.zeros((6,6))
        rot_tf[0:3,0:3] = rot_mat
        rot_tf[3:6,3:6] = rot_mat


        grid_temp = (rot_tf @ self.grid.T).T
        grid_temp /= grid_temp[:,5][:,None]

        grid_temp = 20 *grid_temp
        for px1, py1, _, px2, py2, _ in grid_temp:
            pygame.draw.line(self.view_frontal, (0,0,0), (half_w+px1, half_h-py1), (half_w+px2, half_h-py2), 2)
        '''


        # Draw Obstacles
        for obj in self.objs:
            camera_fov_opp = obj["rel"][1] * self.camera_tan_val
            delta_x = obj["rel"][0] / (camera_fov_opp + 0.000001)

            if not (0 < obj["rel"][1] <= AERIAL_FOV_RADIUS and -1 < delta_x < 1 and obj["colour"] is not None):
                continue

            obstacle_scale = int((half_w * COLLISION_THRESHOLD) // camera_fov_opp) if (COLLISION_THRESHOLD < camera_fov_opp) else half_w
            obj_height = int(obstacle_scale * 5)
            delta_x = int(delta_x*half_w)

            pt1 = (int(half_w+delta_x), int(half_h-obj_height))
            pt2 = (int(half_w+delta_x), int(half_h+obj_height))

            obj_shade  = np.clip((1.0 - obj["rel"][1] / AERIAL_FOV_RADIUS), 0.5, 1.0)
            obj_colour = obj["colour"]
            obj_colour = (obj_shade*obj_colour[0], obj_shade*obj_colour[1], obj_shade*obj_colour[2])
            pygame.draw.line(self.view_frontal, obj_colour, pt1, pt2, obstacle_scale)
    

    def updatePlotView(self) -> None:
        win_width  = self.view_plot.get_width()
        win_height = self.view_plot.get_height()
        half_w = (win_width // 2)
        half_h = (win_height // 2)

        scale_factor = float(half_h) / (3*self.aerial_radius)

        # Draw Window
        pygame.draw.rect(self.view_plot, (255,165,0), self.view_plot.get_rect(), 0, 35)

        # Draw path
        scaled_path_pts = [(scale_factor*x+half_w, half_h-scale_factor*y) for x,y in self.plot_path]        
        pygame.draw.lines(self.view_plot, (0,0,0), False, scaled_path_pts, 2)

        # Draw radius Rings
        legend_colour=(255,255,255)

        last_radius_ratio = 0.0
        for reference_scale in [5, 10, 15, 20, 25, 30]:
            unit_radius_px = scale_factor*reference_scale
            unit_radius_ratio = unit_radius_px / float(half_h)
            if 0.1 < unit_radius_ratio < 0.9 and last_radius_ratio+0.2 < unit_radius_ratio:
                last_radius_ratio = unit_radius_ratio
                text_surface = self.font_small.render(f"{reference_scale} m", True, legend_colour)
                self.view_plot.blit(text_surface, (half_w-(text_surface.get_width() // 2), half_h+unit_radius_px+2))
                pygame.draw.circle(self.view_plot, legend_colour, (half_w, half_h), unit_radius_px, width=1)


    def updateView(self, auv_tf : np.array):
        # Clear screen
        self.view_main_window.fill((0,0,0))

        # Update each view
        self.updateAerialView(auv_tf)
        self.updateFrontView()
        self.updatePlotView()

        aerial_window_h = int(WINDOW_RATIO_AERIAL_FRONT_VIEW * WINDOW_HEIGHT)
        sub_window_w = (WINDOW_WIDTH // 2) - 5

        self.view_main_window.blit(self.view_aerial,  pygame.Rect(0, 0, self.view_aerial.get_width(), self.view_aerial.get_height()))
        self.view_main_window.blit(self.view_frontal, pygame.Rect(0, aerial_window_h+10, self.view_frontal.get_width(), self.view_frontal.get_height()))
        self.view_main_window.blit(self.view_plot,    pygame.Rect(sub_window_w+10, aerial_window_h+10, self.view_plot.get_width(), self.view_plot.get_height()))

        if self.view_display_text is not None:
            pos_text = self.view_aerial.get_rect().center
            self.view_main_window.blit(self.view_display_text, \
                                        (pos_text[0]-(self.view_display_text.get_width() // 2), \
                                        int(0.5*pos_text[1])-(self.view_display_text.get_height() // 2)))
            self.view_main_window.blit(self.view_display_subtext, \
                                        (pos_text[0]-(self.view_display_subtext.get_width() // 2), \
                                        int(0.5*pos_text[1])+self.view_display_text.get_height()-(self.view_display_subtext.get_height() // 2)))
        

        # Update our display
        pygame.display.update()


    ###=============== Keyboard handler Functions ===============###
    def handleControlKeys(self):

        # Loop through all active events
        for event in pygame.event.get():
            # Close the program if the user presses the 'X'
            if event.type == pygame.QUIT:
                return False
            elif event.type != pygame.KEYDOWN:
                continue

            if event.key == pygame.K_o:
                self.aerial_radius += 1
            elif event.key == pygame.K_p:
                self.aerial_radius -= 1


        self.aerial_radius = np.clip(self.aerial_radius, 3, 15)

        return True


    def handlePlayerKeys(self, ignoreKeys : bool=False):

        rel_yaw_enu = 0.0
        rel_fwd_enu  = 0.0
        rel_side_enu = 0.0

        if ignoreKeys:
            return (rel_fwd_enu, rel_side_enu, rel_yaw_enu)

        # Handles game controls
        pressed = pygame.key.get_pressed()
        if pressed[pygame.K_r]:
            self.restart()

        if pressed[pygame.K_LEFT]:
            rel_side_enu -= PLAYER_SIDE_STEP_ENU
        if pressed[pygame.K_RIGHT]:
            rel_side_enu += PLAYER_SIDE_STEP_ENU
        if pressed[pygame.K_UP]:
            rel_fwd_enu += PLAYER_FWD_STEP_ENU
        if pressed[pygame.K_DOWN]:
            rel_fwd_enu -= PLAYER_FWD_STEP_ENU

        if pressed[pygame.K_a]:
            rel_side_enu -= PLAYER_SIDE_STEP_ENU
        if pressed[pygame.K_d]:
            rel_side_enu += PLAYER_SIDE_STEP_ENU
        if pressed[pygame.K_w]:
            rel_fwd_enu += PLAYER_FWD_STEP_ENU
        if pressed[pygame.K_s]:
            rel_fwd_enu -= PLAYER_FWD_STEP_ENU

        if pressed[pygame.K_q]:
            rel_yaw_enu += PLAYER_YAW_STEP_ENU
        if pressed[pygame.K_e]:
            rel_yaw_enu -= PLAYER_YAW_STEP_ENU

        return (rel_fwd_enu, rel_side_enu, rel_yaw_enu)


    ###=============== Code execution ===============###
    def updateCmdOnce(self, rel_fwd_enu : float, rel_side_enu : float, rel_turn_enu : float):
    
        # Update vehicle pose
        auv_tf = self.updateMovement(rel_fwd_enu, rel_side_enu, rel_turn_enu)
        self.updateObjectPositions(auv_tf)
        
        # Check if termination condition is met
        self.checkEndCondition()

        # Update Views
        self.updateView(auv_tf)

        return self.handleControlKeys()
        
    def updateCmdContinuous(self, rel_fwd_enu : float, rel_side_enu : float, rel_turn_enu : float):
        self.last_cmd_fwd  = rel_fwd_enu
        self.last_cmd_side = rel_side_enu
        self.last_cmd_yaw  = rel_turn_enu
        
        return self.updateCmdOnce(rel_fwd_enu, rel_side_enu, rel_turn_enu)

    def spinOnceMission(self):
        return self.updateCmdOnce(self.last_cmd_fwd, self.last_cmd_side, self.last_cmd_yaw)

    def spinOnceUser(self):
        return self.updateCmdOnce(*self.handlePlayerKeys())


###=============== Code debugging Helpers ===============###
class thread_with_trace(threading.Thread):
    def __init__(self, *args, **keywords):
        threading.Thread.__init__(self, *args, **keywords)
        self.killed = False

    def start(self):
        self.killed = False
        self.__run_backup = self.run
        self.run = self.__run     
        threading.Thread.start(self)

    def __run(self):
        sys.settrace(self.globaltrace)
        self.__run_backup()
        self.run = self.__run_backup

    def globaltrace(self, frame, event, arg):
        return self.localtrace if event == 'call' else None

    def localtrace(self, frame, event, arg):
        if self.killed and event == 'line':
            raise SystemExit()

        return self.localtrace

    def kill(self):
        self.killed = True
    
    def join(self, *args):
        threading.Thread.join(self, *args)


def runSim(worldId, isPerfectControl=False, mission_plans=None, depthPIDController=None):

    global GAME_CURRENT_WORLD
    GAME_CURRENT_WORLD = worldId
        
    # Initialise all the pygame modules
    pygame.init()

    # Create a game window
    game_window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

    # Set title
    pygame.display.set_caption("Simple Task Gate SIM")

    mygame = SimpleAUVSimulatorGame(game_window, depthPIDController)
    myinterface = AUVInterface(mygame)
    myinterface.depthController = depthPIDController
    fpsClock = pygame.time.Clock()

    if mission_plans is None:

        # Game loop
        while mygame.spinOnceUser():
            fpsClock.tick(20)
            pass

    else:
        # Game loop
        continue_run_sim = True
        for mission_logic in mission_plans:
        
            print('\n\n=======================')
            print('Running Mission: ' + mission_logic.__name__)
            
            myinterface.is_ok = True
            myinterface.task_success = False
            mission_thread = thread_with_trace(target=mission_logic, args=(myinterface,))
            mission_thread.start()
            
            while continue_run_sim and mission_thread.isAlive():
                continue_run_sim = mygame.spinOnceMission()
                fpsClock.tick(20)
                pass
           
            # Flag mission plan to shutdown
            myinterface.is_ok = False
            fpsClock.tick(10)

            mission_thread.kill()
            mission_thread.join()
            del mission_thread

            print("Mission outcome: " + str("Success" if myinterface.task_success == True else "Failed"))
            print()

            # Case: Mission Failed or user terminate
            if (not continue_run_sim) or (myinterface.task_success != True):
                break
        
            pass
        
        # Case: Mission Success or user terminate
        if not mygame.hasEpochEnded():
            mygame.displayResult("WASTED! Mission Failed", (255, 75, 56), "Please ammend your mission plan.")
            
        if continue_run_sim:
            while mygame.spinOnceMission():
                fpsClock.tick(20)
                pass

    # Uninitialize all pygame modules and quit the program
    pygame.quit()
    sys.exit()


def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--world', '-w', type=int, default=1, help='Select the Simulation World (1 to 5)')
    parser.add_argument('--perfect', '-p', default=False, action="store_true", help="Assumes perfect control over the AUV")
    parser.add_argument('--mission', '-m', default=False, action="store_true", help="Runs AUV's mission")

    args = parser.parse_args()
    myMissions = None
    myDepthPID = None
    if args.mission:
        from mission import getMyMissionPlans
        myMissions, myDepthPID = getMyMissionPlans()
    
    
    runSim(args.world-1, args.perfect, myMissions, myDepthPID)
    pass

if __name__ == '__main__':
    main()








