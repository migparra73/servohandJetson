import numpy as np
import gym
from gym import utils, spaces
from math import exp
import servo
from typing import Type

class PhysicalServoHand(gym.Env, utils.EzPickle):
    def __init__(
                self,
                forward_reward_weight=1.0,
                ctrl_cost_weight=0.0001,
                reset_noise_scale=0.1,
                max_reward=0.0,
                max1=1.0,
                max2=1.0,
                i=-1,
                #posstd=np.zeros([2001000, 1]), # 15 gb memory required
                #volstd=np.zeros([2001000, 14]), # 15 gb memory required
                obs_mean = np.zeros(20),
                obs_var=np.ones(20),
                nn=0,
                exclude_current_positions_from_observation=False,
                servoDriver: Type[servo.Servo] = None # Hint to the interpreter that we are going to use the servo.Servo class.
                ):        
        utils.EzPickle.__init__(**locals())
        self._forward_reward_weight = forward_reward_weight
        self._ctrl_cost_weight = ctrl_cost_weight
        self._reset_noise_scale = reset_noise_scale
        self._max_reward= max_reward
        self._max_1= max1
        self._max_2= max2
        self._posstd=posstd
        self._volstd=volstd
        self._ij=i
        self.face_id=nn
        self._obs_mean = obs_mean
        self._obs_var = obs_var
        self._exclude_current_positions_from_observation = (
            exclude_current_positions_from_observation)
        
        self.action_space = gym.spaces.MultiBinary(7) # Each one is mapped to "back" or "forward" (or in the case of the linear actuator, up/down)
        self.servo_driver = servoDriver
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32) # The observation space is a single float, which is the height of the ball.

    def control_cost(self, action):
        control_cost = self._ctrl_cost_weight * np.sum(np.square(action))
        return control_cost
    
    def step(self, action):
        y_position_before = 10 * np.random.rand(1) # Ball position? - use openCV
        z_position_before = 10 * np.random.rand(1) # Ball position? - use openCV

        # Action is a 7 element array, as defined by action_space above.
        # action[0] is the linear actuator
        # action[1] is the first servo and so on.
        # action[x] is either 0 or 1, where 0 is "clockwise" and 1 is "counterclockwise"
        
        self.servo_driver.servoUnitStep(action) # This is the function that actually moves the servos.
        
        y_position_after = 10 * np.random.rand(1) #Ball position? - use openCV
        z_position_after = 10 * np.random.rand(1) #ball position? - use openCV


        y_velocity = ((y_position_after - y_position_before)
                      / self.dt)
    
        z_velocity = ((z_position_after - z_position_before)
                      / self.dt)

        ctrl_cost = self.control_cost(action)

        if self._ij>999999:   

          rot_weight=1
          lift_weight=1

        else:

          rot_weight=1
          lift_weight=1 
        

        # lift_weight=1

        # rot_weight=1

        
        # forward_reward = rot_weight*(self._forward_reward_weight * y_velocity) - lift_weight*(95*abs(z_position_after-0.06))
        forward_reward =  lift_weight*(95*abs(z_position_after-0.06)) ## only height reward


        
        height=z_position_before

        height_reward=-(95*abs(z_position_after-0.06))

        rotation_reward=(self._forward_reward_weight * y_velocity)

        degree_pos= y_position_after
        

        observation = self.get_obs()

        self._ij+=1 
        #self._posstd[self._ij,:]=z_position_before # Implement this as a memory mapped file to save space
        #self._volstd[self._ij,:]=observation # Implement this as a memory mapped file to save space
        

        current_reward = forward_reward - ctrl_cost
        
        reward=current_reward
        
    
        done = False
        info = {
            'y_position': y_position_after,
            'y_velocity': y_velocity,

            'reward_run': forward_reward,
            'reward_ctrl': -ctrl_cost,
            'height_reward':height_reward
        }

        return observation, reward, done, info, height, rotation_reward,height_reward,degree_pos 

    
    def get_obs(self):

    def reset_model(self):
