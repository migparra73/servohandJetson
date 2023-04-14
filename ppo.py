import gym
import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO

# Create the gym environment
env = gym.make('CartPole-v1')

# Wrap the environment in a vectorized environment
env = DummyVecEnv([lambda: env])

# Create the PPO agent
model = PPO('MlpPolicy', env, verbose=1)

# Train the agent for 10,000 timesteps
model.learn(total_timesteps=10000)

# Evaluate the agent
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
    if dones:
        break

# Close the environment
env.close()