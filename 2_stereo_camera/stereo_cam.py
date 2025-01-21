import pybullet as p
import pybullet_data
from racecarGymEnv import RacecarGymEnv

env.reset()
while True:
    env.step(env.action_space.sample())