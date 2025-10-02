import gym
from gym import spaces
import numpy as np
from dataclasses import dataclass

@dataclass
calss GridConfig:
    size: int = 6
    start: tuple = (0, 0)
    goal: tuple = (5, 5)
    