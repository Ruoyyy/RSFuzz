import attack
import katz
import os
import sys
import numpy as np
import math
sys.path.append(r'D:\liyilin\Swarm\SwarmRoboticsSim\SWARMFLAWFINDER')
from new_tool import *
# pos_old = [0, 0, 0]
# pos_new = [[1,1,1],[2,2,2],[3,3,3]]

# Dornes_len = 1
# Obstacles_len = 2
# print(SwarmDCC(pos_old, pos_new, Dornes_len, Obstacles_len))
new_pos = [0,0]
for i in range(10):
    new_pos = move_to_target(new_pos, [3,3], 1, 0.5)
    print(new_pos)