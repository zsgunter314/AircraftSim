"""
msg_autopilot
    - messages type for input to the autopilot
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""
import numpy as np

class MsgAutopilot:
    def __init__(self):
        self.airspeed_command = 25  # commanded airspeed m/s
        self.course_command = 0  # commanded course angle in rad
        self.altitude_command = 152.4  # commanded altitude in m
        self.phi_feedforward = 0.0  # feedforward command for roll angle

# 500 ft = 152.4 m
#1000 ft = 304.8 m
#2000 ft = 609.6 m