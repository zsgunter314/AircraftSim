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
        self.course_command = np.deg2rad(90)  # commanded course angle in rad
        self.altitude_command = 500  # commanded altitude in m
        self.phi_feedforward = 0.0  # feedforward command for roll angle
