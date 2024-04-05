"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
# from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pid_control import PIDControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from controllers.pd_control import PDControl
from models.mav_dynamics_control import MavDynamics
yaw_damper_kp = 10.0
yaw_damper_kd = 1.0



#ALPHA STUFF
ALPHA_MAX = np.deg2rad(12)
ALPHA_MIN = np.deg2rad(-2)

MAX_GAMMA_OUTPUT = np.deg2rad(45) #X degrees of gamma at any current time
MAX_ALTITUDE = 750 #in meters FOR kp CALCULATIONS


altitude_kp = 0.05
altitude_ki = 0.01
altitude_kd = 0.0001


gamma_kp= 1.2
gamma_ki= 1
gamma_kd= 0


# gamma_kp = (np.deg2rad(12))/(np.deg2rad(15))
# gamma_kd=0.5*gamma_kp
# gamma_ki=0.001




alpha_elevator_kp = -20 
alpha_elevator_ki = -20
alpha_elevator_kd = -2


airspeed_throttle_kp = 0.05
airspeed_throttle_ki = 0.05


MAX_ROLL = np.deg2rad(45)
MIN_ROLL = np.deg2rad(-45)
MAX_CHI = np.deg2rad(180)


chi_kp = 0.5
chi_ki = 0.01 #positive
chi_kd = 0.05


roll_kp = 0.22 #Kp determined to be positive
roll_kd = 0.02#Kd determined to be positive
roll_ki = 0.1


class Autopilot:
    def __init__(self, delta, mav: MavDynamics, ts_control):


        self.throttle_from_airspeed = PIControl(
                        kp=airspeed_throttle_kp,
                        ki=airspeed_throttle_ki,
                        Ts=ts_control,
                        max=1.0,
                        min=0.0,
                        init_integrator=delta.throttle/airspeed_throttle_ki)
        self.elevator_from_alpha = PIDControl(
            kp=alpha_elevator_kp,
            ki=alpha_elevator_ki,
            kd=alpha_elevator_kd,
            max=1,
            min=-1,
            Ts=ts_control,
            init_integrator=delta.elevator/alpha_elevator_ki
        )
        self.yaw_damper = PDControl(kp=yaw_damper_kp, kd=yaw_damper_kd, Ts=ts_control, limit=1.0)

        self.altitude_controller = PIDControl(kp= altitude_kp, ki= altitude_ki, kd = altitude_kd, Ts=ts_control, max=np.deg2rad(15), min =np.deg2rad(-15), init_integrator=0.0)
                
        self.gamma_control = PIDControl(kp= gamma_kp, ki= gamma_ki, kd = gamma_kd, Ts=ts_control, max=ALPHA_MAX, min =ALPHA_MIN, init_integrator=mav.true_state.alpha/gamma_ki)
                
        self.chi_controller = PIDControl(kp= chi_kp, ki= chi_ki, kd = chi_kd, Ts=ts_control, max=MAX_ROLL, min =MIN_ROLL, init_integrator=0.0)
                
        self.roll_controller = PIDControl(kp= roll_kp, ki= roll_ki, kd = roll_kd, Ts=ts_control, max=1.0, min=-1.0, init_integrator=delta.aileron/roll_ki)
        
        self.commanded_state = MsgState()

    def update(self, cmd, state):
	
	#### TODO #####
        # lateral autopilot


        # longitudinal autopilot


        # construct control outputs and commanded states
        delta = MsgDelta(elevator=0,
                         aileron=0,
                         rudder=0,
                         throttle=0)
        
        
        #delta.throttle = self.throttle_from_airspeed.update(23, state.Va)
        # delta.elevator = self.elevator_from_alpha.update(np.deg2rad(3), state.alpha)
        #delta.elevator = self.elevator_from_alpha.update(state.alpha, state.alpha)
        delta.rudder = self.yaw_damper.update(0, state.beta)

        
        delta.throttle = self.throttle_from_airspeed.update(cmd.airspeed_command, state.Va)    

        #delta.throttle = self.throttle_from_airspeed.update(23, state.Va)
        #delta.elevator = self.elevator_from_alpha.update(np.deg2rad(2), state.alpha)


        cmd_gamma = self.altitude_controller.update(cmd.altitude_command, state.altitude)


        #cmd_gamma = np.deg2rad(25)
        # cmd_gamma = np.deg2rad(5)
        #print("Gamma error: " + str(np.rad2deg(cmd_gamma-state.gamma)))



        
        cmd_alpha = self.gamma_control.update(cmd_gamma, state.gamma)  
        
             
        #print("Command alpha: " + str(np.deg2rad(cmd_alpha)))
        #print("Gamma: " + str(np.rad2deg(state.gamma)))
    


        #cmd_alpha = np.deg2rad(3)
        delta.elevator = self.elevator_from_alpha.update(cmd_alpha, state.alpha)


        #print("Error: " + str(np.rad2deg(cmd_alpha-state.alpha)))

        if(cmd.course_command > np.pi):
            cmd.course_command=np.pi-cmd.course_command
        
        
        output_roll = self.chi_controller.update(cmd.course_command, state.chi)

        #output_roll = self.new_method()
        #output_roll = cmd.course_command
        delta.aileron = self.roll_controller.update(output_roll, state.phi)
        #delta.aileron=0.0

        #print("Alpha Error:" + str(np.rad2deg(cmd_alpha-state.alpha)))

        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command


        self.commanded_state.phi = output_roll
        # self.commanded_state.theta = 0
        self.commanded_state.chi = cmd.course_command
        self.commanded_state.alpha = cmd_alpha
        self.commanded_state.gamma = cmd_gamma


        return delta, self.commanded_state

    def new_method(self):
        output_roll = np.deg2rad(25)
        return output_roll

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output