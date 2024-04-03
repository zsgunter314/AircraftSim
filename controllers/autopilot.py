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
from controllers.pd_control import PDControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

airspeed_throttle_kp = 0.005 #0.00001
airspeed_throttle_ki = 0.0005

yaw_damper_kp = 10.0
yaw_damper_kd = 1.0


alpha_elevator_kp = -(1/np.deg2rad(12))
alpha_elevator_ki = -0.0018
alpha_elevator_kd = 0.1 * alpha_elevator_kp

alt_kp = .05
alt_ki = 0.005
alt_kd = .05

gamma_kp= .095
gamma_ki=0.0001
gamma_kd= .011

chi_kp = .2
chi_ki = 0.0001
chi_kd = 0.005

roll_kp = 2
roll_ki = 0.001
roll_kd = 0.05




class Autopilot:
    def __init__(self, delta, mav, ts_control):
        
        self.throttle_from_airspeed = PIControl(
                        kp=airspeed_throttle_kp,
                        ki=airspeed_throttle_ki,
                        Ts=ts_control,
                        max=1.0,
                        min=0,
                        init_integrator=delta.throttle/airspeed_throttle_ki
                        )
        
        self.elevator_from_alpha = PIDControl(
            kp=alpha_elevator_kp,
            ki=alpha_elevator_ki,
            kd=alpha_elevator_kd,
            Ts=ts_control,
            max=1.0,
            min =-1.0,
            init_integrator=delta.elevator/alpha_elevator_ki
    
        )
        
        self.yaw_damper = PDControl(
            kp=yaw_damper_kp, 
            kd = yaw_damper_kd,
            Ts=ts_control,
            limit=1.0
        )
        
        self.alt_control = PIDControl(
            kp= alt_kp,
            ki= alt_ki,
            kd = alt_kd,
            Ts=ts_control,
            max=np.deg2rad(50),
            min =np.deg2rad(-50),
            init_integrator=0.0
        )
        
        self.gamma_control = PIDControl(
            kp= gamma_kp,
            ki= gamma_ki,
            kd = gamma_kd,
            Ts=ts_control,
            max=np.deg2rad(12),
            min =np.deg2rad(-2),
            init_integrator=0.0
        )
        
        self.chi_control = PIDControl(
            kp= chi_kp,
            ki= chi_ki,
            kd = chi_kd,
            Ts=ts_control,
            max=np.deg2rad(45),
            min =np.deg2rad(-45),
            init_integrator=0.0
        )
        
        self.roll_control = PIDControl(
            kp= roll_kp,
            ki= roll_ki,
            kd = roll_kd,
            Ts=ts_control,
            max=1.0,
            min=-1.0,
            init_integrator=delta.aileron/roll_ki
        )
        

        
        
        
        
        # # instantiate lateral-directional controllers
        # self.roll_from_aileron = PDControlWithRate(
        #                 kp=AP.roll_kp,
        #                 kd=AP.roll_kd,
        #                 limit=np.radians(45))
        # self.course_from_roll = PIControl(
        #                 kp=AP.course_kp,
        #                 ki=AP.course_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(30))
        # # self.yaw_damper = TransferFunction(
        # #                 num=np.array([[AP.yaw_damper_kr, 0]]),
        # #                 den=np.array([[1, AP.yaw_damper_p_wo]]),
        # #                 Ts=ts_control)
        # self.yaw_damper = TFControl(
        #                 k=AP.yaw_damper_kr,
        #                 n0=0.0,
        #                 n1=1.0,
        #                 d0=AP.yaw_damper_p_wo,
        #                 d1=1,
        #                 Ts=ts_control)

        # # instantiate longitudinal controllers
        # self.pitch_from_elevator = PDControlWithRate(
        #                 kp=AP.pitch_kp,
        #                 kd=AP.pitch_kd,
        #                 limit=np.radians(45))
        # self.altitude_from_pitch = PIControl(
        #                 kp=AP.altitude_kp,
        #                 ki=AP.altitude_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(30))
        
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
        
        delta.rudder = self.yaw_damper.update(0, state.beta)
        delta.throttle = self.throttle_from_airspeed.update(cmd.airspeed_command, state.Va)            #25
        
        cmd_gamma = self.alt_control.update(cmd.altitude_command, state.altitude)  #cmd.altitude_command
        cmd_alpha = self.gamma_control.update(cmd_gamma, state.gamma)
        delta.elevator = self.elevator_from_alpha.update(cmd_alpha, state.alpha)
        
        cmd_roll = self.chi_control.update(cmd.course_command, state.chi)
        delta.aileron = self.roll_control.update(cmd_roll, state.phi)
        
        print(state.altitude)
        print(state.Va)
        
        
        self.commanded_state.altitude = 0
        self.commanded_state.Va = 0
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = 0
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output