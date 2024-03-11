from models.mav_dynamics_control import MavDynamics
from message_types.msg_delta import MsgDelta
from scipy.optimize import minimize
import numpy as np

def compute_trim(mav: MavDynamics, delta: MsgDelta):
    # parameters to input for trim
    # alpha, elevator, throttle

    x0 = [mav._alpha, delta.elevator, delta.throttle]
    bounds = [(0, np.deg2rad(12)), (-1,1), (0,1)]
    res = minimize(mav.calculate_trim_output, x0, bounds=bounds, method='SLSQP')
    print(res)
    return(res.x[0], res.x[1], res.x[2])

    # delta_a = .1
    # forces_moments = mav._forces_moments(delta=delta)

    # fx = forces_moments[0]
    # fz = forces_moments[2]
    # m = forces_moments[4]

    # print(fx, fz, m)

    # mav.initialize_velocity(mav._Va, mav._alpha + delta_a, mav._beta)

    # forces_moments = mav._forces_moments(delta=delta)

    # fx_n = forces_moments[0]
    # fz_n = forces_moments[2]
    # m_n = forces_moments[4]

    # fx_delta = (fx_n - fx) / delta_a
    # fz_delta = (fz_n - fz) / (delta_a)
    # m_delta = (m_n - m) / delta_a

    # print(fx_delta, fz_delta, m_delta)

    # delta_a = -fx/fx_delta
    # compute_paramters(mav, delta, delta_a/8)

def compute_paramters(mav, delta, delta_a):

    mav.initialize_velocity(mav._Va, mav._alpha + delta_a, mav._beta)
    forces_moments = mav._forces_moments(delta=delta)