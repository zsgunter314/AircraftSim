"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import numpy as np
from scipy import stats
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
import parameters.aerosonde_parameters as MAV
from message_types.msg_sensors import MsgSensors

class Observer:
    def __init__(self, ts_control, initial_measurements = MsgSensors()):
        # initialized estimated state message
        self.estimated_state = MsgState()

        #self.estimated_state.Vg = 20
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant

        


        ##### TODO #####
        self.lpf_gyro_x = AlphaFilter(alpha=0.9, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.5, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.65, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.65, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.65, y0=initial_measurements.accel_z)
        # use alpha filters to low pass filter absolute and differential pressure
   
        self.lpf_abs = AlphaFilter(alpha=0.99, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = EkfAttitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition()
        

        
        """
                            0.5,  # pn
                    0.5,  # pe
                    0.5,  # Vg
                    0.5, # chi
                    0.5, # wn
                    0.5, # we
                    0.0001, #0.0001, # psi
        """

        self.position_ekf.xhat = np.array([[initial_measurements.gps_n], [initial_measurements.gps_e], [initial_measurements.gps_Vg], [initial_measurements.gps_course], [0.0], [0.0], [initial_measurements.gps_course]])

    def update(self, measurement, true_state):
        ##### TODO #####

        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) -self.estimated_state.bx 
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) -self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) -self.estimated_state.bz

        abs_pressure = self.lpf_abs.update(measurement.abs_pressure)
        diff_pressure = self.lpf_diff.update(measurement.diff_pressure)

        # invert sensor model to get altitude and airspeed
        P0 = 101325
        L0 = -0.0065
        T0 = 288.15
        M_M = 0.0289644 #kg/mol
        R = 8.31432
        
        tmp_P = P0 - abs_pressure
        self.estimated_state.altitude = tmp_P/(MAV.rho*MAV.gravity)
        self.estimated_state.Va = np.sqrt((2/MAV.rho)*diff_pressure)

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)


        #self.estimated_state.altitude = true_state.altitude
        # not estimating these
        self.estimated_state.gamma = true_state.gamma
        self.estimated_state.alpha = true_state.alpha
        self.estimated_state.beta = true_state.beta
        self.estimated_state.bx = true_state.bx
        self.estimated_state.by = true_state.by
        self.estimated_state.bz = true_state.bz
        return self.estimated_state


class AlphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha*self.y + (1-self.alpha)*u
        return self.y



class EkfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):

        self.Q = np.diag([0.5, 0.5])
        self.Q_gyro = np.diag([SENSOR.gyro_sigma**2, SENSOR.gyro_sigma**2, SENSOR.gyro_sigma**2])
        self.R_accel = np.diag([SENSOR.accel_sigma**2, SENSOR.accel_sigma**2, SENSOR.accel_sigma**2])
        self.N = 1  # number of prediction step per sample
        self.xhat = np.array([[0.0], [0.0]]) # initial state: phi, theta
        self.P = np.diag([0, 0])
        self.Ts = SIM.ts_control/self.N
        self.gate_threshold = stats.chi2.isf(q=0.01, df=3)


    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        phi_dot = state.p + state.q*np.sin(state.phi)*np.tan(state.theta) + state.r*np.cos(state.phi)*np.tan(state.theta)
        theta_dot = state.q*np.cos(state.phi) - state.r*np.sin(state.phi)
        f_ = np.zeros((2,1))
        f_[0] = phi_dot
        f_[1] = theta_dot
        return f_

    def h(self, x, measurement, state):
        # measurement model y
        x_accel = state.q*state.Va*np.sin(state.theta) + MAV.gravity*np.sin(state.theta)
        y_accel = state.r*state.Va*np.cos(state.theta) - state.p*state.Va*np.sin(state.theta) - MAV.gravity*np.cos(state.theta)*np.sin(state.phi)
        z_accel = -state.q*state.Va*np.cos(state.theta) - MAV.gravity*np.cos(state.theta)*np.cos(state.phi)
      
        h_ = np.array([[x_accel],  # x-accel
                        [y_accel],# y-accel
                        [z_accel]])  # z-accel
        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        Tp = self.Ts
        for i in range(0, self.N):
            A_matrix = jacobian(self.f, self.xhat, measurement, state)
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            self.P = self.P + self.Ts * ((np.matmul(A_matrix, self.P)) + self.P*A_matrix.T+ self.Q)

    def measurement_update(self, measurement, state):
        # measurement updates
        h = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T


        S = np.matmul(np.matmul(C, self.P), C.T) + self.R_accel
        L = np.matmul(np.matmul(self.P, C.T), np.linalg.inv(S))
        error_stuff = y - h

        S_inv = np.linalg.inv(S)
        if (y-h).T @ S_inv @ (y-h) < self.gate_threshold:
        # update state estimate and covariance
            self.xhat = self.xhat + np.matmul(L, error_stuff)
            self.P = np.matmul((np.eye(2) - np.matmul(L, C)), self.P)

class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi

    #sensor noise -> increase R 
    #proecess noise -> increase Q 
    #more prediction high R low Q
    #tune Q
    def __init__(self):

        self.Q = np.diag([
                    0.5,  # pn
                    0.5,  # pe
                    0.5,  # Vg
                    0.5, # chi
                    0.5, # wn
                    0.5, # we
                    0.0001, #0.0001, # psi
                    ])
        self.R_gps = np.diag([
            SENSOR.gps_n_sigma**2,  # pn
            SENSOR.gps_e_sigma**2,  # pe
            SENSOR.gps_Vg_sigma**2,  # Vg
            SENSOR.gps_course_sigma**2,  # chi
        ])
        
        self.R_pseudo = np.diag([           
                    0.001,  # pseudo measurement #1
                    0.001,  # pseudo measurement #2
                    ])
        self.N = 1  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[0.0], [0.0], [0.01], [0.0], [0.0], [0.0], [0.0]])
        self.P = np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.gps_n_old = 0
        self.gps_e_old = 0
        self.gps_Vg_old = 0
        self.gps_course_old = 0
        self.pseudo_threshold = stats.chi2.isf(q=.01, df=3)
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, measurement, state):

        # system dynamics for propagation model: xdot = f(x, u)
        f_ = np.array([[0],
                       [0],
                       [0],
                       [0],
                       [0.0],
                       [0.0],
                       [0],
                       ])
        psi_dot = state.q * (np.sin(state.phi)/(np.cos(state.theta))) + state.r*((np.cos(state.phi))/(np.cos(state.theta)))
        f_[0] = state.Va * np.cos(state.chi)
        f_[1] = state.Vg * np.sin(state.chi)
        term1 = state.Va*np.cos(state.psi)+state.wn
        term2= -state.Va*psi_dot*np.sin(state.psi)
        term3 = state.Va*np.sin(state.psi) + state.we
        term4 = state.Va*psi_dot*np.cos(state.psi)
        f_[2] = (term1*term2 + term3*term4)/state.Vg
        f_[3] = (MAV.gravity/(state.Vg))*np.tan(state.phi)
        f_[4] = 0
        f_[5] = 0
        f_[6] = psi_dot
        return f_

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        h_ = np.array([
            [state.north], #pn
            [state.east], #pe
            [state.Vg], #Vg
            [state.chi], #chi
        ])
        return h_

    def h_pseudo(self, x, measurement, state):

        h_ = np.array([[state.Va * np.cos(state.psi) + state.wn - state.Vg*np.cos(state.chi)],
                       [state.Va * np.sin(state.psi) + state.we - state.Vg*np.sin(state.chi)]])
        # measurement model for wind triangale pseudo measurement

        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)

            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
        
            # convert to discrete time models
            Ad = np.eye(7) + self.Ts * A + (A**2)*self.Ts
            
            self.P = Ad*self.P*Ad.T + (self.Ts**2)*self.Q

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudo measurement
        
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T #left as zero because we do not have wind.
        S = C@self.P@C.T + self.R_pseudo
        
        
        S_inv = np.linalg.inv(S)
        L = self.P@C.T@S_inv

        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            self.P = (np.eye(7) - L@C)@self.P@(np.eye(7)-L@C).T + (L @ self.R_pseudo @L.T)   
            #tmp_one = (L @ self.R_pseudo @L.T)
            self.xhat = self.xhat + L@(y-h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            
            
            S = C@self.P@C.T + self.R_gps
            S_inv = np.linalg.inv(S)
            L = self.P@C.T@S_inv
            """
                                0.5,  # pn
                        0.5,  # pe
                        0.5,  # Vg
                        0.5, # chi
                        0.5, # wn
                        0.5, # we
                        0.0001, #0.0001, # psi
            """


            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                self.P = (np.eye(7) - L@C)@self.P@(np.eye(7)-L@C).T + (L @ self.R_gps @L.T)   
                self.xhat = self.xhat + L@(y-h)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


def jacobian(fun, x, measurement, state):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement, state)
    m = f.shape[0]
    #print("Jonathon Elliott")
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J