class PID_Control:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.01, max=1.0, min=-1.0, init_integrator=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.max = max
        self.min = min
        self.integrator = init_integrator

    def update(self, commanded_value, current_value):
        error = commanded_value - current_value
        self.integrator = self.integrator + error * self.Ts
        Error_rate = (error - last_error) / self.Ts

        u = self.kp * error + self.kd * Error_rate + self.ki *  self.integrator

        last_error = error

        return u