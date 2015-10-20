""" This library gives a simple implementation of a pid controller for the RoseBot.
 PID controllers output a value for error between desired reference input and measurement feedback to
 minimize error value."""


class RoseBot_pid:


    def __init__(self, P=1.0, I=0.0, D=0.0, derivator=0, integrator=0, integrator_max=100, integrator_min=-100):

        self.kp = P
        self.ki = I
        self.kd = D
        self.derivator = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        """
        Calculate pid output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.p_value = self.kp * self.error
        self.d_value = self.kd * (self.error - self.derivator)
        self.derivator = self.error

        self.integrator = self.integrator + self.error

        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.i_value = self.integrator * self.ki

        pid = self.p_value + self.i_value + self.d_value

        return pid
