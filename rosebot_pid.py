"""# This library gives a simple implementation of a PID controller for the RedBot.
# PID controllers output a value for error between desired reference input and measurement feedback to
# minimize error value.
#
# Credit and thanks to cnr437@gmail.com for the majority of this code """


class PID:


    def __init__(self, P=1.0, I=0.0, D=0.0, derivator=0, integrator=0, integrator_max=100, integrator_min=-100):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.derivator = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.p_value = self.Kp * self.error
        self.d_value = self.Kd * (self.error - self.derivator)
        self.derivator = self.error

        self.integrator = self.integrator + self.error

        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.i_value = self.integrator * self.Ki

        PID = self.p_value + self.i_value + self.d_value

        return PID

    def set_point(self, set_point):
        """
        Initilize the setpoint of PID. Sets the value that the PID control aims to stabilize around
        """
        self.set_point = set_point
        self.integrator = 0
        self.derivator = 0

    def set_integrator(self, integrator):
        self.integrator = integrator

    def set_derivator(self, derivator):
        self.derivator = derivator

    def set_Kp(self, P):
        self.Kp = P

    def set_Ki(self, I):
        self.Ki = I

    def set_Kd(self, D):
        self.Kd = D

    def get_point(self):
        return self.set_point

    def get_error(self):
        return self.error

    def get_integrator(self):
        return self.integrator

    def get_derivator(self):
        return self.derivator
