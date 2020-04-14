from math import pi
import time


class PID (object):

    def __init__(self, kp, kd, ki, minOutput, maxOutput, integratorMin, integratorMax):
        self.m_kp = kp                       # weight of proportional control
        self.m_kd = kd                       # weight of derivative control
        self.m_ki = ki                       # weight of integral control
        self.m_minOutput = minOutput         # largest output from controller
        self.m_maxOutput = maxOutput         # smallest output from controller
        self.m_integratorMin = integratorMin    # maximum integral value
        self.m_integratorMax = integratorMax    # minimum integral value
        self.m_integral = 0
        self.m_previousError = 0
        self.m_previousTime = time.time()

    # reset of essential PID parameters
    def reset(self):
        self.m_integral = 0
        self.m_previousError = 0
        self.m_previousTime = time.time()

    # sets (or resets) the integral value of the PID controller
    def setIntegral(self, integral):
        self.m_integral = integral

    # returns the value of ki
    def set_ki(self):
        return self.m_ki

    def update(self, value, targetValue):
        t = time.time()
        dt = 0.001
        error = -targetValue + value

        if error > pi:
            error = error-2*pi
        elif error < -pi:
            error = error+2*pi

        self.m_integral += error * dt
        self.m_integral = max(min(self.m_integral, self.m_integratorMax), self.m_integratorMin)
        p = self.m_kp * error
        d = self.m_kd * (error - self.m_previousError) / dt
        i = self.m_ki * self.m_integral
        output = p + d + i
        rudder_angle = max(min(output, self.m_maxOutput), self.m_minOutput)

        return rudder_angle