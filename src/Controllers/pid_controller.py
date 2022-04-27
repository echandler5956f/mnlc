#!/usr/bin/env python
class PIDController():
    # basic PID controller borrowed from my RBE2002 final project
    sumError = 0
    errorBound = 0
    (Kp, Ki, Kd) = (0, 0, 0)
    prevError = 0
    currError = 0
    currEffort = 0
    effortCap = 0

    def PIDController(self, Kp, Ki, Kd, errorBound):
        """
        Class constructor
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.errorBound = errorBound
        self.effortCap = 0

    def ComputeEffort(self, error):
        self.currError = error  # store in case we want it later
        self.sumError += self.currError
        if(self.errorBound > 0):  # cap error; errorBound == 0 means don't cap
            # you could multiply sumError by Ki to make it scale
            if(abs(self.sumError) > self.errorBound):
                # if we exceeded the limit, just subtract it off again
                self.sumError -= self.currError
        derivError = self.currError - self.prevError
        self.prevError = self.currError
        self.currEffort = self.Kp * self.currError + \
            self.Ki * self.sumError + self.Kd * derivError
        if(self.effortCap > 0 and self.currEffort > self.effortCap):
            self.currEffort = self.effortCap
        return self.currEffort

    def reset(self):
        self.currError = 0
        self.sumError = 0
        self.prevError = 0
        self.currEffort = 0