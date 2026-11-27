from math import asin, atan2, cos, pi, sin, sqrt

import numpy as np

HOME = np.tile([0., 0.9, -1.8], 4)
KP, KD = 55., 2.
PERIOD = 0.012
LENGTH = 0.213


def angles(quat):
    w, x, y, z = quat
    roll = atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = asin(float(np.clip(2 * (w*y - z*x), -1, 1)))
    yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return roll, pitch, yaw


class Policy:
    def __init__(self, weights):
        self.weights = weights
        self.phase = 0.
        self.heading = None
        self.blend = 0.

    def step(self, speed, quat, gyro, dt=PERIOD):
        roll, pitch, yaw = angles(quat)
        if self.heading is None:
            self.heading = yaw
        frequency, stride_gain, lift, height, duty, roll_gain, pitch_gain, steer_gain, hip_gain, width = self.weights
        self.phase = (self.phase + dt * frequency) % 1.
        self.blend = float(np.clip(self.blend + dt * (2 if speed > 0.02 else -4), 0, 1))
        heading_error = atan2(sin(yaw - self.heading), cos(yaw - self.heading))
        steer = float(np.clip(steer_gain * heading_error + 0.08 * gyro[2], -0.25, 0.25))
        result = []
        for leg, offset in enumerate((0., .5, .75, .25)):
            side = -1 if leg % 2 == 0 else 1
            front = 1 if leg < 2 else -1
            phase = (self.phase + offset) % 1.
            stride = (speed + side * steer) * stride_gain * duty / frequency
            if phase < duty:
                x = stride * (.5 - phase / duty)
                up = 0.
            else:
                swing = (phase - duty) / (1 - duty)
                x = -.5 * stride * cos(pi * swing)
                up = lift * sin(pi * swing)**2
            depth = height - up + side * roll_gain * roll - front * pitch_gain * pitch
            depth = float(np.clip(depth, .19, .37))
            knee = -2 * asin(sqrt(max(0., 1 - min(.999, (x*x + depth*depth)/(4*LENGTH*LENGTH)))))
            thigh = atan2(-x, depth) - knee / 2
            result.extend((side * width + hip_gain * roll, thigh, knee))
        return HOME + self.blend * (np.array(result) - HOME)
