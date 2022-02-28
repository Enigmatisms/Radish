#-*-coding:utf-8-*-

import numpy as np
class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def R(self):
        cosa = np.cos(self.theta)
        sina = np.sin(self.theta)
        return np.array([[cosa, -sina], [sina, cosa]])
    
    def t(self):
        return np.array([[self.x], [self.y]])

    def inverse(self):
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)
        trans_x = self.x * cos_theta + self.y * sin_theta
        trans_y = self.x * sin_theta - self.y * cos_theta
        return Pose(-trans_x, trans_y, -self.theta)

    @staticmethod 
    def goodAngle(angle:float):
        if angle > np.pi:
            angle -= 2 * np.pi
        if angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def __mul__(self, p):
        trans = self.R() @ p.t() + self.t()
        return Pose(trans[0, 0], trans[1, 0], Pose.goodAngle(self.theta + p.theta))

    def __sub__(self, p):
        return Pose(self.x - p.x, self.y - p.y, Pose.goodAngle(self.theta - p.theta))

    def normSquared(self) -> float:
        return self.x ** 2 + self.y ** 2

    def norm(self) -> float:
        return np.sqrt(self.normSquared())

    def cWiseAbs(self):
        return Pose(abs(self.x), abs(self.y), abs(self.theta))

    def vec(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta]).reshape(-1, 1)