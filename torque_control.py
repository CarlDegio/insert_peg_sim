import numpy as np


class Torque:
    def __init__(self, q, q_dot):
        self.q = q  # 3*1
        self.q_dot = q_dot  # 3*1
        self.length = np.array([0.18, 0.15, 0.113])  # len
        self.r = np.array([0.02, 0.015, 0.005])
        self.torque_limit = np.array([12.0, 12.0, 12.0])  # torque_limit,suit for gear
        self.mass = 0.1
        self.g = 10.0
        self.controller = PD(self.pos())
        self.goal = np.array([[0.25], [-0.15], [-1.55]])

    def torque_calc(self, q, q_dot, goal):
        """
        move the end-effector to goal position by PID. k_p&k_d need to be adjusted further
        :param q: joint pos, column vector
        :param q_dot: joint vel, column vector
        :param goal: end-effector pos, column vector
        :return: torque on every joint, column vector
        """
        self.goal = goal
        self.q = q
        self.q_dot = q_dot
        u = self.J().T * self.M_x() * self.controller.step(self.pos(), self.goal) + self.gravity()
        return u

    def pos(self):
        """
        the position of end-effector
        :return: end-effector pos, third dim is canged as 0 degree to the right
        """
        q = self.q
        l = self.length
        return np.mat([-l[0] * np.sin(q[0]) - l[1] * np.sin(q[0] + q[1]) - l[2] * np.sin(q[0] + q[1] + q[2]),
                       l[0] * np.cos(q[0]) + l[1] * np.cos(q[0] + q[1]) + l[2] * np.cos(q[0] + q[1] + q[2]),
                       q[0] + q[1] + q[2] + np.pi / 2], dtype=np.float64)

    def J(self):
        """
        Jacob Matrix
        :return:
        """
        q = self.q
        l = self.length
        jacob1 = [
            -l[0] * np.cos(q[0]) - l[1] * np.cos(q[0] + q[1]) - l[2] * np.cos(
                q[0] + q[1] + q[2]),
            - l[1] * np.cos(q[0] + q[1]) - l[2] * np.cos(q[0] + q[1] + q[2]),
            - l[2] * np.cos(q[0] + q[1] + q[2])]
        jacob2 = [
            -l[0] * np.sin(q[0]) - l[1] * np.sin(q[0] + q[1]) - l[2] * np.sin(
                q[0] + q[1] + q[2]),
            - l[1] * np.sin(q[0] + q[1]) - l[2] * np.sin(q[0] + q[1] + q[2]),
            - l[2] * np.sin(q[0] + q[1] + q[2])]
        jacob3 = np.array([1.0, 1.0, 1.0])
        return np.mat([jacob1, jacob2, jacob3], dtype=np.float64)

    def M(self):
        q = self.q
        l = self.length
        M1 = [3 * (self.r[0] ** 2 + self.r[1] ** 2 + self.r[2] ** 2) + 28 * l[0] ** 2 + 16 * l[1] ** 2 + 4 * l[
            2] ** 2 + 12 * l[0] * l[2] * np.cos(q[1] + q[2]) + 36 * l[0] * l[1] * np.cos(q[1]) + 12 * l[1] * l[
                  2] * np.cos(q[2]),
              3 * (self.r[1] ** 2 + self.r[2] ** 2) + 16 * l[1] ** 2 + 4 * l[2] ** 2 + 6 * l[0] * l[2] * np.cos(
                  q[1] + q[2]) + 18 * l[0] * l[1] * np.cos(q[1]) + 12 * l[1] * l[2] * np.cos(q[2]),
              3 * (self.r[2] ** 2) + 4 * l[2] ** 2 + 6 * l[0] * l[2] * np.cos(q[1] + q[2]) + 6 * l[1] * l[2] * np.cos(
                  q[2])]
        M2 = [3 * (self.r[1] ** 2 + self.r[2] ** 2) + 16 * l[1] ** 2 + 4 * l[2] ** 2 + 6 * l[0] * l[2] * np.cos(
            q[1] + q[2]) + 18 * l[0] * l[1] * np.cos(q[1]) + 12 * l[1] * l[2] * np.cos(q[2]),
              3 * (self.r[1] ** 2 + self.r[2] ** 2) + 16 * l[1] ** 2 + 4 * l[2] ** 2 + 12 * l[1] * l[2] * np.cos(q[2]),
              3 * (self.r[2] ** 2) + 4 * l[2] ** 2 + 6 * l[1] * l[2] * np.cos(q[2])]
        M3 = [3 * (self.r[2] ** 2) + 4 * l[2] ** 2 + 6 * l[0] * l[2] * np.cos(q[1] + q[2]) + 6 * l[1] * l[2] * np.cos(
            q[2]),
              3 * (self.r[2] ** 2) + 4 * l[2] ** 2 + 6 * l[1] * l[2] * np.cos(q[2]),
              3 * (self.r[2] ** 2) + 4 * l[2] ** 2]
        return self.mass / 12 * np.mat([M1, M2, M3], dtype=np.float64)

    def M_x(self):
        """
        inertia matrix under cartesian space (x,y,degree)
        :return:
        """
        Mx_inv = self.J() * np.linalg.inv(self.M()) * self.J().T
        if abs(np.linalg.det(self.J() * self.J().T)) > .005 ** 3:
            Mx = np.linalg.inv(Mx_inv)
        else:
            u, s, v = np.linalg.svd(Mx_inv)
            for i in range(len(s)):
                if s[i] < .005:
                    s[i] = 0
                else:
                    s[i] = 1.0 / float(s[i])
            Mx = np.dot(v, np.dot(np.diag(s), u.T))
        return Mx

    def gravity(self):
        q = self.q
        l = self.length
        G = -self.mass * self.g / 12 * np.mat(
            [30 * l[0] * np.sin(q[0]) + 18 * l[1] * np.sin(q[0] + q[1]) + 6 * l[2] * np.sin(q[0] + q[1] + q[2]),
             18 * l[1] * np.sin(q[0] + q[1]) + 6 * l[2] * np.sin(q[0] + q[1] + q[2]),
             6 * l[2] * np.sin(q[0] + q[1] + q[2])], dtype=np.float64)
        return G


class PD:
    def __init__(self, x):
        self.K_P = np.diag([100, 100, 100])
        self.K_D = np.diag([40, 40, 30])
        self.x = x
        self.x_last = x
        self.goal = np.zeros([3, 1])
        self.dt = 0.01  # same as control timestep, to calc differential

    def step(self, x, goal):
        self.goal = goal
        self.x_last = self.x
        self.x = x
        delta_x = self.goal - self.x
        P = np.dot(self.K_P, delta_x)
        # print(delta_x)
        D = np.dot(self.K_D, (np.zeros([3, 1]) - (self.x - self.x_last) / self.dt))
        return P + D
