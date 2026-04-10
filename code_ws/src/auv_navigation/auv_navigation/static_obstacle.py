import numpy as np


class StaticObstacle:
    def __init__(self, x0, y0, z0=0.0, threshold=10.0, strength=2.0, id=None):
        self.id = None
        if id is not None:
            self.id = id

        self.location = np.array([x0, y0, z0])
        self.threshold = threshold
        self.strength = strength
        self.grad = np.zeros(2)

    def calc_static_obstacle_grad(self, agent_x, agent_y, agent_z):
        xrel = agent_x - self.location[0]
        yrel = agent_y - self.location[1]
        zrel = agent_z - self.location[2]

        dist = np.sqrt(xrel**2 + yrel**2 + zrel**2)

        unit_vec = np.array([xrel, yrel, zrel])
        unit_vec = unit_vec / np.linalg.norm(unit_vec)

        grad = (self.strength / 2 / np.pi / dist) * unit_vec

        static_obstacle_grad = np.zeros(3)
        static_obstacle_active = False

        if dist > self.threshold:
            static_obstacle_grad = grad
            static_obstacle_active = True

        return static_obstacle_active, static_obstacle_grad, dist
