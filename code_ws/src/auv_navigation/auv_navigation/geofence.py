import numpy as np


class GeoFence:
    def __init__(self, x0, y0, x1, y1, threshold=10.0, strength=2.0, id=None):
        self.id = None
        if id is not None:
            self.id = id

        self.start = np.array([x0, y0])
        self.end = np.array([x1, y1])

        self.m = (self.end[1] - self.start[1]) / (self.end[0] - self.start[0])
        self.c = self.start[1] - self.m * self.start[0]

        self.psi = np.arctan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

        self.threshold = threshold
        self.strength = strength

        self.grad = np.zeros(2)

    def calc_geofence_grad(self, agent_x, agent_y):
        xrel = agent_x - self.start[0]
        yrel = agent_y - self.start[1]

        xrel_end = self.end[0] - self.start[0]
        yrel_end = self.end[1] - self.start[1]

        xyrel = np.array([xrel, yrel])
        xyrel_end = np.array([xrel_end, yrel_end])

        R = np.array([[np.cos(self.psi), np.sin(self.psi)], [-np.sin(self.psi), np.cos(self.psi)]])

        xcyc = R @ xyrel
        xcyc_end = R @ xyrel_end

        xc = xcyc[0]
        yc = xcyc[1]

        xc_end = xcyc_end[0]
        yc_end = xcyc_end[1]

        grad_c = np.array([0, self.strength / 2 / np.pi / np.abs(yc)])
        grad = R.T @ grad_c

        geofence_grad = np.zeros(2)
        geofence_active = False

        if xc > 0.0 and xc < xc_end:
            if np.abs(yc) < self.threshold:
                geofence_grad = grad
                geofence_active = True

        return geofence_active, geofence_grad, np.abs(yc)
