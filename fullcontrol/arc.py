from typing import Optional
import numpy as np
import fullcontrol as fc
from fullcontrol.common import BaseModelPlus


class Arc(BaseModelPlus):

    cx: Optional[float] = None
    cy: Optional[float] = None
    cz: Optional[float] = None
    r: Optional[float] = None  # radius
    a0: Optional[float] = None  # start angle
    a: Optional[float] = None  # angle

    # next point in steps will request x, y, and z
    # these values are the endpoint of the arc
    @property
    def x(self):
        return self.cx + self.r * np.cos(self.a + self.a0)

    @property
    def y(self):
        return self.cy + self.r * np.sin(self.a + self.a0)

    @property
    def z(self):
        return self.cz

    @property
    def pointend(self):
        return fc.Point(x = self.x, y = self.y, z = self.z)


    # start point of arc
    @property
    def x0(self):
        return self.cx + self.r * np.cos(self.a0)

    @property
    def y0(self):
        return self.cy + self.r * np.sin(self.a0)

    @property
    def pointstart(self):
        return fc.Point(x=self.x0, y=self.y0, z=self.z)


    @property
    def arclength(self):
        return np.abs(self.r * self.a)

    @property
    def center(self):
        return fc.Point(x=self.cx, y=self.cy, z=self.cz)