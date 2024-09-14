from typing import Optional, TYPE_CHECKING
from fullcontrol.arc import Arc as BaseArc
from fullcontrol.visualize.point import Point
from fullcontrol.visualize.controls import PlotControls
import numpy as np

if TYPE_CHECKING:
    from fullcontrol.visualize.state import State
    from fullcontrol.visualize.plot_data import PlotData

# there is an issue where the plot bounding box does not include arcs, since that calculation only looks at points

class Arc(BaseArc):

    def visualize(self, state: 'State', plot_data: 'PlotData', plot_controls: PlotControls):
        from fullcontrol.geometry.arcs import arcXY
        for i in arcXY(centre=Point(x=self.cx, y=self.cy, z=self.cz), radius=self.r, start_angle=self.a0, arc_angle=self.a, segments=np.max([int(self.arclength), 3])):
            i.visualize(state, plot_data, plot_controls)