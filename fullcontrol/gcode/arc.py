from typing import Optional
from fullcontrol.common import Arc as BaseArc
from fullcontrol.gcode import Point

class Arc(BaseArc):
    # gcode additions to Arc class

    def gcode(self, state):
        'process this instance in a list of steps supplied by the designer to generate and return a line of gcode'

        result = ""
        start = Point(x=self.x0, y=self.y0, z=self.z)
        XYZ_str = start.gcode(state)
        if XYZ_str != None:
            result += XYZ_str + "\n"

        end = Point(x=self.x, y=self.y, z=self.z)
        XYZ_str = end.XYZ_gcode(start)
        if XYZ_str != None:  # only write a line of gcode if movement occurs
            G_str = 'G3 ' if self.a >= 0 else 'G2 ' # cannot do arc travels so just assume extruder is on
            F_str = state.printer.f_gcode(state)
            E_str = state.extruder.e_gcode(Point(x=self.x0, y=self.y0, z=self.z+self.arclength), state)
            R_str = f'R{self.r} '
            gcode_str = f'{G_str}{F_str}{XYZ_str}{R_str}{E_str}'
            state.printer.speed_changed = False
            state.point.update_from(end)
            result += gcode_str.strip()  # strip the final space

        return result
