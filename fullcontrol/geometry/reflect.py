import numpy as np
from fullcontrol.geometry import Point
from fullcontrol.geometry import Arc


def reflectXY_mc(p: Point, m_reflect: float, c_reflect: float) -> Point:
    'reflect x and y values of a Point about a line defined by m and c. return new Point with original z'
    m_reflect_normal = -1 / m_reflect  # gradient of the normal of the reflection line
    # intercept of the normal of the reflection line
    c_reflect_normal = p.y - (m_reflect_normal * p.x)
    p_foot = Point(x=(c_reflect_normal - c_reflect) / (m_reflect - m_reflect_normal),  # See my onenote note (AG) - p_foot is the 'foot' of the original point on the reflection line
                   y=(c_reflect_normal - ((m_reflect_normal / m_reflect) * c_reflect)) / (1 - (m_reflect_normal / m_reflect)))
    return Point(x=p.x + 2 * (p_foot.x - p.x), y=p.y + 2 * (p_foot.y - p.y), z=p.z)


def reflectXY(p, p1_reflect: Point, p2_reflect: Point):
    'reflect x and y values of a Point about a line defined by two Points. return new Point with original z'
    if isinstance(p, Point):
        # the if and elif avoid numerical errors associated with calculating the gradient of a vertical line
        if p2_reflect.x - p1_reflect.x == 0:  # reflection line in Y direction
            return Point(x=p.x + 2 * (p1_reflect.x - p.x), y=p.y, z=p.z)
        elif p2_reflect.y - p1_reflect.y == 0:  # reflection line in X direction
            return Point(x=p.x, y=p.y + 2 * (p1_reflect.y - p.y), z=p.z)
        else:
            # gradient of reflection line
            m_reflect = (p2_reflect.y - p1_reflect.y) / \
                (p2_reflect.x - p1_reflect.x)
            # intercept of reflection line
            c_reflect = p1_reflect.y - (m_reflect * p1_reflect.x)
            return reflectXY_mc(p, m_reflect, c_reflect)
    elif isinstance(p, Arc): # maintains spin direction
        rp = reflectXY(p.center, p1_reflect, p2_reflect)
        p1 = p1_reflect
        p2 = p2_reflect
        if p2_reflect.y > p1_reflect.y:
            p1 = p2_reflect
            p2 = p1_reflect
        ar = np.angle(p1.x - p2.x + (p1.y - p2.y)*1j)
        ap = 2*ar - p.a0 - p.a
        return Arc(cx=rp.x, cy=rp.y, cz=rp.z, r=p.r, a=p.a, a0=ap)
    return p
