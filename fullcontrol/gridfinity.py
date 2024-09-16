import fullcontrol as fc
import numpy as np

hp = 0.5 * np.pi


def arc(centre: fc.Point, radius: float, start_angle: float, arc_angle: float):
    return fc.Arc(cx=centre.x, cy=centre.y, cz=centre.z, r=radius, a0=start_angle, a=arc_angle)


def refx(point):
    return fc.reflectXY(point, fc.Point(x=0, y=0), fc.Point(x=1, y=0))


def refy(point):
    return fc.reflectXY(point, fc.Point(x=0, y=0), fc.Point(x=0, y=1))


def intersection(a_s, a_d, b_s, b_d):
    dx = b_s.x - a_s.x
    dy = b_s.y - a_s.y
    det = b_d.x * a_d.y - b_d.y * a_d.x
    u = (dy * b_d.x - dx * b_d.y) / det
    return fc.Point(x=a_s.x + a_d.x * u, y=a_s.y + a_d.y * u, z=a_s.z)


def ray(p, v, mag):
    m = np.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
    v = fc.Vector(x=v.x / m, y=v.y / m, z=v.z / m)
    return fc.Point(x=p.x + v.x * mag, y=p.y + v.y * mag, z=p.z + v.z * mag)


def arc_intersection(a: fc.Arc, p: fc.Point, v: fc.Vector):
    aa = v.x ** 2 + v.y ** 2
    bb = 2 * (v.x * (p.x - a.cx) + v.y * (p.y - a.cy))
    dd = (p.x - a.cx) ** 2 + (p.y - a.cy) ** 2 - a.r ** 2
    d = bb ** 2 - 4 * aa * dd

    if d < 0:
        return

    points = []
    sqrt_d = np.sqrt(d)
    t1 = (-bb + sqrt_d) / (2 * aa)
    t2 = (-bb - sqrt_d) / (2 * aa)

    for t in (t1, t2):
        x = p.x + t * v.x
        y = p.y + t * v.y
        points.append(fc.Point(x=x, y=y))

    if d == 0:
        points = [points[0]]

    points = [((np.angle((p.x - a.cx) + (p.y - a.cy) * 1j) - a.a0) + 2 * np.pi) % (2 * np.pi) for p in points]
    atrue = np.min(points) if a.a >= 0 else np.max(points)
    a.a = atrue if np.sign(a.a) > 0 else atrue - 2 * np.pi


def points_key(length, span, ro, z):
    point1 = fc.Point(x=0.5 * (length - 0.5) - ro, y=ro - 0.5 * (length * span - 0.5), z=z)
    return [point1, refx(point1), refy(refx(point1)), refy(point1)]


# radii = [0.8, 1.6, 1.6, 3.75]
# height = [0, 0.8, 2.6, 4.75]
def z2r(z):
    if z < 0:
        return 0
    if z < 0.8:
        return z + 0.8
    if z < 2.6:
        return 1.6
    if z < 4.75:
        return z - 1
    return 3.75


def z2s(z, height):
    s = 14
    if z < s:
        return s - np.sqrt(s ** 2 - (z - s) ** 2)
    if z > height:
        return -1
    return 0


def z2l(z, EW, HEIGHT):
    def out(i):
        return np.max([i, -EW]) - 0.15

    zz = z - HEIGHT + 2.85
    if zz < 0:
        return 0
    if zz < 1.65:
        return np.max([out(zz), 0])
    if zz < 2.85:
        return out(1.65)
    if zz < 3.55:
        return out(1.65 - (zz - 2.85))
    if zz < 5.35:
        return out(0.95)
    return out(0.95 - (zz - 5.35))


class Gridfinity:

    def __init__(self, options):
        self.EW = options['EW']
        self.EH = options['EH']
        self.LENGTH = options['LENGTH']
        self.HEIGHT = options['HEIGHT']
        self.SPAN = options['SPAN']
        self.nozzle_temp = options['nozzle_temp']
        self.bed_temp = options['bed_temp']
        self.print_speed = options['print_speed']
        self.print_speed_bridge = options['print_speed_bridge']
        self.fan_percent = options['fan_percent']
        self.printer_name = options['printer_name']
        self.initial_z = self.EH * 0.6

    def layer(self, length, ro, ri, z, gap, odd=False, io=0, pre=None, post=None, cont=False):
        # when given an original point, a radius, and an x distance, return a new point of the high intersection
        # return angle and new point
        def trig(point, x, r):
            return np.arcsin(x / r), fc.Point(x=point.x + x, y=point.y + np.sqrt(r * r - x * x), z=point.z)

        EW = self.EW
        span = self.SPAN
        if ro < 3.75:
            span = 1

        key = points_key(length, span, ro, z)
        point2 = fc.Point(x=0.5 * (length - 0.5) - 2 * EW - ri, y=0.5 * (length * span - 0.5) - 2 * EW - ri - gap, z=z)

        x = 0.5 * (length - 0.5) - 3 * EW - point2.x
        r = ri + EW
        theta, point3 = trig(point2, x, ri + EW)
        s = x / r
        xp = x + 1.5 * EW
        x = (s / (s + 1)) * (xp / s - ri - 1.5 * EW)
        theta, point4 = trig(point2, xp - x, ri + 1.5 * EW + x)
        rad7 = x

        inside_corners_first = [[0], [0, 1, 2]]
        inside_corners_last = [[1, 2, 3], [3]]
        substeps = [fc.Point(x=0.75 * EW, y=0.5 * (3 * EW - length * span + 0.5) + io, z=z), fc.Extruder(on=True)]
        if not cont:
            substeps.append(fc.PrinterCommand(id="unretract"))

        if io == 0:
            # bool for whether to do anything wacky
            do_bridge = (0.5 * (length - 0.5)) - (point2.y + ri) > 4 * EW

            # inside corner
            for i in inside_corners_first[odd]:
                if gap > 0:
                    if i == 2 and point4.y > key[2].y and do_bridge:
                        continue
                    if (i == 1 or i == 2) and not do_bridge:
                        continue
                substeps.append(arc(key[i], ro - 1.5 * EW - io, (i - 1) * hp, hp))

            if pre is not None:
                substeps += pre

            if gap > 0:
                # bridge
                bridge = []
                if do_bridge:
                    bridge.append(arc(point2, ri + 0.5 * EW, 0 * hp, hp))
                    bridge.append(arc(refy(point2), ri + 0.5 * EW, 1 * hp, theta))
                    bridge.append(arc(refy(point3), 0.5 * EW, theta + 3 * hp, -np.pi))
                    bridge.append(arc(refy(point2), ri + 1.5 * EW, theta + hp, -theta))

                    if point4.y > key[2].y:
                        ylast = bridge[-1].y
                        dia = 0.5 * (length - 0.5) - 1.5 * EW - ylast
                        point5 = fc.Point(x=refy(key[2]).x, y=ylast + 0.5 * dia, z=z)
                        bridge.append(arc(point5, 0.5 * dia, -hp, np.pi))
                    else:
                        bridge.append(arc(point2, ri + 1.5 * EW, hp, -theta))
                        bridge.append(arc(point4, rad7, -theta + 3 * hp, theta + hp))

                    if odd:
                        bridge = [refy(point) for point in bridge]
                        bridge = bridge[::-1]
                    substeps += bridge
                else:
                    # point6 = fc.Point(x=point2.x - EW, y=point2.y, z=z)
                    substeps.append(arc(point2, ri + 0.5 * EW, 0, hp))
                    substeps.append(arc(refy(point2), ri + 0.5 * EW, hp, hp))

            # inside corners late
            for i in inside_corners_last[odd]:
                if gap > 0:
                    if i == 1 and point4.y > key[2].y and do_bridge:
                        continue
                    if (i == 1 or i == 2) and not do_bridge:
                        continue
                if i == 3 and post is not None:
                    substeps += post
                substeps.append(arc(key[i], ro - 1.5 * EW - io, (i - 1) * hp, hp))
        else:
            off = [0, 0, 0, 0]
            if io > 0:
                off = [0, -io, -io, 0]
            for i in range(4):
                key_copy = fc.Point(x=key[i].x, y=key[i].y - off[i], z=key[i].z)
                substeps.append(arc(key_copy, ro - 1.5 * EW - io, (i - 1) * hp, hp))

        # seam avoidance
        substeps.append(fc.Point(x=0.5 * EW, y=0.5 * (3 * EW - length * span + 0.5) + io, z=z))
        substeps.append(fc.Extruder(on=False))
        # substeps.append(fc.PrinterCommand(id="retract"))
        substeps.append(fc.Point(x=0.5 * EW, y=0.5 * (EW - length * span + 0.5), z=z))
        substeps.append(fc.Extruder(on=True))
        # substeps.append(fc.PrinterCommand(id="unretract"))

        # print(io)
        if z < self.HEIGHT or io > -0.4 * EW:
            # outside corners
            for i in range(4):
                substeps.append(arc(key[i], ro - 0.5 * EW, 0.5 * (i - 1) * np.pi, 0.5 * np.pi))
            substeps.append(fc.Point(x=-0.25 * EW, y=0.5 * (EW - length * span + 0.5), z=z))

        substeps.append(fc.Extruder(on=False))
        if not cont:
            substeps.append(fc.PrinterCommand(id="retract"))
        return substeps

    def layer_bottom(self, length, ro, z, p, odd):
        EW = self.EW
        key = points_key(length, 1, ro, z)
        substeps = [
            fc.Point(x=(p - 0.5) * EW, y=0.5 * ((2 * p) * EW - length + 0.5), z=z + 0.1),
            fc.Extruder(on=True),
            fc.PrinterCommand(id="unretract")]
        for i in range(p)[::-1]:
            y = 0.5 * ((2 * i + 1) * EW - length + 0.5)
            substeps.append(fc.Point(x=(i + 0.5) * EW, y=y, z=z))
            for o in range(4):
                if ro - (i + 0.5) * EW > 0:
                    substeps.append(arc(key[o], ro - (i + 0.5) * EW, (o - 1) * hp, hp))
                else:
                    substeps.append(
                        fc.Point(x=(2 * ((0b1100 >> o) & 1) - 1) * y, y=(2 * ((0b1001 >> o) & 1) - 1) * y, z=z))
            substeps.append(fc.Point(x=(i - 0.5) * EW, y=0.5 * ((2 * i + 1) * EW - length + 0.5), z=z))
        substeps.append(fc.Extruder(on=False))
        substeps.append(fc.Point(z=z + 0.1))
        substeps += fc.move_polar(self.infill(length - 0.5 - 2 * p * EW, z), centre=fc.Point(x=0, y=0, z=z), radius=0,
                                  angle=odd * 0.5 * np.pi)
        substeps.append(fc.Extruder(on=False))
        substeps.append(fc.PrinterCommand(id="retract"))
        return substeps

    def layer_shelf(self, length, ro, z, gap, io=0.0, layer=0):
        EW = self.EW
        SPAN = self.SPAN
        EWb = 0.5
        key = points_key(length, SPAN, ro, z)
        ri = ro - 1.5 * EW - io
        yi = -0.5 * length * SPAN + 1.5 * EW + io - 0.5
        yend = yi + EWb * gap - 0.5 * EW - EWb

        def layer_shelf_corner():
            steps = []
            for i in [1, 2]:
                steps.append(arc(key[i], ri, (i - 1) * hp, hp))
                steps[-1].cy += io
            return steps

        def layer_shelf_end():
            steps = []
            steps.append(fc.Printer(print_speed=self.print_speed))

            # seam avoidance
            steps.append(fc.Point(x=0.5 * EW, y=yi - 0.5 * EW, z=z))
            steps.append(fc.Point(x=0.5 * EW, y=0.5 * (EW - length * SPAN + 0.5), z=z))

            # outside corners
            for i in range(4):
                steps.append(arc(centre=key[i], radius=ro - 0.5 * EW, start_angle=0.5 * (i - 1) * np.pi,
                                 arc_angle=0.5 * np.pi))
            steps.append(fc.Point(x=-0.25 * EW, y=0.5 * (EW - length * SPAN + 0.5), z=z))
            steps.append(fc.Extruder(on=False))
            steps.append(fc.PrinterCommand(id="retract"))

            return steps

        def layer_shelf_loop(point, vector, c1, c2, m):
            steps = []
            steps.append(fc.Printer(print_speed=self.print_speed))
            steps.append(fc.Fan(speed_percent=100))
            steps.append(fc.ManualGcode(text="M221 S95"))
            while True:
                points = [intersection(point, vector, c1, fc.Vector(x=0, y=1)),
                          intersection(point, vector, c1, fc.Vector(x=1, y=0)),
                          intersection(point, vector, c2, fc.Vector(x=0, y=1)),
                          intersection(point, vector, c2, fc.Vector(x=1, y=0))]
                l = list(
                    filter(lambda p: c2.x - 0.01 <= p.x <= c1.x + 0.01 and c1.y - 0.01 <= p.y <= c2.y + 0.01, points))
                if not l:
                    break
                l.sort(key=lambda p: fc.distance(p, point))
                for p in l:
                    steps.append(p)
                point = ray(point, vector, np.sign(l[0].y - point.y) * (fc.distance(l[-1], point) + 20))
                point = ray(point, fc.Vector(x=-1, y=m, z=0), 0.7 * EW)
            steps.append(fc.ManualGcode(text="M221 S95"))
            steps.append(fc.Fan(speed_percent=30))
            return steps

        def layer_shelf1():

            substeps = [fc.Point(x=0.75 * EW, y=yi - 0.5 * EWb, z=z), fc.Extruder(on=True),
                        fc.PrinterCommand(id="unretract"),
                        fc.Printer(print_speed=self.print_speed_bridge), fc.Fan(speed_percent=100),
                        fc.ManualGcode(text="M221 S105")]
            xp = 0
            for i in range(gap):
                odd = i % 2  # right is 0, left is 1
                od = odd * 2 - 1  # right is -1, left is 1
                yii = yi + i * EWb
                yp = np.max([key[0].y - yii, 0])
                xp = np.sqrt(np.abs(ri ** 2 - yp ** 2))
                substeps.append(arc(fc.Point(x=key[odd * 3].x - od * (xp + EW), y=yii, z=z), 0.5 * EWb, -hp,
                                    -od * (2 - ((i + 1) == gap)) * hp))
            substeps.append(fc.Point(x=key[0].x + xp, y=substeps[-1].y, z=z))
            substeps.append(fc.Fan(speed_percent=30))
            substeps.append(fc.ManualGcode(text="M221 S95"))
            substeps.append(fc.Printer(print_speed=self.print_speed * 0.6))
            substeps += layer_shelf_corner()

            substeps.append(fc.Printer(print_speed=self.print_speed))

            ya = yi + gap * EWb
            substeps.append(fc.Point(x=key[2].x - ri, y=ya, z=z))
            substeps.append(fc.Point(x=key[3].x - xp - 0.5 * (EWb + EW), y=ya, z=z))
            substeps.append(fc.Point(x=key[3].x - xp - 0.5 * (EWb + EW), y=yi - 0.5 * EW, z=z))

            substeps += layer_shelf_end()
            return substeps

        def layer_shelf2():

            substeps = [fc.Point(x=0.75 * EW, y=yi - 0.5 * EW, z=z), fc.Extruder(on=True),
                        fc.PrinterCommand(id="unretract")]
            vector = fc.Vector(x=-1, y=1, z=0)

            substeps.append(fc.Point(x=key[0].x + ri, y=yi - 0.5 * EW, z=z))
            point = fc.Point(x=key[0].x + ri, y=yend, z=z)
            substeps.append(point)

            point = ray(point, vector, 10)
            point = ray(point, fc.Vector(x=-1, y=-1, z=0), EW)

            c1 = fc.Point(x=key[0].x + ri - EW, y=yi + 0.5 * EW, z=0)
            c2 = fc.Point(x=key[3].x - ri + 2 * EW, y=yend, z=0)

            substeps += layer_shelf_loop(point, vector, c1, c2, -1)

            substeps.append(fc.Point(x=c2.x - EW, y=c1.y - 0.3, z=z))
            substeps.append(fc.Point(x=c2.x - EW, y=c2.y + EW - 0.3, z=z))
            substeps.append(fc.Point(x=key[0].x + ri, y=c2.y + EW - 0.3, z=z))

            substeps += layer_shelf_corner()

            substeps.append(fc.Point(x=key[3].x - ri, y=yi - 0.5 * EW, z=z))

            substeps += layer_shelf_end()

            return substeps

        def layer_shelf3():
            substeps = [fc.Point(x=0.75 * EW, y=yi - 0.5 * EW, z=z), fc.Extruder(on=True),
                        fc.PrinterCommand(id="unretract")]
            vector = fc.Vector(x=1, y=1, z=0)

            point = fc.Point(x=key[0].x, y=yi - 0.5 * EW, z=z)
            substeps.append(point)

            point = ray(point, vector, -10)

            c1 = fc.Point(x=key[0].x + ri, y=yi + 0.5 * EW, z=0)
            c2 = fc.Point(x=key[3].x - ri + EWb, y=yend, z=0)

            substeps += layer_shelf_loop(point, vector, c1, c2, 1)

            substeps.append(fc.Point(x=c2.x, y=c2.y - 0.3, z=z))
            substeps.append(fc.Point(x=c2.x, y=c2.y + EW - 0.3, z=z))
            substeps.append(fc.Point(x=c1.x, y=c2.y + EW - 0.3, z=z))

            substeps += layer_shelf_corner()

            substeps.append(fc.Point(x=key[2].x - ri, y=c2.y + 2 * EW, z=z))
            substeps.append(fc.Point(x=key[3].x - ri - 0.5 * (EWb + EW), y=c2.y + 2 * EW, z=z))
            substeps.append(fc.Point(x=key[3].x - ri - 0.5 * (EWb + EW), y=yi - 0.5 * EW, z=z))

            substeps += layer_shelf_end()

            return substeps

        func = [layer_shelf1, layer_shelf2, layer_shelf3]
        return func[layer]()

    def infill(self, length, z):
        d = np.sqrt(2) * length
        points1 = [fc.Point(x=x, y=0.5 * d - np.abs(x - 0.5 * d), z=z) for x in
                   np.linspace(0, d, int(np.ceil(d / (0.8 * self.EW))))]
        points2 = [refx(p) for p in points1]
        result = [fc.Point(x=points1[0].x, y=points1[0].y, z=z + 0.1), fc.Extruder(on=True)]
        for i in range(0, len(points1), 2):
            try:
                result.append(points1[i])
                result.append(points1[i + 1])
                result.append(points2[i + 1])
                result.append(points2[i + 2])
            except IndexError:
                break
        result = fc.move(result, fc.Vector(x=-0.5 * d), copy=False)
        result = fc.move_polar(result, fc.Point(x=0, y=0, z=z), 0, 0.25 * np.pi, copy=False)
        return result

    # bottom part of filling the gap between bases
    def bottomFillA(self, length, ro, z):
        EW = self.EW
        fill = []
        kf = points_key(length, 1, ro, z)
        of = z - 1 - ro

        arc1 = fc.Arc(cx=kf[1].x + of, cy=kf[1].y + of * 2, cz=z, r=ro - 1.5 * EW - of, a=hp, a0=0)
        fill.append(arc1)
        arc2 = refy(arc1)
        arc_intersection(arc2, fc.Point(x=-kf[1].x - ro + 2.5 * EW, y=kf[1].y, z=z), fc.Vector(x=0, y=1, z=0))
        fill.append(arc2)
        arc3 = fc.Arc(cx=kf[1].x + of, cy=kf[1].y + of * 2, cz=z, r=ro - 0.5 * EW - of, a=-hp, a0=hp)
        arc4 = fc.Arc(cx=kf[1].x + of, cy=kf[1].y + of * 2, cz=z, r=ro - 0.5 * EW - of, a=-hp, a0=hp)
        arc_intersection(arc3, fc.Point(x=kf[1].x + ro - 2.5 * EW, y=kf[1].y, z=z), fc.Vector(x=0, y=1, z=0))
        arc_intersection(arc4, fc.Point(x=kf[1].x + ro - 1.5 * EW, y=kf[1].y, z=z), fc.Vector(x=0, y=1, z=0))
        arc3 = refy(arc3)
        fill.append(arc3)
        fill.append(arc4)

        fill = fc.move(fill, fc.Vector(x=0, y=-self.LENGTH * 0.5, z=0), copy=False)
        return fill

    # top part of filling the gap between bases
    def bottomFillB(self, length, ro, z):
        fill = []
        kf = points_key(length, 1, ro, z)
        of = z - 1 - ro
        arc1 = fc.Arc(cx=kf[1].x + of, cy=kf[1].y + of * 2 - self.LENGTH * 0.5, cz=z,
                      r=ro - 1.5 * self.EW - of, a=hp, a0=0)
        fill.append(arc1)
        arc2 = refy(arc1)
        arc_intersection(arc2,
                         fc.Point(x=-kf[1].x - ro + 2.5 * self.EW, y=kf[1].y - self.LENGTH * 0.5, z=z),
                         fc.Vector(x=0, y=1, z=0))
        fill.append(arc2)
        fill.append(refx(arc2))
        fill.append(refx(arc1))
        return fill

    def generate(self, centerx, centery):
        initial_z = self.initial_z
        HEIGHT = self.HEIGHT
        EH = self.EH
        EW = self.EW
        LENGTH = self.LENGTH
        SPAN = self.SPAN

        steps = [fc.ManualGcode(text="M83")]

        layers = np.append(np.linspace(initial_z, HEIGHT, int(HEIGHT / EH)),
                           np.linspace(HEIGHT + 0.3, HEIGHT + 3.8, int((3.8 - 0.3) / EH)))
        step1 = layers[1] - layers[0]

        side = False
        n = 0
        first_layer_comp = 0.8 * EW
        shelf = 0
        shelf_layers = 2
        idx_continuous_layer = 0
        for z in layers:
            r = z2r(z)
            h = 0
            if n < 3:
                first = self.layer_bottom(34.5 + 2 * r - 2 * first_layer_comp, r, z, 3, n % 2)
                first = fc.move(first, fc.Vector(x=0, y=LENGTH * (SPAN - 1) * 0.5, z=0), copy=False)
                steps += first
                for i in range(SPAN - 1):
                    second = fc.move(first, fc.Vector(x=0, y=-LENGTH * (i + 1), z=0), copy=False)
                    steps += second
                first_layer_comp = 0
                h = z
                steps.append(fc.Fan(speed_percent=30))
            elif z < HEIGHT - 4 * step1 or z > HEIGHT:
                pre = []
                post = []
                if 2 <= idx_continuous_layer <= 4:
                    temp = self.bottomFillB(34.5 + 2 * r, r, z)
                    if idx_continuous_layer % 2 == 0:
                        temp_pre = fc.move(temp, fc.Vector(x=0, y=-0.5 * (SPAN - 2) * LENGTH, z=0))
                        for i in range(SPAN - 1):
                            pre += fc.move(temp_pre, fc.Vector(x=0, y=i * LENGTH, z=0))
                    else:
                        temp_post = fc.move([refx(refy(p)) for p in temp],
                                            fc.Vector(x=0, y=0.5 * (SPAN - 2) * LENGTH, z=0))
                        for i in range(SPAN - 1):
                            post += fc.move(temp_post, fc.Vector(x=0, y=-i * LENGTH, z=0))

                if r < 3.75:
                    first = self.layer(34.5 + 2 * r, r, r - 1.5 * EW, z, z2s(z - h, HEIGHT) - 0.5 * (41.5 - (34 + 2 * r)), side,
                                       z2l(z, EW, HEIGHT))
                    first = fc.move(first, fc.Vector(x=0, y=LENGTH * (SPAN - 1) * 0.5, z=0), copy=False)
                    steps += first
                    if SPAN > 1:
                        second = self.layer(34.5 + 2 * r, r, r - 1.5 * EW, z, 0, side, z2l(z, EW, HEIGHT))
                        second = fc.move(second, fc.Vector(x=0, y=LENGTH * (0.5 * (SPAN - 1) - 1), z=0), copy=False)
                        steps += second
                        for i in range(SPAN - 2):
                            third = fc.move(second, fc.Vector(x=0, y=-LENGTH * (i + 1), z=0), copy=False)
                            steps += third
                else:
                    if idx_continuous_layer == 0:
                        idx_continuous_layer += 1
                        pre = []
                        post = []
                        temp_pre = self.bottomFillA(34.5 + 2 * r, r, z)
                        temp_post = [refx(refy(p)) for p in temp_pre]
                        temp_pre = fc.move(temp_pre, fc.Vector(x=0, y=-0.5 * (SPAN - 2) * LENGTH, z=0))
                        temp_post = fc.move(temp_post, fc.Vector(x=0, y=0.5 * (SPAN - 2) * LENGTH, z=0))
                        for i in range(SPAN - 1):
                            pre += fc.move(temp_pre, fc.Vector(x=0, y=i * LENGTH, z=0))
                            post += fc.move(temp_post, fc.Vector(x=0, y=-i * LENGTH, z=0))
                    steps += self.layer(34.5 + 2 * r, r, r - 1.5 * EW, z,
                                        z2s(z - h, HEIGHT) - 0.5 * (41.5 - (34 + 2 * r)), side,
                                        z2l(z, EW, HEIGHT),
                                        pre, post, idx_continuous_layer != 0)

                if idx_continuous_layer > 0:
                    idx_continuous_layer += 1
            else:
                steps += self.layer_shelf(34.5 + 2 * r, r, z, 25, z2l(z, EW, HEIGHT), shelf)
                shelf_layers -= 1
                if shelf_layers <= 0:
                    shelf = (n % 2) + 1
                # break
            side = not side
            n += 1

        steps = fc.move(steps, fc.Vector(x=centerx, y=centery, z=0), copy=False)

        steps.append(fc.ManualGcode(text="G1 E-2.8 F2700"))
        return steps
