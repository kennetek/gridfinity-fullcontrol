
from fullcontrol.geometry import Point, interpolated_point, distance
from fullcontrol.common import linspace


def segmented_line(point1: Point, point2: Point, segments: int) -> list:
    'return a list of Points linearly spaced between the start Point and end Point. a total of Points in list = segments+1'
    x_steps = linspace(point1.x, point2.x, segments+1)
    y_steps = linspace(point1.y, point2.y, segments+1)
    z_steps = linspace(point1.z, point2.z, segments+1)
    return [Point(x=x_steps[i], y=y_steps[i], z=z_steps[i]) for i in range(segments+1)]


def segmented_path(points: list, segments: int) -> int:
    lengths = [distance(points[i], points[i+1])
               for i in range(len(points)-1)]
    cumulative_length = [0]
    for length in lengths:
        cumulative_length.append(cumulative_length[-1]+length)
    seg_length = cumulative_length[-1]/segments
    path_pts = [points[0]]
    path_length_now = 0
    path_section_now = 0
    for seg in range(segments-1):
        path_length_now += seg_length
        while path_length_now > cumulative_length[path_section_now]:
            path_section_now += 1
        interpolation_length = path_length_now - \
            cumulative_length[path_section_now-1]
        interpolation_fraction = interpolation_length / \
            distance(points[path_section_now-1], points[path_section_now])
        path_pts.append(interpolated_point(
            points[path_section_now-1], points[path_section_now], interpolation_fraction))
    path_pts.append(points[-1])
    return path_pts
