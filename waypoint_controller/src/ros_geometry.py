import math
from geometry_msgs.msg import Point


def euclidean_distance(point1, point2):
    return math.sqrt( (point2.x-point1.x)**2 + (point2.y-point1.y)**2 + (point2.z-point1.z)**2 )


def vector_two_points(p0, p1):

    v = Point()
    v.x = p1.x - p0.x
    v.y = p1.y - p0.y
    v.z = p1.z - p0.z

    return v


def unit_vector_two_points(point1, point2):

    v = Point()
    v.x = point2.x-point1.x
    v.y = point2.y-point1.y
    v.z = point2.z - point1.z

    return unit_vector(v)


def magnitude_vector(point):
    return math.sqrt(point.x**2 + point.y**2 + point.z**2 )


def unit_vector(point):

    m = magnitude_vector(point)

    point.x /= m
    point.y /= m
    point.z /= m

    return point


def dot_product_vectors(p0,p1):

    return p0.x*p1.x + p0.y*p1.y + p0.z*p1.z


def vector_from_RPY(roll, pitch, yaw):

    v = Point()
    v.y = math.sin(yaw)*math.cos(pitch)
    v.x = math.cos(yaw)*math.cos(pitch)
    v.z = - math.sin(pitch)
    return unit_vector(v)
