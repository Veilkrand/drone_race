import math
from geometry_msgs.msg import Point, Quaternion, Vector3
from scipy.interpolate import interp1d
import numpy as np
import tf.transformations

def distance(point1, point2):
    return math.sqrt( (point2.x-point1.x)**2 + (point2.y-point1.y)**2 + (point2.z-point1.z)**2 )

def vector_to_list(vec):
    return [vec.x, vec.y, vec.z]

def vector_from_to(p0, p1):
    v = Vector3()
    v.x = p1.x - p0.x
    v.y = p1.y - p0.y
    v.z = p1.z - p0.z
    return v

def unit_vector_from_to(point1, point2):
    return normalize(vector_from_to(point1, point2))

def magnitude_vector(vec):
    return math.sqrt(vec.x**2 + vec.y**2 + vec.z**2 )

def normalize(vec, magnitude=1.0):
    res = Vector3()
    m = magnitude_vector(vec)
    res.x = vec.x * magnitude / m
    res.y = vec.y * magnitude / m
    res.z = vec.z * magnitude / m
    return res

def dot(v1,v2):
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z

def scalar_multiply(a,v):
    res = Vector3()
    res.x = a * v.x
    res.y = a * v.y
    res.z = a * v.z
    return res

def point_plus_vector(point, vector):
    res = Vector3()
    res.x = point.x + vector.x
    res.y = point.y + vector.y
    res.z = point.z + vector.z
    return res

def point_to_vector(point):
    res = Vector3()
    res.x = point.x
    res.y = point.y
    res.z = point.z
    return res

# http://www.chrobotics.com/library/understanding-quaternions
# http://mathworld.wolfram.com/Quaternion.html
# http://kieranwynn.github.io/pyquaternion/
# http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Quaternion.html
# http://wiki.ros.org/tf2/Tutorials/Quaternions
# http://www.chrobotics.com/library/understanding-euler-angles
# http://docs.ros.org/jade/api/tf/html/python/transformations.html

def rpy_to_vector(roll, pitch, yaw, magnitude=1.0):
    v = Point()
    v.y = math.sin(yaw)*math.cos(pitch)
    v.x = math.cos(yaw)*math.cos(pitch)
    v.z = - math.sin(pitch)
    return normalize(v, magnitude)

def quaternion_to_vector(quaternion, magnitude=1.0):
    tf_quat = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
    tf_euler = tf.transformations.euler_from_quaternion(tf_quat)
    return rpy_to_vector(tf_euler[0],tf_euler[1],tf_euler[2], magnitude)

def rotate_vector_wrt_quaternion(quaternion, vector):
    p = np.array([vector.x, vector.y, vector.z, 0])
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    # p' = q * p * q'
    p_prime = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q, p),
                tf.transformations.quaternion_inverse(q))
    return Vector3(*(p_prime[:3].tolist()))

#def rotate_vector_wrt_quaternion(q, v):
#    tf_quat = np.array([q.x, q.y, q.z, q.w])
#    tf_vec = np.array([v.x, v.y, v.z])
#    result = np.matmul(tf.transformations.quaternion_matrix(tf_quat)[:3, :3], tf_vec)
#    return Vector3(*(result.tolist()))

def rpy_to_quat(roll, pitch, yaw):
    quaternion_array = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'rxyz')
    result = Quaternion()
    result.x = quaternion_array[0]
    result.y = quaternion_array[1]
    result.z = quaternion_array[2]
    result.w = quaternion_array[3]
    return result

def vector_to_quat(vec):
    """To fully determine the orientation represented by the resulting quaternion, this method will assume the top of objects would always facing up
    """
    np_vec = np.array([vec.x, vec.y, vec.z])
    norm = np.linalg.norm(np_vec)
    if norm < 1e-5:
        return np.array([0.0, 0.0, 0.0, 1.0])
    obj_x = np.array([vec.x, vec.y, vec.z]) / norm
    obj_z_t = np.array([0.0, 0.0, 1.0])
    obj_y = np.cross(obj_z_t, obj_x)
    obj_y /= np.linalg.norm(obj_y) 
    obj_z = np.cross(obj_x, obj_y)
    rot_mat = np.identity(4)
    rot_mat[:3,:3] = np.array([obj_x,
                               obj_y,
                               obj_z]).T
    q = tf.transformations.quaternion_from_matrix(rot_mat)
    return q / np.linalg.norm(q)
    

def points_to_spline(points):
    # returns "spline" data that can be used for mapping between path distance s and Point on the spline.
    # https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html
    distance_so_far = 0.0
    last_point = points[0]
    distances = []
    for point in points:
        distances.append(distance_so_far + distance(last_point, point))
        distance_so_far = distances[-1]
    return {'x': interp1d(distances, [p.x for p in points], kind='cubic', bounds_error=False, fill_value="extrapolate"),
            'y': interp1d(distances, [p.y for p in points], kind='cubic', bounds_error=False, fill_value="extrapolate"),
            'z': interp1d(distances, [p.z for p in points], kind='cubic', bounds_error=False, fill_value="extrapolate"),
            'max_distance': distances[-1]}

def spline_distance_to_point(spline, distance):
    res = Point()
    res.x = spline['x'](distance)
    res.y = spline['y'](distance)
    res.z = spline['z'](distance)
    return res

def spline_distance_to_tangent(spline,distance):
    return unit_vector_from_to(spline_distance_to_point(spline, distance - 0.1), spline_distance_to_point(spline, distance + 0.1))

def spline_distance_to_curvature(spline, distance):
    # Centripetal acceleration = curvature * speed^2 where both acceleration and curvature have type Vector3.
    # Use example speed of 1 m/s2 to calculate acceleration, then convert to curvature.
    # https://opentextbc.ca/physicstestbook2/chapter/centripetal-acceleration/
    # https://en.wikipedia.org/wiki/Differential_geometry_of_curves#Normal_or_curvature_vector
    velocity_before = spline_distance_to_tangent(spline, distance - 0.1)
    velocity_after = spline_distance_to_tangent(spline, distance - 0.1)
    acceleration = scalar_multiply(1/0.2, vector_from_to(velocity_before, velocity_after))
    curvature = acceleration # because acceleration calculated using speed of 1.0 and acceleration=curvature*speed^2
    return curvature

