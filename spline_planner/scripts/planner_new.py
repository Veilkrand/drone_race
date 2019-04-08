#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from spline_planner.cfg import PlannerConfig
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Point, Transform, Twist, Vector3, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from collections import deque
import tf
import numpy as np
import time

from spline import *


def vector3_to_ndarray(vec):
  return np.array([vec.x, vec.y, vec.z])

def point_to_ndarray(point):
  return np.array([point.x, point.y, point.z])

def quaternion_to_ndarray(q):
  return np.array([q.x, q.y, q.z, q.w])

def vec_to_quat(vec):
    """To fully determine the orientation represented by the resulting quaternion, this method will assume the top of objects would always facing up
    """
    norm = np.linalg.norm(vec)
    if norm < 1e-5:
        return np.array([0, 0, 0, 1], dtype=np.float)
    obj_x = vec / norm
    obj_z_t = np.array([0, 0, 1], dtype=np.float)
    obj_y = np.cross(obj_z_t, obj_x)
    obj_y /= np.linalg.norm(obj_y) 
    obj_z = np.cross(obj_x, obj_y)
    rot_mat = np.identity(4)
    rot_mat[:3,:3] = np.array([obj_x,
                               obj_y,
                               obj_z]).T
    q = tf.transformations.quaternion_from_matrix(rot_mat)
    return q / np.linalg.norm(q)


class SplinePlannerNew(object):
  def __init__(self):
    self.srv = None
    self.max_speed = 6.0
    self.max_acceleration = 3.0
    self.ds = 0.2
    self.gate_locations = []
    self.odometry_sub = None
    self.raw_waypoints_viz_pub = None # type: rospy.Publisher
    self.traj_pub = None # type: rospy.Publisher
    self.last_position = None
    self.current_time = None
    self.current_position = None
    self.current_velocity = None
    self.current_acceleration = None
    self.initial_position = None
    self.previous_gate_idx = None
    self.target_gate_idx = None
    self.gates_sequence = []
    self.current_path = None
    self.stop_planning = False
    self.waypoint_speeds = []

  def rprop_optimize(self, waypoints, d1, d2):
    new_d1 = d1.copy()
    start = time.time()

    min_k_path = None
    min_k = None
    t_start = None

    eta_inc = 1.2
    eta_dec = 0.5
    factor = 1
    delta = 0.1
    last_ks = []
    last_factors = []

    for i in range(1):
      path = propose_geometric_spline_path(waypoints, new_d1, d2)
      trajectory_points = sample_path([path[0]])
      trajectory_points, info = self.calculate_points_with_geometric_information(trajectory_points)
      print(factor, info['max_k'])
      if min_k is None or info['max_k'] < min_k:
        min_k = info['max_k']
        min_k_path = path

      last_factors.append(factor)
      last_ks.append(info['max_k'])
      if len(last_ks) > 3:
        del last_factors[0]
        del last_ks[0]

      if len(last_ks) == 1:
        t_start = trajectory_points[0]['d1']

      if len(last_ks) == 3:
        grad_1 = last_ks[-1] - last_ks[-2]
        grad_2 = last_ks[-2] - last_ks[-3]
        change = last_factors[-1] - last_factors[-2]
        if grad_1 * grad_2 > 0:
          change *= eta_inc
        else:
          change *= eta_dec

        delta = np.sign(grad_1) * change

      factor += delta
      new_d1[0] = t_start * factor
    end = time.time()
    rospy.loginfo("Time used: {}".format(end-start))
    trajectory_points = sample_path(min_k_path)
    trajectory_points, _ = self.calculate_points_with_geometric_information(trajectory_points)
    return trajectory_points

  def generate_trajectory_strategy_2(self):
    """Instead of using current position as the starting re-planning waypoint, this strategy don't
    use any information about the current position of the drone. It instead only use 3 nearby gate 
    positions (or the initial position if we're at the very beginning) to generate a new trajectory.
    """
    if self.previous_gate_idx is None:
      wp0 = self.initial_position
    else:
      wp0 = self.gate_locations[self.previous_gate_idx]['center']
    wp1 = self.get_waypoint_for_next_n_gate(1)
    wp2 = self.get_waypoint_for_next_n_gate(2)
    d1, d2 = {}, {}
    path = propose_geometric_spline_path((wp0, wp1, wp2))
    trajectory_points = sample_path(path)
    trajectory_points, info = self.calculate_points_with_geometric_information(trajectory_points)
    path = self.generate_velocity_profile(trajectory_points)
    self.current_path = path
    return path

  def generate_trajectory(self):
    # every time when re-planning, there are 3 raw waypoints:
    # the current position, position of the next gate, position of the next next gate
    #
    # in case that the drone is too close to 
    # the next gate, an alternate waypoint from the last trajectory is used instead of the position
    # of the next gate. the alternate waypoint must not be too close to the gate too.
    #
    # in case that we don't have a next next gate, a waypoint that is 5 meter away from the last gate
    # along the trajectory direction is used instead
    next_gate = self.get_waypoint_for_next_n_gate()
    wp0 = self.current_position
    d1 = {}
    d2 = {}
    if self.current_path is not None:
      skip_next_gate = False
      if np.linalg.norm(next_gate - wp0) <= 0.5:
        rospy.loginfo("Too close to next gate: {}. Will head for more future gates for waypoints.".format(self.target_gate_idx))
        skip_next_gate = True
      
      if skip_next_gate:
        wp1 = self.get_waypoint_for_next_n_gate(2)
        wp2 = self.get_waypoint_for_next_n_gate(3)
      else:
        wp1 = next_gate
        wp2 = self.get_waypoint_for_next_n_gate(2)
    else:
      rospy.loginfo("Planning for the first time")
      wp1 = next_gate
      wp2 = self.get_waypoint_for_next_n_gate(2)

    self.publish_raw_waypoints_viz(wp0, wp1, wp2)
    # path = propose_geometric_spline_path((wp0, wp1, wp2), d1, d2)
    # trajectory_points = sample_path(path)

    current_speed = np.linalg.norm(self.current_velocity)
    if current_speed > 0.01:
      d1[0] = self.current_velocity / current_speed * np.linalg.norm(wp1 - wp0) * 0.1
    
    #path = self.rprop_optimize((wp0, wp1, wp2), d1, d2)
    path = propose_geometric_spline_path((wp0, wp1, wp2), d1, d2)
    trajectory_points = sample_path(path)
    trajectory_points, info = self.calculate_points_with_geometric_information(trajectory_points)
    path = self.generate_velocity_profile(trajectory_points)
    self.current_path = path
    return path

  def search_for_nearest_waypoint(self, position):
    num_points = len(self.current_path)
    for i in range(num_points - 1, -1, -1):
        prev_pt = self.current_path[i - 1]["point"]
        next_pt = self.current_path[i]["point"]
        vec_ref = np.array(next_pt) - np.array(prev_pt)
        vec = np.array(next_pt) - np.array(position)
        if np.dot(vec, vec_ref) <= 0:
          return i
    return 0

  def get_waypoint_for_next_n_gate(self, n=1):
    if n == 1:
      return self.gate_locations[self.target_gate_idx]['center']
    if n - 1 > len(self.gates_sequence):
      if len(self.gates_sequence) <= 1:
        # no gates or only one gate left. get waypoint 5 meters along the direction from the current position to the target gate
        target_gate_loc = self.gate_locations[self.target_gate_idx]['center']
        direction = target_gate_loc - self.current_position
        direction /= np.linalg.norm(direction)
      else:
        # in this case, direction is defined as the last but 1 gate towards the last gate.
        target_gate_loc = self.gate_locations[self.gates_sequence[-1]]['center']
        direction = target_gate_loc - self.gate_locations[self.gates_sequence[-2]]['center']
        direction /= np.linalg.norm(direction)
      return target_gate_loc + 5 * direction
    else:
      return self.gate_locations[self.gates_sequence[n - 2]]['center']

  def generate_velocity_profile(self, points):
    # trajectory_points = self.calculate_points_with_geometric_information(points)
    trajectory_points = points

    trajectory_points[0]['speed'] = np.linalg.norm(self.current_velocity)
    trajectory_points[0]['velocity'] = self.current_velocity
    trajectory_points[0]['time'] = 0.0
    num_traj = len(trajectory_points)
    for i in range(num_traj - 1):
      prev_speed = trajectory_points[i]['speed']
      prev_time = trajectory_points[i]['time']
      ds = trajectory_points[i + 1]['ds']

      speed = self.calculate_safe_speed(trajectory_points[i + 1]['curvature'], prev_speed, ds)
      trajectory_points[i + 1]['speed'] = speed
      trajectory_points[i + 1]['velocity'] = speed * trajectory_points[i + 1]['unit_d1']

      avg_speed = 0.5 * (prev_speed + speed)
      current_time = prev_time + ds / avg_speed
      trajectory_points[i + 1]['time'] = current_time

      accel = (trajectory_points[i + 1]['velocity'] - trajectory_points[i]['velocity']) / (current_time - prev_time)
      trajectory_points[i]['acceleration'] = accel
    if len(trajectory_points) > 1:
      trajectory_points[-1]['acceleration'] = trajectory_points[-2]['acceleration']
    return trajectory_points


  def calculate_safe_speed(self, curvature, speed_neighbor, ds):
    centripetal = curvature * speed_neighbor**2
    if centripetal >= self.max_acceleration:
      return min(self.max_speed, np.sqrt(abs(self.max_acceleration / curvature)))

    remaining_acceleration = np.sqrt(self.max_acceleration**2 - centripetal**2)
    # see /Planning Motion Trajectories for Mobile Robots Using Splines/
    # (refered as Sprunk[2008] later) for more details (eq 3.21)
    v_this = np.sqrt(speed_neighbor ** 2 + 2 * ds * remaining_acceleration)
    return min(self.max_speed, v_this)

  def calculate_points_with_geometric_information(self, points):
    result = []
    info = {}
    max_k = 0
    this_point = None # type: {}
    last_point = None # type: {}
    for (s, t), (pt, d1, d2) in points:
      last_point = this_point
      # curvature is calcuated as norm(deriv1 x deriv2) / norm(deriv1)**3
      # see: https://en.wikipedia.org/wiki/Curvature#Local_expressions_2
      d1xd2 = np.cross(d1, d2)
      norm_d1 = np.linalg.norm(d1)
      norm_d2 = np.linalg.norm(d2)
      k = 0 # curvature
      if norm_d1 > 1e-5:
        k = np.linalg.norm(d1xd2) / norm_d1 ** 3

      # the first order derivative is given as ds/dt, where s is the arc length and t is the internal parameter of the spline,
      # not time.
      # because of this, the magnitude of first order derivative is not the same with viable speed,
      # but nevertheless, the direction of the derivative of them are the same.

      # also note that unit normal vector at point is just the normalized second order derivative,
      # it is in the opposite direction of the radius vector

      # the cross product of unit tangent vector and unit normal vector
      # is also mentioned as unit binormal vector
      if norm_d1 > 1e-5:
        unit_d1 = d1 / norm_d1
      else:
        unit_d1 = np.array([0, 0, 0], dtype=np.float)
      
      if norm_d2 > 1e-5:
        unit_d2 = d2 / norm_d2
      else:
        unit_d2 = np.array([0, 0, 0], dtype=np.float)

      unit_binormal = np.cross(unit_d1, unit_d2)

      ds = s - last_point['s'] if last_point is not None else 0.0

      this_point = {
        't': t,
        's': s,
        'ds': ds,
        'point': pt,
        'd1': d1, 
        'd2': d2,
        'unit_d1': unit_d1,
        'unit_d2': unit_d2,
        'unit_b': unit_binormal,
        'curvature': k
      }
      if k > max_k:
        max_k = k
      result.append(this_point)
    info['max_k'] = max_k
    return result, info

  def publish_raw_waypoints_viz(self, *waypoints):
    raw_waypoints_marker = Marker()
    raw_waypoints_marker.header.stamp = rospy.Time.now()
    raw_waypoints_marker.header.frame_id = "world"
    raw_waypoints_marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
    raw_waypoints_marker.scale = Vector3(0.5, 0.5, 0.5)
    raw_waypoints_marker.type = Marker.SPHERE_LIST
    raw_waypoints_marker.action = Marker.ADD
    raw_waypoints_marker.id = 1
    for wp in waypoints:
      if wp is not None:
        raw_waypoints_marker.points.append(Point(wp[0], wp[1], wp[2]))
    self.raw_waypoints_viz_pub.publish(raw_waypoints_marker)

  def publish_trajectory(self, trajectory):
    trajectory_msg = MultiDOFJointTrajectory()
    trajectory_msg.header.frame_id='world'
    trajectory_msg.joint_names = ['base']
    for idx in range(len(trajectory)):
      point = trajectory[idx]
      point['time'] = trajectory[idx]['time']
      transform = Transform()
      transform.translation = Vector3(*(point['point'].tolist()))
      transform.rotation = Quaternion(*(vec_to_quat(point['velocity']).tolist()))
      velocity = Twist()
      velocity.linear = Vector3(*(point['velocity'].tolist()))
      acceleration = Twist()
      acceleration.linear = Vector3(*(point['acceleration'].tolist()))
      
      trajectory_msg.points.append(MultiDOFJointTrajectoryPoint([transform], [velocity], [acceleration], rospy.Duration(point['time'])))

    trajectory_msg.header.stamp = rospy.Time.now()
    self.traj_pub.publish(trajectory_msg)

  def head_for_next_gate(self):
    if len(self.gates_sequence) == 0:
      rospy.loginfo("No next targeting gate.")
      self.target_gate_idx = None
      return False
    self.previous_gate_idx = self.target_gate_idx
    self.target_gate_idx = self.gates_sequence[0]
    del self.gates_sequence[0]
    rospy.loginfo("Next gate: {}".format(self.target_gate_idx))
    return True

  def is_cross_gate(self, gate_index, position_before, position_after):
    """Check if the drone has passed the target gate.

    To do this, make two vectors: from last position to gate and current position to gate,
    project them and the vector from gate center to gate edge(left/right edge) onto the XY-plane.
    By comparing the sign of the cross products of position-gate vector and gate-border vector,
    if they are not the same, then we have cross the gate.
    """
    if np.linalg.norm(position_after - position_before) < 1e-5:
        # the two positions are too close
        return False
    gate_position = self.gate_locations[gate_index]["center"]
    gate_proj_xy = self.gate_locations[gate_index]["gate_proj_xy"]

    position_before_xy = (position_before - gate_position)[:2]
    position_after_xy = (position_after - gate_position)[:2]

    return np.cross(position_before_xy, gate_proj_xy) * np.cross(position_after_xy, gate_proj_xy) < 0

  def odometry_callback(self, odometry):
    # type: (Odometry) -> None
    prev_time = self.current_time
    self.current_time = odometry.header.stamp.to_sec()

    self.last_position = self.current_position
    self.current_position = point_to_ndarray(odometry.pose.pose.position)

    prev_velocity = self.current_velocity
    self.current_velocity = SplinePlannerNew.rotate_vector_wrt_quaternion(
                                                quaternion_to_ndarray(odometry.pose.pose.orientation),
                                                vector3_to_ndarray(odometry.twist.twist.linear))

    if prev_velocity is not None and prev_time is not None:
      self.current_acceleration = (self.current_velocity - prev_velocity) / (self.current_time - prev_time)

    if self.last_position is None or self.target_gate_idx is None:
      return
    # check if the drone has passed the target gate
    if self.is_cross_gate(self.target_gate_idx, self.last_position, self.current_position):
      rospy.loginfo("Drone has passed gate {}".format(self.target_gate_idx))
      if self.head_for_next_gate():
        rospy.loginfo("Heading for gate {}".format(self.target_gate_idx))
      else:
        rospy.loginfo("All gates have been visited. Stop planning.")
        self.stop_planning = True

  @staticmethod
  def rotate_vector_wrt_quaternion(q, v):
    p = np.array([0, 0, 0, 0], dtype=np.float)
    p[0:3] = v
    # p' = q * p * q'
    p_prime = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q, p),
                tf.transformations.quaternion_inverse(q))
    return p_prime[:3]

  def reconfigure_parameteres(self, config, level):
    rospy.loginfo("""Parameters reconfiguration requested:
ds: {ds}
max_speed: {max_linear_speed}
max_total_acceleration: {max_total_acceleration}""".format(**config))
    self.max_speed = config.max_linear_speed
    self.max_acceleration = config.max_total_acceleration
    self.ds = config.ds

    return config

  def load_nominal_gates_locations(self):
    """Load nominal gates information from parameter server and store it as the 
    initial position of gates.
    """
    # for convenient, append a None at index 0 and let gate 1 be in index 1 of 
    # self.gate_locations
    self.gate_locations.append(None)
    num_total_gates = 23
    for i in range(1, num_total_gates + 1):
      nominal_gate_param_name = "/uav/Gate{}/nominal_location".format(i)
      corners = np.array(rospy.get_param(nominal_gate_param_name))
      norms = [np.linalg.norm(corners[1] - corners[0]),
               np.linalg.norm(corners[2] - corners[0]),
               np.linalg.norm(corners[3] - corners[0])]
      idx = np.argmax(norms)
      center = (corners[idx + 1] + corners[0]) / 2
      gate_proj_xy = (corners[idx + 1] - corners[0])[:2]
      self.gate_locations.append({
        "center": center,
        "gate_proj_xy": gate_proj_xy / np.linalg.norm(gate_proj_xy)
      })

  def load_course_gates(self):
    """Load gate sequence of course from parameter /uav/gate_names
    """
    gates = rospy.get_param("/uav/gate_names", None)
    
    if gates is None:
      rospy.logwarn("Unable to load course definition from /uav/gate_names")
      self.gates_sequence = []
      return

    self.gates_sequence = list(int(g.replace("Gate", ""), 10) for g in gates)
    rospy.loginfo("Course loaded. Gate sequence: {}".format(self.gates_sequence))

  def load_initial_pose(self):
    """Load initial pose of the drone"""
    pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
    px, py, pz, qx, qy, qz, qw = pose
    self.initial_position = np.array([px, py, pz], dtype=np.float)
    self.initial_orientation = np.array([qx, qy, qz, qw], dtype=np.float)

  def start(self, name="SplinePlannerNew"):
    rospy.init_node(name)
    self.srv = Server(PlannerConfig, self.reconfigure_parameteres)

    self.load_nominal_gates_locations()
    self.load_course_gates()
    self.load_initial_pose()
    
    self.odometry_sub = rospy.Subscriber("~odometry", Odometry, self.odometry_callback)
    self.traj_pub = rospy.Publisher("~trajectory", MultiDOFJointTrajectory, queue_size=1, latch=True)
    self.raw_waypoints_viz_pub = rospy.Publisher("~raw_waypoints_viz", Marker, queue_size=1, latch=True)
    rospy.loginfo("Planner node ready.")
    self.head_for_next_gate()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      if not self.stop_planning:
        if self.current_position is not None and self.target_gate_idx is not None:
          trajectory = self.generate_trajectory_strategy_2()
          if trajectory is None:
            rospy.logwarn("Failed to generate trajectory")
          else:
            self.publish_trajectory(trajectory)
        elif self.target_gate_idx is None:
          rospy.logwarn("No target gate. Planning aborted.")
        else:
          rospy.loginfo("Planner is still waiting for current position be initialized")
      rate.sleep()


if __name__ == '__main__':
  rospy.loginfo("Starting the new spline planner node")
  node = SplinePlannerNew()
  node.start()
