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

import ros_geometry as geo
from spline import *


def vector3_to_ndarray(vec):
  return np.array([vec.x, vec.y, vec.z])

def point_to_ndarray(point):
  return np.array([point.x, point.y, point.z])

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
    self.current_position = None
    self.current_velocity = None
    self.target_gate_idx = None
    self.gates_sequence = [2, 13, 9, 1]
    self.stop_planning = False

  def generate_trajectory(self):
    waypoints = []
    look_ahead_time = 1
    # every time when re-planning, there are 3 raw waypoints:
    # the current position, position of the next gate, position of the next next gate
    # current position has to be looked ahead.
    #
    # in case that the drone is too close to 
    # the next gate, an alternate waypoint from the last trajectory is used instead of the position
    # of the next gate. the alternate waypoint must not be too close to the gate too.
    #
    # in case that we don't have a next next gate, a waypoint that is 5 meter away from the last gate
    # along the trajectory direction is used instead

    # TODO: currently look ahead / alternate waypoints selection are not implemented yet
    next_gate = self.gate_locations[self.target_gate_idx]['center']
    next_next_gate = self.gate_locations[self.gates_sequence[0]]['center']
    self.publish_raw_waypoints_viz(self.current_position, next_gate, next_next_gate)

    path = propose_geometric_spline_path(self.current_position, next_gate, next_next_gate)
    trajectory_points = sample_path(path)
    return self.generate_velocity_profile(trajectory_points)
  
  def generate_velocity_profile(self, points):
    trajectory_points = self.calculate_points_with_geometric_information(points)

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
      result.append(this_point)

    return result

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
      transform.translation = Point(*(point['point'].tolist()))
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
      return
    self.target_gate_idx = self.gates_sequence[0]
    del self.gates_sequence[0]
    rospy.loginfo("Next gate: {}".format(self.target_gate_idx))

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
    self.last_position = self.current_position
    self.current_position = point_to_ndarray(odometry.pose.pose.position)
    self.current_velocity = vector3_to_ndarray(odometry.twist.twist.linear)
    if self.last_position is None or self.target_gate_idx is None:
      return
    # check if the drone has passed the target gate
    if self.is_cross_gate(self.target_gate_idx, self.last_position, self.current_position):
      rospy.loginfo("Drone has passed gate {}".format(self.target_gate_idx))
      self.head_for_next_gate()

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

  def start(self, name="SplinePlannerNew"):
    rospy.init_node(name)
    self.srv = Server(PlannerConfig, self.reconfigure_parameteres)
    self.load_nominal_gates_locations()
    self.odometry_sub = rospy.Subscriber("~odometry", Odometry, self.odometry_callback)
    self.traj_pub = rospy.Publisher("~trajectory", MultiDOFJointTrajectory, queue_size=1, latch=True)
    self.raw_waypoints_viz_pub = rospy.Publisher("~raw_waypoints_viz", Marker, queue_size=1, latch=True)
    rospy.loginfo("Planner node ready.")
    self.head_for_next_gate()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      if not self.stop_planning:
        if self.current_position is not None and self.target_gate_idx is not None:
          trajectory = self.generate_trajectory()
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