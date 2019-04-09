#!/usr/bin/python

"""
Determine location by comparing IR markers to gate nominal positions.
"""

import rospy
import math
from flightgoggles.msg import IRMarkerArray
from nav_msgs.msg import Odometry

class OdometryGateNode(object):
  def __init__(self):
    self.odometry_subscriber = None
    self.irmarker_subscriber = None
    self.last_odometry = None
    self.gt_odometry = None
    self.nominal = {}
    self.image_height = rospy.get_param("/uav/flightgoggles_ros_bridge/image_height") # 768
    self.image_width = rospy.get_param("/uav/flightgoggles_ros_bridge/image_width") # 1024

    self.dist_to_inv_pixel_ratio = 1.0
    self.height_to_pixel_dist_ratio = 1.0
    self.width_to_pixel_dist_ratio = 1.0

    self.ratios = {}
    for i in range(1,5):
      self.ratios[i] = {'min': 10000, 'max': -10000}

  def start(self):
    self.odometry_subscriber = rospy.Subscriber("/odometry/map", Odometry, self.odometry_callback)
    self.gt_odometry_subscriber = rospy.Subscriber("/CheatOdometryNode/odometry", Odometry, self.gt_odometry_callback) # TODO: remove later
    self.irmarker_subscriber = rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, self.irmarker_callback)
    rate = rospy.spin()

  def odometry_callback(self, odometry):
    self.last_odometry = odometry

  def gt_odometry_callback(self, odometry):
    self.gt_odometry = odometry

  def irmarker_callback(self, irmarkers):
    if self.last_odometry is None:
      return
    gates_input = {}
    for marker in irmarkers.markers:
      name = marker.landmarkID.data
      if not gates_input.has_key(name):
        gates_input[name] = []
      gates_input[name].append(marker)
    for name in gates_input.keys():
      spread = self.pixel_spread(gates_input[name])
      drone_pos = self.gt_odometry.pose.pose.position # TODO: switch to measured odometry
      dist = self.dist(self.get_nominal(name)['location'], [drone_pos.x, drone_pos.y, drone_pos.z])
      width = self.get_nominal(name)['width']
      ratio = dist * spread / width
      num_points = len(gates_input[name])
      estimated_distance = self.estimate_gate_distance(name, gates_input[name])
      #rospy.loginfo("points=" + str(num_points) + " ratio=" + str(ratio) + " spread=" + str(spread) + " dist=" + str(dist))
      if estimated_distance['min_distance'] > dist or estimated_distance['max_distance'] < dist:
        rospy.loginfo("Distance " + str(dist) + " out of range (" + str(estimated_distance['min_distance']) + ", " + str(estimated_distance['max_distance']) + ")")
      if dist > 20:
        if ratio > self.ratios[num_points]['max']:
          self.ratios[num_points]['max'] = ratio
          self.ratios[num_points]['max_details'] = {'name': name, 'spread': spread, 'dist': dist}
        if ratio < self.ratios[num_points]['min']:
          self.ratios[num_points]['min'] = ratio
          self.ratios[num_points]['min_details'] = {'name': name, 'spread': spread, 'dist': dist}
      rospy.loginfo("cumulative: " + str(self.ratios))

  def estimate_gate_distance(self, gate_name, markers):
    ratios = {'min': {2: 93, 3: 441, 4: 514}, 'max': {2: 955, 3: 1168, 4: 1282}}
    width = self.get_nominal(gate_name)['width']
    spread = self.pixel_spread(markers)
    num_markers = len(markers)
    if num_markers < 2:
      return {'min_distance': 0, 'max_distance':1000}
    min_ratio = ratios['min'][num_markers]
    max_ratio = ratios['max'][num_markers]
    min_distance = min_ratio * width / spread
    max_distance = max_ratio * width / spread
    return {'min_distance': min_distance, 'max_distance': max_distance}

  def get_nominal(self, name):
    if self.nominal.has_key(name):
      return self.nominal[name]
    points = rospy.get_param("/uav/" + name + "/nominal_location") # list of 4 xyz points
    point_sum = [0,0,0]
    for point in points:
      for i in range(3):
        point_sum[i] += point[i]
    point_ave = [0,0,0]
    for i in range(3):
      point_ave[i] = point_sum[i] * 0.25
    width = 2 * self.dist(points[0],point_ave)
    self.nominal[name] = {'location': point_ave, 'width': width}
    return self.nominal[name]

  def pixel_spread(self, markers):
    max_dist = 0.0
    for i in range(len(markers) - 1):
      for j in range(i+1, len(markers)):
        m1 = markers[i]
        m2 = markers[j]
        dist = math.sqrt((m1.x - m2.x)**2 + (m1.y - m2.y)**2)
        if dist > max_dist:
          max_dist = dist
    return max_dist

  def dist(self, p1, p2):
    d2 = 0.0
    for i in range(len(p1)):
      d2 += (p1[i] - p2[i])**2
    return math.sqrt(d2)

if __name__ == '__main__':
  rospy.loginfo("Starting the odometry gate node")
  rospy.init_node("odometry_gate", anonymous=True)
  node = OdometryGateNode()
  node.start()

