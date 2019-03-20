#!/usr/bin/python

"""
This node uses the drone's laser range finder, which is pointed down,
to estimate the drone's altitude. While the idea of determining height
based on a rangefinder pointed at the ground is fairly simple, there
are a variety of challenges:
  1) The rangefinder points down with respect to the drone body, which
     due to the drone's orientation is not directly toward the floor.
  2) There are two floors at heights of 1.0m and 9.7m, so that we might
     not know which floor we detected.
  3) Parts of the floor are raised due to slabs or debris, often by
     0-1m and occasionally by up to 3m, which substantially increases
     the uncertainty of our measurements.
"""

import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion

class OdometryAltimeterNode(object):

  def __init__(self):
    self.floor_heights = [1.0, 9.7]
    self.latest_odometry = None
    self.completed_launch = False

    self.max_range = rospy.get_param("/uav/flightgoggles_laser/rangefinder_max_range", 20)
    self.range_variance = rospy.get_param("/uav/flightgoggles_laser/rangefinder_variance", 0.009)

    init_pose_param = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
    self.init_position = init_pose_param[0:3]
    self.init_orientation = init_pose_param[-4:]

    self.range_subscriber = None
    self.odometry_subscriber = None
    self.altitude_publisher = None

  def start(self):
    self.range_subscriber = rospy.Subscriber("/uav/sensors/downward_laser_rangefinder", Range, self.range_callback)
    self.odometry_subscriber = rospy.Subscriber("/odometry/map", Odometry, self.odometry_callback)
    self.altitude_publisher = rospy.Publisher('/odometry/altitude', PoseWithCovarianceStamped, queue_size=1)
    rospy.spin()

  def odometry_callback(self, odometry):
    self.latest_odometry = odometry

  def range_callback(self, range_msg):
    distance = abs(range_msg.range)
    if not self.completed_launch and distance > 0.5 and self.latest_odometry is not None:
      self.completed_launch = True
    height_from_floor = distance
    floor_height = self.init_position[2]
    height_variance = self.range_variance

    if self.latest_odometry is not None:
      expected_height = self.latest_odometry.pose.pose.position.z

      orientation = self.latest_odometry.pose.pose.orientation
      orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
      (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
      range_to_height_ratio = max(0.1, abs(math.cos(roll) * math.cos(pitch)))
      height_from_floor = distance * range_to_height_ratio

      if self.completed_launch:
        expected_floor = expected_height - height_from_floor
        for floor in self.floor_heights:
          if abs(floor - expected_floor) < abs(floor_height - expected_floor):
            floor_height = floor

    # High uncertainty when at or near max range
    if distance > self.max_range * 0.9:
      height_variance += 100.0

    # Uncertainty due to debris and slabs is far greater than measurement uncertainty
    if self.completed_launch:
      height_variance += 0.25

    xy_variance = 1000000
    orientation_variance = 1000000
    if not self.completed_launch:
      xy_variance = 0.1 + height_from_floor**2
      orientation_variance = 0.1 + 100.0 * height_from_floor**2

    pose = PoseWithCovarianceStamped()
    pose.header.seq = range_msg.header.seq
    pose.header.stamp = range_msg.header.stamp
    pose.header.frame_id = 'map'

    if self.completed_launch:
      pose.pose.pose.position = self.latest_odometry.pose.pose.position
      pose.pose.pose.orientation = self.latest_odometry.pose.pose.orientation
    else:
      pose.pose.pose.position.x = self.init_position[0]
      pose.pose.pose.position.y = self.init_position[1]
      pose.pose.pose.orientation.x = self.init_orientation[0]
      pose.pose.pose.orientation.y = self.init_orientation[1]
      pose.pose.pose.orientation.z = self.init_orientation[2]
      pose.pose.pose.orientation.w = self.init_orientation[3]

    pose.pose.pose.position.z = floor_height + height_from_floor

    variances = [xy_variance] * 2 + [height_variance] + [orientation_variance] * 4

    for i in range(6):
      pose.pose.covariance[i*6 + i] = variances[i]

    self.altitude_publisher.publish(pose)

if __name__ == '__main__':
  rospy.loginfo("Starting the odometry altimeter node")
  rospy.init_node("odometry_altimeter", anonymous=True)
  node = OdometryAltimeterNode()
  node.start()

