#!/usr/bin/python

"""
This node is used only during development to measure our odometer's accuracy.
Depends on cascaded_pid_control/src/cheat_odometry_node.cpp to determine
correct odometry. The approach is to compare our odometry estimate in
/odometry/map to the ground truth in /CheatOdometryNode/odometry.
"""

from __future__ import print_function
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdometryDiagnosticNode(object):
  def __init__(self):
    self.odometry_actual = None
    self.odometry_measured = None

    self.odometry_actual_subscriber = None
    self.odometry_measured_subscriber = None

  def start(self):
    self.odometry_actual_subscriber = rospy.Subscriber("/CheatOdometryNode/odometry", Odometry, self.odometry_actual_callback)
    self.odometry_measured_subscriber = rospy.Subscriber("/odometry/map", Odometry, self.odometry_measured_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      if self.odometry_actual is not None and self.odometry_measured is not None:
        self.show_diagnostics(self.odometry_actual, self.odometry_measured)
      rate.sleep()

  def odometry_actual_callback(self, odometry):
    if self.odometry_actual is None:
      rospy.loginfo("Received ground truth odometry")
    self.odometry_actual = odometry

  def odometry_measured_callback(self, odometry):
    if self.odometry_measured is None:
      rospy.loginfo("Received measured odometry")
    self.odometry_measured = odometry

  def show_diagnostics(self, actual, measured):
    pos_actual = actual.pose.pose.position
    pos_measured = measured.pose.pose.position
    vel_actual = actual.twist.twist.linear
    vel_measured = measured.twist.twist.linear
    msg = "\nDiagnostics:"
    msg += "\n  Differences:"
    msg += "\n    Position: " + str(self.vector_distance(pos_actual, pos_measured))
    msg += "\n    Velocity: " + str(self.vector_distance(vel_actual, vel_measured))
    msg += "\n    Speed: " + str(abs(self.vector_magnitude(vel_actual) - self.vector_magnitude(vel_measured)))
    actual_rpy = self.odometry_to_rpy(actual)
    measured_rpy = self.odometry_to_rpy(measured)
    msg += "\n    Roll: " + str(abs(actual_rpy['roll'] - measured_rpy['roll']))
    msg += "\n    Pitch: " + str(abs(actual_rpy['pitch'] - measured_rpy['pitch']))
    msg += "\n    Yaw: " + str(abs(actual_rpy['yaw'] - measured_rpy['yaw']))
    msg += "\n  Absolute:"
    msg += "\n    POS actual:   " + str(pos_actual.x) + "  " + str(pos_actual.y) + "  " + str(pos_actual.z)
    msg += "\n        measured: " + str(pos_measured.x) + "  " + str(pos_measured.y) + "  " + str(pos_measured.z)
    msg += "\n    VEL actual:   " + str(vel_actual.x) + "  " + str(vel_actual.y) + "  " + str(vel_actual.z)
    msg += "\n        measured: " + str(vel_measured.x) + "  " + str(vel_measured.y) + "  " + str(vel_measured.z)
    msg += "\n    RPY actual:   " + str(actual_rpy)
    msg += "\n        measured: " + str(measured_rpy)
    msg += "\n"
    rospy.loginfo(msg)

  def vector_distance(self, v1, v2):
    return math.sqrt(pow((v1.x - v2.x),2) + pow((v1.y - v2.y),2) + pow((v1.z - v2.z),2))

  def vector_magnitude(self, v):
    return math.sqrt(pow(v.x,2) + pow(v.y,2) + pow(v.z,2))

  def odometry_to_rpy(self, odometry):
    orientation_quat = odometry.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

if __name__ == '__main__':
  rospy.loginfo("Starting the odometry diagnostic node")
  rospy.init_node("odometry_diagnostic", anonymous=True)
  node = OdometryDiagnosticNode()
  node.start()

