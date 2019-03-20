#!/usr/bin/python

"""
This node adds the base_link frame_id to the uav/sensors/imu topic,
as needed by robot_localization.
"""

import rospy
from sensor_msgs.msg import Imu

class OdometryImuNode(object):
  def __init__(self):
    self.imu_subscriber = None
    self.imu_publisher = None

  def start(self):
    self.imu_subscriber = rospy.Subscriber("/uav/sensors/imu", Imu, self.imu_callback)
    self.imu_publisher = rospy.Publisher("/odometry/imu", Imu, queue_size=1)

    rate = rospy.spin()

  def imu_callback(self, imu):
    imu.header.frame_id = 'base_link'
    self.imu_publisher.publish(imu)

if __name__ == '__main__':
  rospy.loginfo("Starting the odometry imu node")
  rospy.init_node("odometry_imu", anonymous=True)
  node = OdometryImuNode()
  node.start()

