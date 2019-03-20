#!/usr/bin/python

"""
This node just splits the /odometry/filtered topic into two separate topics,
/odometry/odom and /odometry/map, based on frame_id.
"""

import rospy
from nav_msgs.msg import Odometry

class OdometrySplitNode(object):
  def __init__(self):
    self.odometry_subscriber = None
    self.odometry_odom_publisher = None
    self.odometry_map_publisher = None

  def start(self):
    self.odometry_subscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)
    
    self.odometry_odom_publisher = rospy.Publisher('/odometry/odom', Odometry, queue_size=1)
    self.odometry_map_publisher = rospy.Publisher('/odometry/map', Odometry, queue_size=1)

    rate = rospy.spin()

  def odometry_callback(self, odometry):
    if odometry.header.frame_id == 'odom':
      self.odometry_odom_publisher.publish(odometry)
    elif odometry.header.frame_id == 'map':
      self.odometry_map_publisher.publish(odometry)
    else:
      rospy.logerror('Unrecognized frame_id: ' + str(odometry.header.frame_id))

if __name__ == '__main__':
  rospy.loginfo("Starting the odometry split node")
  rospy.init_node("odometry_split", anonymous=True)
  node = OdometrySplitNode()
  node.start()

