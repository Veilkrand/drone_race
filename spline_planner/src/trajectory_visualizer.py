#!/usr/bin/python

"""
This is a node for visualizing any trajectory_msgs/MultiDOFJointTrajectory messages.
"""

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import ColorRGBA

def id_generator(start):
  yield start
  start += 1

def trajectory_callback(traj):
  id_gen = id_generator(1)
  path_viz = Marker()
  path_viz.header.frame_id = 'world'
  path_viz.header.stamp = rospy.Time.now()
  path_viz.id = next(id_gen)
  path_viz.scale = Vector3(0.1, 0, 0)
  path_viz.color = ColorRGBA(0, 1, 0, 1)    
  path_viz.type = Marker.LINE_STRIP
  path_viz.action = Marker.ADD
  for pt in traj.points:
    path_viz.points.append(pt.transforms[0].translation)
  viz_pub.publish(path_viz)

if __name__ == "__main__":
  rospy.init_node("traj_viz")
  rate = rospy.Rate(1)
  trajectory_message_name = rospy.resolve_name("~trajectory")
  published_topic_name = rospy.resolve_name("~trajectory_viz")
  traj_sub = rospy.Subscriber(trajectory_message_name, MultiDOFJointTrajectory, trajectory_callback)
  viz_pub = rospy.Publisher(published_topic_name, Marker, queue_size=1)
  while not rospy.is_shutdown():
    rate.sleep()