#!/usr/bin/python

"""
This is a node for visualizing any trajectory_msgs/MultiDOFJointTrajectory messages.
"""

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float32
from jsk_rviz_plugins.msg import OverlayText
import ros_geometry as geo
import math
import numpy as np
import tf

def magnitude(vec3):
  return math.sqrt(vec3.x ** 2 + vec3.y ** 2 + vec3.z ** 2)

def id_generator(start):
  while True:
    yield start
    start += 1

def trajectory_callback(traj):
  id_gen = id_generator(1)
  # path visualizing
  # waypoint position, connected with line
  traj_viz = MarkerArray()
  
  path_viz = Marker()
  path_viz.header.frame_id = 'world'
  path_viz.header.stamp = rospy.Time.now()
  path_viz.id = next(id_gen)
  path_viz.scale = Vector3(0.05, 0, 0)
  path_viz.color = ColorRGBA(0, 1, 0, 1)    
  path_viz.type = Marker.LINE_STRIP
  path_viz.action = Marker.ADD
  for pt in traj.points:
    path_viz.points.append(pt.transforms[0].translation)

  traj_viz.markers.append(path_viz)

  # waypoint direction. (linear velocity at waypoint)
  # the direction of linear velocity is the same as the x direction of the drone, which is also the
  # desired heading of the drone.
  #
  # to fully determine the orientation, we still need the direction of y or z axis of the drone
  # after that we can construct a rotation matrix, and then convert it into quaternion
  num_points = len(traj.points)
  num_velocity_viz = 5
  step = max(1, num_points / num_velocity_viz)
  i = 0
  while i < num_points:
    pt = traj.points[i]
    translation = pt.transforms[0].translation
    linear = pt.velocities[0].linear
    norm = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
    if norm < 1e-3:
      i += step
      continue
    q = geo.vector_to_quat(linear)
    velocity_viz = Marker()
    velocity_viz.header.frame_id = "world"
    velocity_viz.header.stamp = rospy.Time.now()
    velocity_viz.id = next(id_gen)
    velocity_viz.type = Marker.ARROW
    velocity_viz.action = Marker.ADD
    velocity_viz.pose.position = translation
    velocity_viz.pose.orientation = Quaternion(*(q.tolist()))
    velocity_viz.scale = Vector3(norm, 0.05, 0.05)
    velocity_viz.color = ColorRGBA(1, 0, 0, 0.5)

    traj_viz.markers.append(velocity_viz)

    i += step
  viz_pub.publish(traj_viz)

  # visualize information of the next 3 waypoints
  if num_points >= 3:
    p1 = traj.points[0]
    p2 = traj.points[1]
    p3 = traj.points[2]
    waypoints_info_text = OverlayText()
    waypoints_info_text.action = OverlayText.ADD
    waypoints_info_text.width = 128
    waypoints_info_text.height = 64
    waypoints_info_text.left = 0
    waypoints_info_text.top = 0
    waypoints_info_text.bg_color = ColorRGBA(0, 0, 0, 0.5)
    waypoints_info_text.fg_color = ColorRGBA(1, 1, 1, 1)
    waypoints_info_text.text_size = 8
    waypoints_info_text.text = """Next waypoint @ {:.2f}s: {:.2f} {:.2f} {:.2f}. Speed: {:.2f}
Next waypoint @ {:.2f}s: {:.2f} {:.2f} {:.2f}. Speed: {:.2f}
Next waypoint @ {:.2f}s: {:.2f} {:.2f} {:.2f}. Speed: {:.2f}
""".format(p1.time_from_start.to_sec(), p1.transforms[0].translation.x, p1.transforms[0].translation.y, p1.transforms[0].translation.z, magnitude(p1.velocities[0].linear),
           p2.time_from_start.to_sec(), p2.transforms[0].translation.x, p2.transforms[0].translation.y, p2.transforms[0].translation.z, magnitude(p2.velocities[0].linear),
           p3.time_from_start.to_sec(), p3.transforms[0].translation.x, p3.transforms[0].translation.y, p3.transforms[0].translation.z, magnitude(p3.velocities[0].linear),)
    waypoints_info_pub.publish(waypoints_info_text)
  

def odometry_callback(odometry):
  # speed info
  twist = odometry.twist.twist
  speed = magnitude(twist.linear)
  msg = Float32()
  msg.data = speed
  linear_speed_pub.publish(msg)

  # position info
  pos = odometry.pose.pose.position
  position_text = OverlayText()
  position_text.action = OverlayText.ADD
  position_text.width = 128
  position_text.height = 64
  position_text.left = 0
  position_text.top = 0
  position_text.bg_color = ColorRGBA(0, 0, 0, 0.5)
  position_text.fg_color = ColorRGBA(1, 1, 1, 1)
  position_text.text_size = 8
  position_text.text = "Current position: {:.2f} {:.2f} {:.2f}".format(pos.x, pos.y, pos.z)
  position_pub.publish(position_text)
  

if __name__ == "__main__":
  rospy.init_node("traj_viz")
  rate = rospy.Rate(60)
  
  trajectory_message_name = rospy.resolve_name("~trajectory")
  published_topic_name = rospy.resolve_name("~trajectory_viz")
  waypoints_info_topic_name = rospy.resolve_name("~waypoints_info")
  traj_sub = rospy.Subscriber(trajectory_message_name, MultiDOFJointTrajectory, trajectory_callback)
  viz_pub = rospy.Publisher(published_topic_name, MarkerArray, queue_size=1)
  waypoints_info_pub = rospy.Publisher(waypoints_info_topic_name, OverlayText, queue_size=1)
  
  odometry_message_name =  rospy.resolve_name("~odometry")
  linear_speed_topic_name = rospy.resolve_name("~linear_speed")
  position_topic_name = rospy.resolve_name("~position")
  odometry_sub = rospy.Subscriber(odometry_message_name, Odometry, odometry_callback)
  linear_speed_pub = rospy.Publisher(linear_speed_topic_name, Float32, queue_size=1)
  position_pub = rospy.Publisher(position_topic_name, OverlayText, queue_size=1)

  rospy.loginfo("Trajectory visualize node ready.")  
  while not rospy.is_shutdown():
    rate.sleep()
