#!/usr/bin/python

"""
This is a node for visualizing any trajectory_msgs/MultiDOFJointTrajectory messages.
"""

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import ColorRGBA
import math
import numpy as np
import tf

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
  for i in range(0, len(traj.points), 3):
    pt = traj.points[i]
    translation = pt.transforms[0].translation
    linear = pt.velocities[0].linear
    norm = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
    drone_x = np.array([linear.x / norm, linear.y / norm, linear.z / norm])
    drone_z_t = np.array([0, 0, 1])
    drone_y = np.cross(drone_z_t, drone_x)
    drone_z = np.cross(drone_x, drone_y)
    rot_mat = np.identity(4)
    rot_mat[:3,:3] = np.array([drone_x,
                               drone_y,
                               drone_z]).T
    q = tf.transformations.quaternion_from_matrix(rot_mat)
    velocity_viz = Marker()
    velocity_viz.header.frame_id = "world"
    velocity_viz.header.stamp = rospy.Time.now()
    velocity_viz.id = next(id_gen)
    velocity_viz.type = Marker.ARROW
    velocity_viz.action = Marker.ADD
    velocity_viz.pose.position = translation
    velocity_viz.pose.orientation = Quaternion(*(q.tolist()))
    velocity_viz.scale = Vector3(norm * 3, 0.05, 0.05)
    velocity_viz.color = ColorRGBA(1, 0, 0, 0.5)

    traj_viz.markers.append(velocity_viz)
    
  viz_pub.publish(traj_viz)

if __name__ == "__main__":
  rospy.init_node("traj_viz")
  rate = rospy.Rate(1)
  trajectory_message_name = rospy.resolve_name("~trajectory")
  published_topic_name = rospy.resolve_name("~trajectory_viz")
  traj_sub = rospy.Subscriber(trajectory_message_name, MultiDOFJointTrajectory, trajectory_callback)
  viz_pub = rospy.Publisher(published_topic_name, MarkerArray, queue_size=1)
  while not rospy.is_shutdown():
    rate.sleep()
