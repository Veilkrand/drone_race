#!/usr/bin/python

import rospy
import sys
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion, Twist, Point, Vector3
import tf
import math

if __name__ == '__main__':
  if len(sys.argv) < 4:
    print("Usage: rosrun cascaded_pid_control set_point.py target_x target_y target_z [target_yaw]")
    exit(-1)

  rospy.init_node("SetPoint")
  x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
  yaw = 0
  if len(sys.argv) > 4:
    yaw = float(sys.argv[4])

  print("Will set location of the drone to {}, {}, {}. Yaw: {} degree".format(x, y, z, yaw))
  
  traj_pub = rospy.Publisher("/CascadedPidControl/trajectory", MultiDOFJointTrajectory, queue_size=1, latch=True)
  trajectory = MultiDOFJointTrajectory()
  trajectory.header.frame_id = 'world'
  trajectory.header.stamp = rospy.Time.now()
  trajectory.joint_names = ['base']
  transform = Transform()
  transform.translation = Vector3(x, y, z)
  transform.rotation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw * math.pi / 180.0, 'rxyz').tolist())

  trajectory.points.append(MultiDOFJointTrajectoryPoint([transform], [Twist()], [Twist()], rospy.Duration(0)))
  traj_pub.publish(trajectory)
  while not rospy.is_shutdown():
    rospy.sleep(rospy.Duration(3))
    rospy.signal_shutdown("Job done.")
    rospy.spin()
