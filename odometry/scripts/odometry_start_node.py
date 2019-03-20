#!/usr/bin/python

"""
This node reads the drone's starting location and orientation from the
/uav/flightgoggles_uav_dynamics/init_pose parameter and publishes in
odometry format to initialize odometry.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose

class OdometryStartNode(object):
  def __init__(self):
    self.init_pose_param = None

  def start(self):
    rospy.wait_for_service('set_pose')
    set_pose_service = rospy.ServiceProxy('set_pose', SetPose)

    self.init_pose_param = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")

    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'

    pose.pose.pose.position.x = self.init_pose_param[0]
    pose.pose.pose.position.y = self.init_pose_param[1]
    pose.pose.pose.position.z = self.init_pose_param[2]
    pose.pose.pose.orientation.x = self.init_pose_param[3]
    pose.pose.pose.orientation.y = self.init_pose_param[4]
    pose.pose.pose.orientation.z = self.init_pose_param[5]
    pose.pose.pose.orientation.w = self.init_pose_param[6]

    for i in range(6):
      pose.pose.covariance[i*6 + i] = 0.00001

    set_pose_service(pose)
    rospy.loginfo("finished setting initial pose: " + str(self.init_pose_param))

    rospy.spin()

if __name__ == '__main__':
  rospy.loginfo("Starting the odometry start node")
  rospy.init_node("odometry_start", anonymous=True)
  node = OdometryStartNode()
  node.start()

