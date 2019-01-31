import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform, Quaternion
import std_msgs.msg
from geometry_msgs.msg import Point
import tf

import time, math


class CommandPublisher:

    def __init__(self, _namespace):
        self._namespace = _namespace

        self.command_pub = rospy.Publisher('/' + _namespace + '/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

        while self.command_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            print("There is no subscriber available, trying again in 1 second.")
            time.sleep(1)

    def publish_point_command(self, x, y, z, yaw_deg):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))

        traj = MultiDOFJointTrajectory()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = 'frame'
        traj.joint_names.append('base_link')
        traj.header = header

        transforms = Transform(translation=Point(x, y, z),
                               rotation=Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        velocities = Twist()
        accelerations = Twist()
        point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Time(2))

        traj.points.append(point)

        self.command_pub.publish(traj)



    def publish_pose_command(self, x, y, z, rx, ry, rz):

        print('Command pose: '+self._namespace, x, y, z, rx, ry, rz )

        quaternion = tf.transformations.quaternion_from_euler(rx, ry, rz)

        traj = MultiDOFJointTrajectory()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = 'frame'
        traj.joint_names.append('base_link')
        traj.header = header

        transforms = Transform(translation=Point(x, y, z),
                               rotation=Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        velocities = Twist()
        # velocities.linear.x = 0
        # velocities.linear.y = 0
        # velocities.linear.z = 1
        accelerations = Twist()
        point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Time(2))

        traj.points.append(point)

        self.command_pub.publish(traj)

    def publish_pose_speed_command(self, x, y, z, rx, ry, rz, speed_vector):

        print('Command pose: '+self._namespace, x, y, z, rx, ry, rz, speed_vector )

        quaternion = tf.transformations.quaternion_from_euler(rx, ry, rz)
        rotation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])


        location = Point(x, y, z)

        transforms = Transform(translation=location,
                               rotation=rotation)

        velocities = Twist()
        velocities.linear = speed_vector
        accelerations = Twist()

        traj = MultiDOFJointTrajectory()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = 'frame'
        traj.joint_names.append('base_link')
        traj.header = header

        point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Time(0))


        traj.points.append(point)

        self.command_pub.publish(traj)


