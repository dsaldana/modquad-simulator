import rospy
import tf
from nav_msgs.msg import Odometry

class OdometryManager(object):
    def __init__(self, n, robot_sufij='/modquad'):
        self.n = n
        self.robot_sufij = robot_sufij
        ### Odometry for all robots. Dic{topic: (x,y,z)}
        # self.states = {robot_sufij + '%02d/odom' % (i + 1): None for i in range(n)}
        self._locations = [None for _ in range(n)]
        self._velocities = [None for _ in range(n)]
        self._orientations = [None for _ in range(n)]

        self._poses = [None for _ in range(n)]
        self._twists = [None for _ in range(n)]

    def subscribe(self):
        for i in range(self.n):
            # subscriber
            rospy.Subscriber(self.robot_sufij + '%02d/odom' % (i + 1), Odometry, self._callback_odom)

    def _callback_odom(self, odom):
        # state vector
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        # orientation
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # Linear velocities
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z

        ### Extract id from topic name
        topic = odom._connection_header['topic']
        # extract robot id from the topic name
        robot_id = int(topic[len(self.robot_sufij):len(self.robot_sufij) + 2]) - 1

        self._locations[robot_id] = (x, y, z)  # store the recent location
        self._orientations[robot_id] = euler
        self._velocities[robot_id] = (vx, vy, vz)  # store the recent location
        self._poses[robot_id] = odom.pose.pose
        self._twists[robot_id] = odom.twist.twist

    def get_poses(self):
        return self._poses

    def get_twists(self):
        return self._twists

    def get_locations(self):
        """
        :return: robot locations [(x1,y1,z1),...,(xn, yn,zn)] 
        """
        return self._locations

    def get_velocities(self):
        return self._velocities

    def get_robot_loc(self, r2):
        return self.get_locations()[r2]

    def get_orientations(self):
        return self._orientations
