import math

import tf
from geometry_msgs.msg import PoseStamped
import rospy
from std_srvs.srv import Empty


def goal_to_pose(x, y, z, yaw):
    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.frame_id = '/world'

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    print quaternion
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    return goal


def circular_motion():
    rospy.init_node('circular', anonymous=True)

    # Prefix
    prefix = 'modquad'
    # Goal publisher
    pubGoal = rospy.Publisher('/modquad01/goal', PoseStamped, queue_size=1)
    # Takeoff service
    takeoff_service = rospy.ServiceProxy('/modquad01/takeoff', Empty)

    # takeoff
    takeoff_service()

    # Time counter
    t = 1.
    s = 100.
    # Circle loop
    while not rospy.is_shutdown():
        pubGoal.publish(goal_to_pose(math.cos(t / s), math.sin(t / s), 1, 0.))

        t += 1
        rospy.sleep(.1)


if __name__ == '__main__':
    circular_motion()
