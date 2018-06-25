from nav_msgs.msg import Odometry


def publish_odom(x, pub):
    """
    Convert quad state into an odometry message and publish it.
    :param x: 
    :param pub: 
    """
    # Roll pitch yaw trust
    odom = Odometry()
    odom.pose.pose.position.x = x[0]
    odom.pose.pose.position.y = x[1]
    odom.pose.pose.position.z = x[2]

    # Velocities
    odom.twist.twist.linear.x = x[3]
    odom.twist.twist.linear.y = x[4]
    odom.twist.twist.linear.z = x[5]

    # Orientation
    odom.pose.pose.orientation.x = x[6]
    odom.pose.pose.orientation.y = x[7]
    odom.pose.pose.orientation.z = x[8]
    odom.pose.pose.orientation.w = x[9]

    # Angular velocities
    odom.twist.twist.angular.x = x[10]
    odom.twist.twist.angular.y = x[11]
    odom.twist.twist.angular.z = x[12]

    odom.child_frame_id = 'modquad'
    odom.header.frame_id = 'simulator'

    pub.publish(odom)
