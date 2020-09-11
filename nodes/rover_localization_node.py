#!/usr/bin/env python
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose2D, Twist, TwistStamped
from leo_rover_localization.srv import SetReferencePose, SetReferencePoseResponse
from nav_msgs.msg import Odometry

epsilon = 0.05
gamma = 0.02


def Quad2Euler(q):
    """
    This function transforms from quernion to euler

    #Quaternion_to_Euler_Angles_Conversion
    For details, see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    Parameters
    ----------
    q : Pose().orientation
        This is an orientation vector in queternion form

    Returns
        The euler form of the parameter q
    -------
    int
        Description of return value
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
        # use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Transform the q of the form PoseStamped().pose.orientation to angle
    # by applying atan2(y,x) => float angle from math library.
    return roll, pitch, yaw


def callback_localization(msg, args):
    rel, ref, prev = args
    new_x = msg.pose.pose.position.x
    new_y = msg.pose.pose.position.y
    new_theta = Quad2Euler(msg.pose.pose.orientation)[2]

    if (math.sqrt((new_x - prev.x) ** 2 + (new_y - prev.y) ** 2) < gamma):
        rel.x = new_x - ref.x
        rel.y = new_y - ref.y

    if (abs(new_theta - prev.theta) < math.pi/150):
        rel.theta = new_theta - ref.theta

    prev.x = new_x
    prev.y = new_y
    prev.theta = new_theta


def callback_locomotion(msg, vel):
    vel.linear.x = msg.twist.linear.x
    vel.angular.z = msg.twist.angular.z


def callback_artag_marker(msg, args):
    rel, world, vel = args
    rospy.loginfo('[rover_localization] ')
    if abs(vel.linear.x) < epsilon and abs(vel.angular.z) < 2 * epsilon:
        world.x = msg.transform.translation.x - rel.x
        world.y = msg.transform.translation.y - rel.y


def handle_taring_the_balance(msg):
    global relative_pose, world_frame_pose
    rospy.loginfo('#handle_taring_the_balance responding...')
    world_frame_pose.x = msg.reference.x - relative_pose.x
    world_frame_pose.y = msg.reference.y - relative_pose.y
    rospy.loginfo('rover now at \{x:%4.2f, y:%4.2f, theta:%4.2f\}' % (
        msg.reference.x, msg.reference.y, msg.reference.theta))
    return SetReferencePoseResponse(True)


if __name__ == '__main__':
    rospy.init_node('rover_localization', anonymous=True)

    vel = Twist()
    temp_pose = Pose2D()
    rover_pose = Pose2D()
    relative_pose = Pose2D()
    reference_pose = Pose2D()
    world_frame_pose = Pose2D()

    # rufat's node
    rospy.Subscriber('odometry/filtered', Odometry,
                     callback_localization, (relative_pose, reference_pose, temp_pose))

    # Arda's node
    rospy.Subscriber('/base_link_transform', TransformStamped,
                     callback_artag_marker, (relative_pose, world_frame_pose, vel))
    rospy.Subscriber('/wheel_odom', TwistStamped, callback_locomotion, vel)
    rospy.Service('taring_the_balance', SetReferencePose, handle_taring_the_balance)
    rospy.loginfo_once('#taring_the_balance running @rover_localization')

    pub = rospy.Publisher('ground_truth_to_pose2D', Pose2D, queue_size=10)

    rospy.wait_for_message(
        '/leo_localization/odometry/filtered', Odometry, timeout=10)
    rospy.loginfo_once('[rover_localization] connected odometry/filtered')
    reference_pose.x = relative_pose.x
    reference_pose.y = relative_pose.y
    reference_pose.theta = relative_pose.theta

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        rover_pose.x = world_frame_pose.x + relative_pose.x
        rover_pose.y = world_frame_pose.y + relative_pose.y
        rover_pose.theta = relative_pose.theta

        pub.publish(rover_pose)

        rate.sleep()
