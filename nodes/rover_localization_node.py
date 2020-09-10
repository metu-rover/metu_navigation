#!/usr/bin/env python
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose2D, Twist, TwistStamped
from leo_rover_localization.srv import SetReferancePose, SetDestinationResponse
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
    rel, ref, prv = args
    msg_x = msg.pose.pose.position.x
    msg_y = msg.pose.pose.position.y
    msg_theta = Quad2Euler(msg.pose.pose.orientation)[2]

    if (math.sqrt((prv.x - msg_x)**2 + (prv.y - msg_y)**2) < gamma):
        rel.x = msg_x - ref.x
        rel.y = msg_y - ref.y

    if (abs(msg_theta - prv.theta) < math.pi/150):
        rel.theta = msg_theta - ref.theta

    prv.x = msg_x
    prv.y = msg_y
    prv.theta = msg_theta


def callback_locomotion(msg, vel):
    vel.linear.x = msg.twist.linear.x
    vel.angular.z = msg.twist.angular.z


def callback_artag_marker(msg, args):
    rel, world, vel = args

    if abs(vel.linear.x) < epsilon and abs(vel.angular.z) < 2 * epsilon:
        world.x = msg.transform.translation.x - rel.x
        world.y = msg.transform.translation.y - rel.y


def handle_taring_the_balance(msg, ref):
    msg.referance


if __name__ == '__main__':
    rospy.init_node('rover_localization', anonymous=True)

    vel = Twist()
    temp_pose = Pose2D()
    rover_pose = Pose2D()  # msg
    relative_pose = Pose2D()  # rel
    world_pose = Pose2D()  # world
    reference_pose = Pose2D()  # tar

    # rufat's node
    rospy.Subscriber('odometry/filtered', Odometry,
                     callback_localization, (relative_pose, reference_pose, temp_pose))

    # Arda's node
    rospy.Subscriber('/base_link_transform', TransformStamped,
                     callback_artag_marker, (relative_pose, world_pose, vel))
    rospy.Subscriber('/wheel_odom', TwistStamped, callback_locomotion, vel)
    rospy.Service('taring_the_balance', SetReferancePose,
                  handle_taring_the_balance)
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
        # init + A_0 + (R_cur - R_0)
        rover_pose.x = world_pose.x + relative_pose.x
        rover_pose.y = world_pose.y + relative_pose.y
        rover_pose.theta = relative_pose.theta

        pub.publish(rover_pose)

        rate.sleep()
