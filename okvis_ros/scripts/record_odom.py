#!/usr/bin/python

from nav_msgs.msg import Odometry
import rospy
import argparse


def odom_callback(odom_msg: Odometry, odom_file: str):
    position = odom_msg.pose.pose.position
    quaterion = odom_msg.pose.pose.orientation
    stamp = str(odom_msg.header.stamp.to_sec())
    with open(odom_file, "a") as f:
        f.write(
            "{} {} {} {} {} {} {} {}\n".format(
                stamp,
                position.x,
                position.y,
                position.z,
                quaterion.x,
                quaterion.y,
                quaterion.z,
                quaterion.w,
            )
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="odom_recorder",
        description="Records ROS odometry message and saves to text file in TUM format.",
        add_help=True,
    )

    odom_topic = "/okvis_node/okvis_odometry"
    parser.add_argument("--output_file", "-o", help="output  file", required=True)

    rospy.init_node("odom_subscriber", anonymous=True)

    args = parser.parse_args()
    with open(args.output_file, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")

    print("Odometry topic: {}".format(odom_topic))
    print("Recording odometry to {}".format(args.output_file))
    odom_sub = rospy.Subscriber(
        odom_topic, Odometry, odom_callback, args.output_file, queue_size=100
    )

    while not rospy.is_shutdown():
        rospy.spin()
