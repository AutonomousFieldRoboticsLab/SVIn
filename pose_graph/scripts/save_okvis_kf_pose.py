#!/usr/bin/env python


from email import header
import rospy
from nav_msgs.msg import Odometry
import rospkg
import os

package_path = rospkg.RosPack().get_path("pose_graph")
file_path = os.path.join(package_path, "okvis_kf_pose.txt")


def odom_callback(odom_msg: Odometry):
    trans = odom_msg.pose.pose.position
    quat = odom_msg.pose.pose.orientation
    stamp = str(odom_msg.header.stamp)
    stamp = stamp[:10] + "." + stamp[10:]
    with open(file_path, "a") as file:
        file.write(
            f"{stamp} {trans.x} {trans.y} {trans.z} {quat.x} {quat.y} {quat.z} {quat.w}\n"
        )


if __name__ == "__main__":

    rospy.init_node("record_odom", anonymous=True)
    file = open(file_path, "w")
    file.close()
    rospy.Subscriber("/okvis_node/keyframe_pose", Odometry, odom_callback)

    while not rospy.is_shutdown():
        rospy.spin()
