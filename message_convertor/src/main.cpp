#!/usr/bin/env python3
"""
Convert nav_msgs/Odometry to maplab_msgs/OdometryWithImuBiases in a bag.

- Copies header, child_frame_id, pose, twist.
- Sets odometry_state = 1.
- gyro_bias = pose.covariance[0:3]
- accel_bias = pose.covariance[3:6]
"""

import argparse
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from maplab_msgs.msg import OdometryWithImuBiases

def main():
    parser = argparse.ArgumentParser(description="Convert Odometry -> OdometryWithImuBiases")
    parser.add_argument("--in-bag", required=True, help="Input rosbag path")
    parser.add_argument("--out-bag", required=True, help="Output rosbag path")
    parser.add_argument("--in-topic", default="/odom", help="Input Odometry topic")
    parser.add_argument("--out-topic", default="/odometry_with_biases", help="Output Maplab topic")
    args = parser.parse_args()

    with rosbag.Bag(args.in_bag, "r") as ib, rosbag.Bag(args.out_bag, "w") as ob:
        count = 0
        for topic, msg, t in ib.read_messages(topics=[args.in_topic]):
            if not isinstance(msg, Odometry):
                continue

            out = OdometryWithImuBiases()
            out.header = msg.header
            out.child_frame_id = msg.child_frame_id
            out.pose = msg.pose
            out.twist = msg.twist

            # Biases from pose.covariance
            out.gyro_bias = Vector3(
                x=msg.pose.covariance[0],
                y=msg.pose.covariance[1],
                z=msg.pose.covariance[2]
            )
            out.accel_bias = Vector3(
                x=msg.pose.covariance[3],
                y=msg.pose.covariance[4],
                z=msg.pose.covariance[5]
            )

            out.odometry_state = 0  # fixed

            ob.write(args.out_topic, out, t=msg.header.stamp if msg.header.stamp else t)
            count += 1

        print(f"[DONE] Converted {count} messages.")

if __name__ == "__main__":
    main()
