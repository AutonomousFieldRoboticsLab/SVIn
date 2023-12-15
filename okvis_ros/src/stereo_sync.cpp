#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/console.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_img;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_pol_compressed_img;

std::string left_img_topic, right_img_topic;
std::string left_compressed_img_topic, right_compressed_img_topic;
bool compressed_image;
ros::Time previous_stamp(1);

ros::Publisher left_img_pub, right_img_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg) {
  ROS_INFO_STREAM_ONCE("Inside stereo image callback");

  ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;
  std_msgs::Header left_header, right_header;
  left_header.stamp = stamp;
  right_header.stamp = stamp;
  left_header.frame_id = left_msg->header.frame_id;
  right_header.frame_id = right_msg->header.frame_id;

  if (previous_stamp >= stamp) {
    return;
  }

  left_img_pub.publish(left_msg);
  right_img_pub.publish(right_msg);

  previous_stamp = stamp;
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& left_msg,
                             const sensor_msgs::CompressedImageConstPtr& right_msg) {
  ROS_INFO_STREAM_ONCE("Inside stereo compressed image callback");

  ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;
  std_msgs::Header left_header, right_header;
  left_header.stamp = stamp;
  right_header.stamp = stamp;
  left_header.frame_id = left_msg->header.frame_id;
  right_header.frame_id = right_msg->header.frame_id;

  if (previous_stamp >= stamp) {
    return;
  }

  cv_bridge::CvImageConstPtr left_cv_ptr, right_cv_ptr;
  left_cv_ptr = cv_bridge::toCvCopy(left_msg);
  right_cv_ptr = cv_bridge::toCvCopy(right_msg);

  left_img_pub.publish(left_cv_ptr->toImageMsg());
  right_img_pub.publish(right_cv_ptr->toImageMsg());

  previous_stamp = stamp;
}

void readParameters(const ros::NodeHandle& nh) {
  if (!nh.hasParam("left_img_topic")) {
    ROS_FATAL_STREAM("left_img_topic param not found");
    exit(1);
  } else {
    nh.getParam("left_img_topic", left_img_topic);
  }

  if (!nh.hasParam("right_img_topic")) {
    ROS_FATAL_STREAM("right_img_topic not found");
    exit(1);
  } else {
    nh.getParam("left_img_topic", right_img_topic);
  }

  if (nh.hasParam("compressed")) nh.getParam("compressed", compressed_image);

  if (compressed_image) {
    left_compressed_img_topic = left_img_topic + "/compressed";
    right_compressed_img_topic = right_img_topic + "/compressed";
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_sync");
  ros::NodeHandle nh = ros::NodeHandle("~");
  std::vector<std::string> keys;
  nh.getParamNames(keys);

  for (auto k : keys) {
    ROS_INFO_STREAM(k);
  }
  readParameters(nh);

  ROS_INFO_STREAM("Left Image Topic: " << left_img_topic);
  ROS_INFO_STREAM("Right Image Topic: " << right_img_topic);

  message_filters::Subscriber<sensor_msgs::Image> l_img_sub(nh, left_img_topic, 100);
  message_filters::Subscriber<sensor_msgs::Image> r_img_sub(nh, right_img_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> l_img_compressed_sub(nh, left_compressed_img_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> r_img_compressed_sub(nh, right_compressed_img_topic, 100);

  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::Synchronizer<sync_pol_img> image_sync(sync_pol_img(25), l_img_sub, r_img_sub);
  message_filters::Synchronizer<sync_pol_compressed_img> compressed_image_sync(
      sync_pol_compressed_img(25), l_img_compressed_sub, r_img_compressed_sub);

  left_img_pub = nh.advertise<sensor_msgs::Image>("/cam0/image_raw", 100);
  right_img_pub = nh.advertise<sensor_msgs::Image>("/cam1/image_raw", 100);

  if (compressed_image) {
    compressed_image_sync.registerCallback(boost::bind(&compressedImageCallback, _1, _2));
  } else {
    image_sync.registerCallback(boost::bind(&imageCallback, _1, _2));
  }

  ros::spin();

  return 0;
}