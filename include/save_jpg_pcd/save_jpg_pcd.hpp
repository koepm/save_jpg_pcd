#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

using StampedImageMsg = sensor_msgs::msg::Image;
using StampedImageMsgSubscriber = message_filters::Subscriber<StampedImageMsg>;

using StampedPclMsg = sensor_msgs::msg::PointCloud2;
using StampedPclMsgSubscriber = message_filters::Subscriber<StampedPclMsg>;

using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<StampedImageMsg, StampedPclMsg>;
using ApproximateSync = message_filters::Synchronizer<ApproximateSyncPolicy>;


class FusionSync : public rclcpp::Node
{
public:
  FusionSync();

private:
  std::shared_ptr<StampedImageMsgSubscriber> img_sub_;
  std::shared_ptr<StampedPclMsgSubscriber> pcl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_;

  std::shared_ptr<ApproximateSync> approximate_sync_;
  
  void keyCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void approximateSyncCallback(
    const std::shared_ptr<const sensor_msgs::msg::Image>& msg1,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg2
    );
};