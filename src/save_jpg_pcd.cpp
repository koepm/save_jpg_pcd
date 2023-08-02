#include "save_jpg_pcd/save_jpg_pcd.hpp"


//=======================!!!!!!경로만 수정해 주세요 !!!!===============================

std::string jpg_dir_path = "/root/ws/src/bag_file/calibration_jpg/";
std::string pcd_dir_path = "/root/ws/src/bag_file/calibration_pcd/";
std::string image_sub_name = "video1";
std::string pointcloud_sub_name = "velodyne_points";

//=================================================================================
auto cap_moment = 0;

FusionSync::FusionSync()
: Node("save_jpg_pcd")
{
  img_sub_ = std::make_shared<StampedImageMsgSubscriber>(this, image_sub_name, rmw_qos_profile_sensor_data);
  pcl_sub_ = std::make_shared<StampedPclMsgSubscriber>(this, pointcloud_sub_name, rmw_qos_profile_sensor_data);
  key_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"turtle1/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {keyCallback(msg);});

  approximate_sync_ = std::make_shared<ApproximateSync>(ApproximateSyncPolicy(10), *img_sub_, *pcl_sub_);
  approximate_sync_->registerCallback(std::bind(&FusionSync::approximateSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
}


void FusionSync::keyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cap_moment = int(msg->linear.x);
}

// 이미지, 포인트클라우드, 좌표 처리하는 함수
void FusionSync::approximateSyncCallback(
  const std::shared_ptr<const sensor_msgs::msg::Image>& msg1,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg2
  )
{
    try
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      // ROS2 메세지를 PCL 포인트 클라우드로 변환
      pcl::fromROSMsg(*msg2, *point_cloud);
      cv_bridge::CvImagePtr cv_ptr_;
      cv_ptr_ = cv_bridge::toCvCopy(msg1, "bgr8");
      cv::Mat raw_image = cv_ptr_->image;
      
      std::string image_file_name = jpg_dir_path + std::to_string(msg1->header.stamp.sec) + ".jpg";
      std::string pointcloud_file_name = pcd_dir_path + std::to_string(msg2->header.stamp.sec) + ".pcd";
      

      if (cap_moment == 2)
      {
        // 이미지를 jpg 파일로 저장
        cv::imwrite(image_file_name, raw_image);
        RCLCPP_INFO(this->get_logger(), "Saved image as: %s", image_file_name.c_str());

        // PointCloud를 PCD 파일로 저장
        pcl::io::savePCDFileASCII(pointcloud_file_name, *point_cloud);
        RCLCPP_INFO(this->get_logger(), "Saved PointCloud as: %s", pointcloud_file_name.c_str());
        cap_moment = -1;
      }
      

    }
    catch(cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "exception: %s", e.what());
      return;
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionSync>());
  rclcpp::shutdown();
  return 0;
}