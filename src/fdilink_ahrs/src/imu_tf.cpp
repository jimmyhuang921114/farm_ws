#include <inttypes.h>

#include <memory>
#include <string>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using namespace std;

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29

int position_x;
int position_y;
int position_z;
std::string imu_topic;
std::string imu_frame_id, world_frame_id;

static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
rclcpp::Node::SharedPtr nh_{nullptr};

class imu_data_to_tf : public rclcpp::Node {
public:
  imu_data_to_tf() : Node("imu_data_to_tf") {
    this->declare_parameter<std::string>("world_frame_id", "/world");
    this->get_parameter("world_frame_id", world_frame_id);

    this->declare_parameter<std::string>("imu_frame_id", "/imu");
    this->get_parameter("imu_frame_id", imu_frame_id);

    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->get_parameter("imu_topic", imu_topic);

    this->declare_parameter<std::int16_t>("position_x", 1);
    this->get_parameter("position_x", position_x);

    this->declare_parameter<std::int16_t>("position_y", 1);
    this->get_parameter("position_y", position_y);

    this->declare_parameter<std::int16_t>("position_z", 1);
    this->get_parameter("position_z", position_z);

    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic.c_str(), 10, std::bind(&imu_data_to_tf::ImuCallback, this, _1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion q;
    q.setX(imu_data->orientation.x);
    q.setY(imu_data->orientation.y);
    q.setZ(imu_data->orientation.z);
    q.setW(imu_data->orientation.w);
    q.normalized();

    geometry_msgs::msg::TransformStamped tfs;
    tfs.header.stamp = rclcpp::Node::now();
    tfs.header.frame_id = "world";
    tfs.child_frame_id = "imu";
    tfs.transform.translation.x = position_x;
    tfs.transform.translation.y = position_y;
    tfs.transform.translation.z = position_z;
    tfs.transform.rotation.x = q.getX();
    tfs.transform.rotation.y = q.getY();
    tfs.transform.rotation.z = q.getZ();
    tfs.transform.rotation.w = q.getW();
    br->sendTransform(tfs);
  }
};

int
main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_data_to_tf>());
  rclcpp::shutdown();

  return 0;
}
