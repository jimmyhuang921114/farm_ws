#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace sensor_bringup {

class LivoxCloudGuard : public rclcpp::Node {
public:
  LivoxCloudGuard() : Node("livox_cloud_guard") {
    const auto input_topic = declare_parameter("input_topic", "/livox/lidar");
    const auto output_topic = declare_parameter("output_topic", "/livox/lidar_valid");
    const auto imu_topic = declare_parameter("imu_topic", "/livox/imu_base");
    enabled_ = declare_parameter("enabled", true);
    minimum_valid_ratio_ = declare_parameter("minimum_valid_ratio", 0.5);
    baseline_limit_ = declare_parameter("baseline_window", 200);
    baseline_warmup_ = declare_parameter("baseline_warmup_frames", 20);
    minimum_fraction_ = declare_parameter("minimum_fraction_of_median", 0.1);
    absolute_minimum_ = declare_parameter("absolute_minimum_points", 1);
    warning_interval_ = declare_parameter("warning_interval_sec", 5.0);
    lidar_expected_period_ = declare_parameter("lidar_expected_period_sec", 0.1);
    imu_expected_period_ = declare_parameter("imu_expected_period_sec", 0.005);
    lidar_warning_gap_ = declare_parameter("lidar_warning_gap_sec", 0.3);
    imu_warning_gap_ = declare_parameter("imu_warning_gap_sec", 0.05);
    lidar_warning_gap_ = std::max(lidar_warning_gap_, 3.0 * lidar_expected_period_);
    imu_warning_gap_ = std::max(imu_warning_gap_, 3.0 * imu_expected_period_);
    recoverable_gap_ = declare_parameter("recoverable_gap_sec", 2.0);
    restart_gap_ = declare_parameter("restart_required_gap_sec", 30.0);
    max_lidar_imu_delta_ = declare_parameter("max_lidar_imu_delta_sec", 0.02);
    recovery_lidar_frames_ = declare_parameter("recovery_lidar_frames", 20);
    recovery_imu_duration_ = declare_parameter("recovery_imu_duration_sec", 1.0);

    auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/livox/continuity/diagnostics", 10);
    auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/livox/continuity/state", state_qos);
    restart_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/livox/continuity/restart_required", state_qos);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      std::bind(&LivoxCloudGuard::cloud_callback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos, std::bind(&LivoxCloudGuard::imu_callback, this, std::placeholders::_1));
    acknowledge_service_ = create_service<std_srvs::srv::Trigger>(
      "/livox/continuity/acknowledge_restart",
      std::bind(
        &LivoxCloudGuard::acknowledge_restart, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&LivoxCloudGuard::timer_callback, this));
    transition(State::RECOVERING, "startup_validation");
    RCLCPP_INFO(
      get_logger(), "Continuity guard: %s + %s -> %s", input_topic.c_str(),
      imu_topic.c_str(), output_topic.c_str());
  }

private:
  enum class State { HEALTHY, DEGRADED, DISCONNECTED, RECOVERING };

  struct FieldOffsets {
    int x{-1};
    int y{-1};
    int z{-1};
    uint8_t datatype{0};
  };

  static double stamp(const builtin_interfaces::msg::Time & value) {
    return value.sec + value.nanosec / 1e9;
  }

  static const char * state_name(State state) {
    switch (state) {
      case State::HEALTHY: return "HEALTHY";
      case State::DEGRADED: return "DEGRADED";
      case State::DISCONNECTED: return "DISCONNECTED";
      case State::RECOVERING: return "RECOVERING";
    }
    return "UNKNOWN";
  }

  static FieldOffsets xyz_offsets(const sensor_msgs::msg::PointCloud2 & message) {
    FieldOffsets result;
    for (const auto & field : message.fields) {
      if (field.name == "x") {
        result.x = static_cast<int>(field.offset);
        result.datatype = field.datatype;
      }
      if (field.name == "y") result.y = static_cast<int>(field.offset);
      if (field.name == "z") result.z = static_cast<int>(field.offset);
    }
    return result;
  }

  static double scalar(const uint8_t * data, uint8_t datatype) {
    if (datatype == sensor_msgs::msg::PointField::FLOAT32) {
      float value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
      double value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  int adaptive_minimum() const {
    if (baseline_.size() < static_cast<size_t>(baseline_warmup_)) return absolute_minimum_;
    auto sorted = std::vector<size_t>(baseline_.begin(), baseline_.end());
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
    return std::max(
      absolute_minimum_, static_cast<int>(
        std::ceil(sorted[sorted.size() / 2] * minimum_fraction_)));
  }

  double nearest_imu_delta(double lidar_stamp) const {
    if (imu_stamps_.empty()) return std::numeric_limits<double>::infinity();
    auto closest = imu_stamps_.front();
    for (const double value : imu_stamps_) {
      if (std::abs(value - lidar_stamp) < std::abs(closest - lidar_stamp)) closest = value;
    }
    return lidar_stamp - closest;
  }

  void mark_discontinuity(const std::string & reason, bool restart) {
    ++discontinuity_count_;
    reason_ = reason;
    if (restart) restart_required_ = true;
    recovery_lidar_count_ = 0;
    recovery_imu_start_wall_ = 0.0;
  }

  void transition(State next, const std::string & reason) {
    if (state_ != next) {
      RCLCPP_WARN(get_logger(), "Continuity %s -> %s: %s", state_name(state_), state_name(next), reason.c_str());
    }
    state_ = next;
    reason_ = reason;
    publish_status(true);
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr message) {
    const double sensor_stamp = stamp(message->header.stamp);
    const double wall = steady_seconds();
    imu_sensor_gap_ = last_imu_stamp_ > 0.0 ? sensor_stamp - last_imu_stamp_ : 0.0;
    imu_wall_gap_ = last_imu_wall_ > 0.0 ? wall - last_imu_wall_ : 0.0;
    const bool non_increasing = last_imu_stamp_ > 0.0 && imu_sensor_gap_ <= 0.0;
    const bool sensor_jump = last_imu_stamp_ > 0.0 && imu_sensor_gap_ > restart_gap_ && imu_wall_gap_ < recoverable_gap_;
    if (non_increasing || sensor_jump) {
      mark_discontinuity(non_increasing ? "imu_stamp_not_increasing" : "imu_sensor_time_jump", true);
      transition(State::RECOVERING, reason_);
    } else if (imu_sensor_gap_ > imu_warning_gap_ || imu_wall_gap_ > imu_warning_gap_) {
      mark_discontinuity("imu_gap", imu_sensor_gap_ > restart_gap_ || imu_wall_gap_ > restart_gap_);
      transition(imu_wall_gap_ > recoverable_gap_ ? State::RECOVERING : State::DEGRADED, reason_);
    }
    last_imu_stamp_ = sensor_stamp;
    last_imu_wall_ = wall;
    imu_stamps_.push_back(sensor_stamp);
    while (imu_stamps_.size() > 1000) imu_stamps_.pop_front();
    if (state_ == State::RECOVERING && recovery_imu_start_wall_ == 0.0) recovery_imu_start_wall_ = wall;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    ++input_frames_;
    const double sensor_stamp = stamp(message->header.stamp);
    const double wall = steady_seconds();
    lidar_sensor_gap_ = last_lidar_stamp_ > 0.0 ? sensor_stamp - last_lidar_stamp_ : 0.0;
    lidar_wall_gap_ = last_lidar_wall_ > 0.0 ? wall - last_lidar_wall_ : 0.0;
    lidar_imu_delta_ = nearest_imu_delta(sensor_stamp);
    const bool non_increasing = last_lidar_stamp_ > 0.0 && lidar_sensor_gap_ <= 0.0;
    const bool sensor_jump = last_lidar_stamp_ > 0.0 && lidar_sensor_gap_ > restart_gap_ && lidar_wall_gap_ < recoverable_gap_;
    if (state_ == State::DISCONNECTED) {
      const bool imu_recent = last_imu_wall_ > 0.0 &&
        wall - last_imu_wall_ <= imu_warning_gap_;
      if (imu_recent) {
        transition(State::RECOVERING, "lidar_and_imu_streams_resumed");
        recovery_lidar_count_ = 0;
      }
    }
    if (non_increasing || sensor_jump) {
      mark_discontinuity(non_increasing ? "lidar_stamp_not_increasing" : "lidar_sensor_time_jump", true);
      transition(State::RECOVERING, reason_);
    } else if (
      state_ != State::DISCONNECTED &&
      (lidar_sensor_gap_ > lidar_warning_gap_ || lidar_wall_gap_ > lidar_warning_gap_))
    {
      const bool restart = lidar_sensor_gap_ > restart_gap_ || lidar_wall_gap_ > restart_gap_;
      mark_discontinuity("lidar_gap", restart);
      transition(lidar_wall_gap_ > recoverable_gap_ ? State::RECOVERING : State::DEGRADED, reason_);
    }
    last_lidar_stamp_ = sensor_stamp;
    last_lidar_wall_ = wall;

    const size_t declared = static_cast<size_t>(message->width) * message->height;
    const size_t available = message->point_step ? message->data.size() / message->point_step : 0;
    const size_t count = std::min(declared, available);
    const auto offsets = xyz_offsets(*message);
    std::vector<uint8_t> keep(count, 0);
    size_t finite_count = 0;
    size_t quality_count = 0;
    std::vector<std::string> quality_reasons;
    if (message->is_bigendian || offsets.x < 0 || offsets.y < 0 || offsets.z < 0 ||
      (offsets.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
      offsets.datatype != sensor_msgs::msg::PointField::FLOAT64))
    {
      quality_reasons.emplace_back("unsupported_xyz_layout");
    } else {
      for (size_t index = 0; index < count; ++index) {
        const auto * point = message->data.data() + index * message->point_step;
        const double x = scalar(point + offsets.x, offsets.datatype);
        const double y = scalar(point + offsets.y, offsets.datatype);
        const double z = scalar(point + offsets.z, offsets.datatype);
        const bool finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        keep[index] = finite;
        finite_count += finite;
        quality_count += finite && !(x == 0.0 && y == 0.0 && z == 0.0);
      }
    }
    const int threshold = adaptive_minimum();
    const double valid_ratio = count ? static_cast<double>(quality_count) / count : 0.0;
    if (!count) quality_reasons.emplace_back("empty_cloud");
    if (quality_count < static_cast<size_t>(threshold)) quality_reasons.emplace_back("below_adaptive_minimum");
    if (valid_ratio < minimum_valid_ratio_) quality_reasons.emplace_back("low_valid_ratio");
    if (!quality_reasons.empty()) {
      ++dropped_frames_;
      ++consecutive_drops_;
      recovery_lidar_count_ = 0;
      reason_ = quality_reasons.front();
      publish_status();
      return;
    }

    if (state_ == State::DEGRADED && lidar_sensor_gap_ > 0.0 && lidar_sensor_gap_ < lidar_warning_gap_) {
      transition(State::HEALTHY, "degraded_input_recovered");
    }
    if (state_ == State::RECOVERING) {
      const bool lidar_period_ok = lidar_sensor_gap_ > 0.0 && lidar_sensor_gap_ <= lidar_warning_gap_;
      const bool imu_duration_ok = recovery_imu_start_wall_ > 0.0 &&
        wall - recovery_imu_start_wall_ >= recovery_imu_duration_;
      const bool sync_ok = std::abs(lidar_imu_delta_) <= max_lidar_imu_delta_;
      recovery_lidar_count_ = lidar_period_ok && sync_ok ? recovery_lidar_count_ + 1 : 0;
      if (recovery_lidar_count_ < static_cast<size_t>(recovery_lidar_frames_) || !imu_duration_ok) {
        ++gated_frames_;
        publish_status();
        return;
      }
      transition(State::HEALTHY, "recovery_validation_complete");
    }
    if (state_ == State::DISCONNECTED) {
      ++gated_frames_;
      publish_status();
      return;
    }

    if (!enabled_ || finite_count == count) {
      publisher_->publish(*message);
    } else {
      sensor_msgs::msg::PointCloud2 output = *message;
      output.height = 1;
      output.width = static_cast<uint32_t>(finite_count);
      output.row_step = output.width * output.point_step;
      output.data.resize(output.row_step);
      size_t destination = 0;
      for (size_t index = 0; index < count; ++index) {
        if (!keep[index]) continue;
        std::memcpy(output.data.data() + destination,
          message->data.data() + index * message->point_step, message->point_step);
        destination += message->point_step;
      }
      output.is_dense = true;
      removed_points_ += count - finite_count;
      publisher_->publish(std::move(output));
    }
    ++output_frames_;
    consecutive_drops_ = 0;
    last_good_lidar_stamp_ = sensor_stamp;
    last_good_imu_stamp_ = imu_stamps_.empty() ? 0.0 : imu_stamps_.back();
    baseline_.push_back(quality_count);
    while (baseline_.size() > static_cast<size_t>(baseline_limit_)) baseline_.pop_front();
    publish_status();
  }

  void timer_callback() {
    const double wall = steady_seconds();
    const double lidar_silence = last_lidar_wall_ > 0.0 ? wall - last_lidar_wall_ : wall - start_wall_;
    const double imu_silence = last_imu_wall_ > 0.0 ? wall - last_imu_wall_ : wall - start_wall_;
    if (lidar_silence > recoverable_gap_ || imu_silence > recoverable_gap_) {
      if (state_ != State::DISCONNECTED) {
        mark_discontinuity(lidar_silence > imu_silence ? "lidar_receive_timeout" : "imu_receive_timeout", false);
        transition(State::DISCONNECTED, reason_);
      }
      if (lidar_silence > restart_gap_ || imu_silence > restart_gap_) restart_required_ = true;
    }
    publish_status();
  }

  void acknowledge_restart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ != State::HEALTHY) {
      response->success = false;
      response->message = "continuity state is not HEALTHY";
      return;
    }
    restart_required_ = false;
    response->success = true;
    response->message = "restart requirement acknowledged";
    publish_status();
  }

  double steady_seconds() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_status(bool force = false) {
    const double status_wall = steady_seconds();
    if (!force && status_wall - last_status_wall_ < 0.5) return;
    last_status_wall_ = status_wall;
    std_msgs::msg::String state_message;
    state_message.data = state_name(state_);
    state_publisher_->publish(state_message);
    std_msgs::msg::Bool restart_message;
    restart_message.data = restart_required_;
    restart_publisher_->publish(restart_message);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "livox_sensor_continuity";
    status.hardware_id = "livox_mid360";
    status.level = state_ == State::HEALTHY ? status.OK :
      (state_ == State::DEGRADED || state_ == State::RECOVERING ? status.WARN : status.ERROR);
    status.message = state_name(state_);
    const std::vector<std::pair<std::string, std::string>> values = {
      {"state", state_name(state_)}, {"reason", reason_},
      {"restart_required", restart_required_ ? "true" : "false"},
      {"discontinuity_count", std::to_string(discontinuity_count_)},
      {"last_good_lidar_stamp", std::to_string(last_good_lidar_stamp_)},
      {"last_good_imu_stamp", std::to_string(last_good_imu_stamp_)},
      {"lidar_sensor_gap_sec", std::to_string(lidar_sensor_gap_)},
      {"imu_sensor_gap_sec", std::to_string(imu_sensor_gap_)},
      {"lidar_wall_gap_sec", std::to_string(lidar_wall_gap_)},
      {"imu_wall_gap_sec", std::to_string(imu_wall_gap_)},
      {"lidar_nearest_imu_delta_sec", std::to_string(lidar_imu_delta_)},
      {"recovery_lidar_frames", std::to_string(recovery_lidar_count_)},
      {"recovery_lidar_target", std::to_string(recovery_lidar_frames_)},
      {"input_frames", std::to_string(input_frames_)},
      {"output_frames", std::to_string(output_frames_)},
      {"gated_frames", std::to_string(gated_frames_)},
      {"dropped_frames", std::to_string(dropped_frames_)},
      {"removed_points", std::to_string(removed_points_)}};
    for (const auto & value : values) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = value.first;
      item.value = value.second;
      status.values.push_back(std::move(item));
    }
    array.status.push_back(std::move(status));
    diagnostics_->publish(std::move(array));
  }

  bool enabled_{true};
  bool restart_required_{false};
  State state_{State::DISCONNECTED};
  std::string reason_{"not_initialized"};
  double minimum_valid_ratio_{0.5};
  int baseline_limit_{200};
  int baseline_warmup_{20};
  double minimum_fraction_{0.1};
  int absolute_minimum_{1};
  double warning_interval_{5.0};
  double lidar_expected_period_{0.1};
  double imu_expected_period_{0.005};
  double lidar_warning_gap_{0.3};
  double imu_warning_gap_{0.05};
  double recoverable_gap_{2.0};
  double restart_gap_{30.0};
  double max_lidar_imu_delta_{0.02};
  int recovery_lidar_frames_{20};
  double recovery_imu_duration_{1.0};
  double start_wall_{steady_seconds()};
  double last_lidar_stamp_{0.0};
  double last_imu_stamp_{0.0};
  double last_lidar_wall_{0.0};
  double last_imu_wall_{0.0};
  double last_good_lidar_stamp_{0.0};
  double last_good_imu_stamp_{0.0};
  double recovery_imu_start_wall_{0.0};
  double lidar_sensor_gap_{0.0};
  double imu_sensor_gap_{0.0};
  double lidar_wall_gap_{0.0};
  double imu_wall_gap_{0.0};
  double lidar_imu_delta_{std::numeric_limits<double>::infinity()};
  double last_status_wall_{0.0};
  size_t recovery_lidar_count_{0};
  size_t discontinuity_count_{0};
  size_t input_frames_{0};
  size_t output_frames_{0};
  size_t gated_frames_{0};
  size_t dropped_frames_{0};
  size_t removed_points_{0};
  size_t consecutive_drops_{0};
  std::deque<size_t> baseline_;
  std::deque<double> imu_stamps_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr restart_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr acknowledge_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensor_bringup

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_bringup::LivoxCloudGuard>());
  rclcpp::shutdown();
  return 0;
}
