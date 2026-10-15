// Read-only MuJoCo -> MPRC/RViz collision synchronization.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <dual_arm_reactive_control/msg/collision_object.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros/ros_two/plugin_utils.hpp"

namespace mujoco_ros {
class DynamicObstaclesPlugin : public MujocoPlugin {
 public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    auto node = get_node();
    declare_parameter_if_not_declared(node->get_node_parameters_interface(),
        "object_ids", rclcpp::ParameterValue(std::vector<std::string>{}));
    declare_parameter_if_not_declared(node->get_node_parameters_interface(),
        "geom_names", rclcpp::ParameterValue(std::vector<std::string>{}));
    declare_parameter_if_not_declared(node->get_node_parameters_interface(),
        "publish_rate", rclcpp::ParameterValue(100.0));
    return CallbackReturn::SUCCESS;
  }

  // Passive callback runs after world transforms and velocities are computed,
  // also during paused mj_forward. It never changes model or dynamic state.
  void PassiveCallback(const mjModel *model, mjData *data) override {
    if (!publisher_) return;
    const bool advanced = data->time != last_time_;
    const auto now = std::chrono::steady_clock::now();
    const bool heartbeat = now - last_wall_publish_ >= std::chrono::milliseconds(100);
    if (advanced && data->time > last_time_ && data->time < next_publish_) return;
    for (auto &entry : entries_) {
      const mjtNum *position = data->geom_xpos + 3 * entry.geom;
      mjtNum quaternion[4];
      mju_mat2Quat(quaternion, data->geom_xmat + 9 * entry.geom);
      mjtNum velocity[6];
      mj_objectVelocity(model, data, mjOBJ_GEOM, entry.geom, velocity, 0);
      bool changed = !entry.sent;
      for (int i = 0; i < 3; ++i) changed |= std::abs(position[i] - entry.last[i]) > 1e-10;
      for (int i = 0; i < 4; ++i) changed |= std::abs(quaternion[i] - entry.last[i+3]) > 1e-10;
      // Avoid flooding identical frames while paused, but reflect paused edits.
      if (!advanced && !changed && !heartbeat) continue;
      auto &msg = entry.message;
      msg.pose.position.x = position[0];
      msg.pose.position.y = position[1];
      msg.pose.position.z = position[2];
      msg.pose.orientation.w = quaternion[0];
      msg.pose.orientation.x = quaternion[1];
      msg.pose.orientation.y = quaternion[2];
      msg.pose.orientation.z = quaternion[3];
      msg.twist.angular.x = velocity[0];
      msg.twist.angular.y = velocity[1];
      msg.twist.angular.z = velocity[2];
      msg.twist.linear.x = velocity[3];
      msg.twist.linear.y = velocity[4];
      msg.twist.linear.z = velocity[5];
      publisher_->publish(msg);
      for (int i = 0; i < 3; ++i) entry.last[i] = position[i];
      for (int i = 0; i < 4; ++i) entry.last[i+3] = quaternion[i];
      entry.sent = true;
    }
    if (advanced || heartbeat) last_wall_publish_ = now;
    last_time_ = data->time;
    next_publish_ = data->time + period_;
  }

 protected:
  bool Load(const mjModel *model, mjData *) override {
    auto node = get_node();
    const auto ids = node->get_parameter("object_ids").as_string_array();
    const auto geoms = node->get_parameter("geom_names").as_string_array();
    const double rate = node->get_parameter("publish_rate").as_double();
    entries_.clear();
    if (ids.size() != geoms.size() || !std::isfinite(rate) || rate <= 0) {
      RCLCPP_ERROR(node->get_logger(), "Invalid dynamic collision sync configuration");
      return false;
    }
    period_ = 1.0 / rate;
    std::set<std::string> unique;
    for (size_t i = 0; i < ids.size(); ++i) {
      Entry entry;
      entry.geom = mj_name2id(model, mjOBJ_GEOM, geoms[i].c_str());
      if (entry.geom < 0 || ids[i].empty() || !unique.insert(ids[i]).second) {
        RCLCPP_ERROR(node->get_logger(), "Invalid/duplicate sync id %s or missing geom %s",
                     ids[i].c_str(), geoms[i].c_str());
        return false;
      }
      auto &msg = entry.message;
      msg.id = ids[i];
      msg.operation = msg.MOVE;
      auto &shape = msg.geometry;
      const mjtNum *size = model->geom_size + 3 * entry.geom;
      switch (model->geom_type[entry.geom]) {
        case mjGEOM_SPHERE:
          shape.type = shape.SPHERE; shape.dimensions = {size[0]}; break;
        case mjGEOM_BOX:
          shape.type = shape.BOX; shape.dimensions = {2*size[0], 2*size[1], 2*size[2]}; break;
        case mjGEOM_CYLINDER:
          shape.type = shape.CYLINDER; shape.dimensions = {2*size[1], size[0]}; break;
        default:
          RCLCPP_ERROR(node->get_logger(), "Unsupported collision geom %s", geoms[i].c_str());
          return false;
      }
      entries_.push_back(entry);
    }
    // Retain one message per geom for consumers that start while paused.
    publisher_ = node->create_publisher<Message>("/dynamic_obstacle",
        rclcpp::QoS(std::max<size_t>(1, entries_.size())).reliable().transient_local());
    // Base Publisher pointer intentionally publishes in the simulator callback,
    // independent of whether the plugin LifecyclePublisher has been activated.
    Reset();
    RCLCPP_INFO(node->get_logger(), "Dynamic collision sync: %zu geoms at %.1f Hz (read-only)",
                entries_.size(), rate);
    return true;
  }
  void Reset() override {
    last_time_ = -std::numeric_limits<double>::infinity();
    next_publish_ = last_time_;
    for (auto &entry : entries_) entry.sent = false;
  }
 private:
  using Message = dual_arm_reactive_control::msg::CollisionObject;
  struct Entry {
    int geom;
    Message message;
    mjtNum last[7]{};
    bool sent = false;
  };
  std::vector<Entry> entries_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point last_wall_publish_{};
  double period_ = .01;
  double last_time_ = -std::numeric_limits<double>::infinity();
  double next_publish_ = -std::numeric_limits<double>::infinity();
};
}  // namespace mujoco_ros
PLUGINLIB_EXPORT_CLASS(mujoco_ros::DynamicObstaclesPlugin, mujoco_ros::MujocoPlugin)
