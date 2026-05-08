#ifndef MUJOCO_ROS2_DYNAMIC_OBSTACLES_PLUGIN__DYNAMIC_OBSTACLES_PLUGIN_HPP_
#define MUJOCO_ROS2_DYNAMIC_OBSTACLES_PLUGIN__DYNAMIC_OBSTACLES_PLUGIN_HPP_

#include <thread>
#include <memory>
#include <vector>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros/ros_two/plugin_utils.hpp"
#include "mujoco_ros/common_types.hpp"

namespace mujoco_ros {

class DynamicObstaclesPlugin : public mujoco_ros::MujocoPlugin
{
public:
    mujoco_ros::CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/) override;
    ~DynamicObstaclesPlugin() override;
    void ControlCallback(const mjModel* model, mjData* data) override;

protected:
    bool Load(const mjModel *m, mjData *d) override;
    void Reset() override;

private:
    rclcpp::Logger get_my_logger() { return rclcpp::get_logger("DynamicObstaclesPlugin"); };
    
    struct Obstacle {
        int id;
        std::string name;
        mjtNum velocity[3];
    };

    std::vector<Obstacle> obstacles_;
    const mjModel* m_;
    mjData* d_;
    
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;
};

} // namespace mujoco_ros

#endif // MUJOCO_ROS2_DYNAMIC_OBSTACLES_PLUGIN__DYNAMIC_OBSTACLES_PLUGIN_HPP_
