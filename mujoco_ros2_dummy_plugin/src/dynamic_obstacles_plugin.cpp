#include "mujoco_ros2_dummy_plugin/dynamic_obstacles_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros {

DynamicObstaclesPlugin::~DynamicObstaclesPlugin() {}

mujoco_ros::CallbackReturn DynamicObstaclesPlugin::on_configure(const rclcpp_lifecycle::State &/*previous_state*/) {
    RCLCPP_INFO(get_my_logger(), "Configuring DynamicObstaclesPlugin");
    obstacle_pub_ = get_node()->create_publisher<dual_arm_reactive_control::msg::CollisionObject>("/dynamic_obstacle", 10);
    return mujoco_ros::CallbackReturn::SUCCESS;
}

bool DynamicObstaclesPlugin::Load(const mjModel *m, mjData *d) {
    m_ = m;
    d_ = d;
    
    std::random_device rd;
    gen_ = std::mt19937(rd());
    dis_ = std::uniform_real_distribution<>(-0.1, 0.1);

    obstacles_.clear();
    for (int i = 0; i < m->nbody; i++) {
        const char* name = mj_id2name(m, mjOBJ_BODY, i);
        if (name && std::string(name).find("dyn_sphere_") == 0) {
            Obstacle obs;
            obs.id = i;
            obs.name = name;
            obs.velocity[0] = dis_(gen_);
            obs.velocity[1] = dis_(gen_);
            obs.velocity[2] = dis_(gen_);
            obstacles_.push_back(obs);
            RCLCPP_INFO(get_my_logger(), "Added dynamic obstacle: %s", name);
        }
    }

    return true;
}

void DynamicObstaclesPlugin::ControlCallback(const mjModel* model, mjData* data) {
    for (auto& obs : obstacles_) {
        // Find the specific joint for this body to get correct qpos and qvel addresses
        int joint_id = -1;
        for (int j = 0; j < model->njnt; j++) {
            if (model->jnt_bodyid[j] == obs.id) {
                joint_id = j;
                break;
            }
        }

        if (joint_id == -1) continue;

        int qpos_adr = model->jnt_qposadr[joint_id];
        int qvel_adr = model->jnt_dofadr[joint_id];
        
        // Free joint has 7 pos (3 pos, 4 quat)
        mjtNum* pos = &data->qpos[qpos_adr];
        
        mjtNum dist = mju_norm3(pos);
        if (dist > 0.8) {
            // Check if moving away from origin (dot product of position and velocity > 0)
            mjtNum dot = pos[0]*obs.velocity[0] + pos[1]*obs.velocity[1] + pos[2]*obs.velocity[2];
            if (dot > 0) {
                // Reflect velocity vector
                mjtNum normal[3] = {pos[0]/dist, pos[1]/dist, pos[2]/dist};
                mjtNum v_dot_n = (obs.velocity[0]*normal[0] + obs.velocity[1]*normal[1] + obs.velocity[2]*normal[2]);
                
                obs.velocity[0] -= 2.0 * v_dot_n * normal[0];
                obs.velocity[1] -= 2.0 * v_dot_n * normal[1];
                obs.velocity[2] -= 2.0 * v_dot_n * normal[2];
                
                RCLCPP_INFO(get_my_logger(), "Obstacle %s reflected. Dist: %.2f Pos: [%.2f %.2f %.2f]", 
                            obs.name.c_str(), dist, pos[0], pos[1], pos[2]);
            }
        }

        // Add ground bounce logic (assuming floor is at z=0)
        // If z coordinate is less than radius (0.10) and moving downwards
        if (pos[2] <= 0.10 && obs.velocity[2] < 0) {
            obs.velocity[2] = -obs.velocity[2];
            RCLCPP_INFO(get_my_logger(), "Obstacle %s bounced. Z: %.2f", obs.name.c_str(), pos[2]);
        }

        // Apply velocities directly to qvel (3 lin, 3 ang)
        if (qvel_adr != -1) {
            data->qvel[qvel_adr]   = obs.velocity[0];
            data->qvel[qvel_adr+1] = obs.velocity[1];
            data->qvel[qvel_adr+2] = obs.velocity[2];
            data->qvel[qvel_adr+3] = 0;
            data->qvel[qvel_adr+4] = 0;
            data->qvel[qvel_adr+5] = 0;
        }
    }
}

void DynamicObstaclesPlugin::PassiveCallback(const mjModel* model, mjData* data) {
    if (!obstacle_pub_) return;

    for (const auto& obs : obstacles_) {
        int joint_id = -1;
        for (int j = 0; j < model->njnt; j++) {
            if (model->jnt_bodyid[j] == obs.id) {
                joint_id = j;
                break;
            }
        }

        if (joint_id == -1) continue;

        int qpos_adr = model->jnt_qposadr[joint_id];
        mjtNum* pos = &data->qpos[qpos_adr];
        mjtNum* quat = &data->qpos[qpos_adr + 3];

        auto msg = dual_arm_reactive_control::msg::CollisionObject();
        msg.id = obs.name;
        msg.operation = dual_arm_reactive_control::msg::CollisionObject::MOVE;
        
        msg.pose.position.x = pos[0];
        msg.pose.position.y = pos[1];
        msg.pose.position.z = pos[2];
        msg.pose.orientation.w = quat[0];
        msg.pose.orientation.x = quat[1];
        msg.pose.orientation.y = quat[2];
        msg.pose.orientation.z = quat[3];

        obstacle_pub_->publish(msg);
    }
}

void DynamicObstaclesPlugin::Reset() {
    for (auto& obs : obstacles_) {
        obs.velocity[0] = dis_(gen_);
        obs.velocity[1] = dis_(gen_);
        obs.velocity[2] = dis_(gen_);
    }
}

} // namespace mujoco_ros

PLUGINLIB_EXPORT_CLASS(mujoco_ros::DynamicObstaclesPlugin, mujoco_ros::MujocoPlugin)
