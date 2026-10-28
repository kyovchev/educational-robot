#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>

class JoyMoveNode : public rclcpp::Node {
public:
    JoyMoveNode() : Node("joy_moveit_node") {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyMoveNode::joy_callback, this, std::placeholders::_1), sub_opt);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&JoyMoveNode::init_robot, this), callback_group_);
    }

private:
    void init_robot() {
        timer_->cancel();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "arm");
        
        RCLCPP_INFO(this->get_logger(), "Robot Initialized. Press 'L1' to toggle Home/Start, hold 'R1' to move.");
        go_to_start_position(); // Start at your defined pose
    }

    void go_to_start_position() {
        is_moving_ = true;
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);

        tf2::Quaternion q;
        q.setRPY(3.14, 0.0, 0.0);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = 0.12;
        target_pose.pose.position.y = 0.00;
        target_pose.pose.position.z = 0.05;
        target_pose.pose.orientation = tf2::toMsg(q);

        move_group_->setPoseTarget(target_pose);
        move_group_->move(); 
        
        at_start_pose_ = true;
        is_moving_ = false;
        RCLCPP_INFO(this->get_logger(), "At Start Position.");
    }

    void go_to_home() {
        is_moving_ = true;
        // Slower movement for Home as requested
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);

        move_group_->setNamedTarget("Home");
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->execute(plan);
        }
        
        at_start_pose_ = false;
        is_moving_ = false;
        RCLCPP_INFO(this->get_logger(), "At Home Position.");
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (is_moving_) return;

        // --- TOGGLE LOGIC (Button L1 / Index 6) ---
        if (msg->buttons[6] == 1) {
            if (at_start_pose_) {
                go_to_home();
            } else {
                go_to_start_position();
            }
            return; // Exit callback after toggle
        }

        // --- MOVE LOGIC (Button R1 / Index 7) ---
        if (msg->buttons[7] == 1) {
            is_moving_ = true;
            move_group_->setMaxVelocityScalingFactor(1.0); // Reset speed for nudging
            
            auto current_pose = move_group_->getCurrentPose();
            double step = 0.01; 
            if (fabs(msg->axes[1]) > 0.3) current_pose.pose.position.x += msg->axes[1] * step;
            if (fabs(msg->axes[0]) > 0.3) current_pose.pose.position.y += msg->axes[0] * step;
            if (fabs(msg->axes[2]) > 0.3) current_pose.pose.position.z += msg->axes[2] * step;
            tf2::Quaternion q;
            q.setRPY(3.14, 0.0, 0.0);
            q.normalize();
            current_pose.pose.orientation = tf2::toMsg(q);

            RCLCPP_INFO(
                this->get_logger(),
                "Current position: x=%.3f, y=%.3f, z=%.3f",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z
            );

            if (fabs(msg->axes[1]) > 0.3 || fabs(msg->axes[0]) > 0.3 || fabs(msg->axes[2]) > 0.3) {
                move_group_->setPoseTarget(current_pose);
                move_group_->move(); 
            }

            is_moving_ = false;
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    bool is_moving_ = false;
    bool at_start_pose_ = false; // Flag to track current state
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyMoveNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}