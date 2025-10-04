#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotic_arm_msgs/action/robotic_arm_task.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace robotic_arm_remote
{
class TaskServo : public rclcpp::Node
{
public:
    explicit TaskServo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("TaskServo", options)
    {
        arm_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "arm_links_controller/joint_trajectory", 10);
        gripper_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "gripper_controller/joint_trajectory", 10);

        arm_motion_duration_ = this->declare_parameter<double>("arm_motion_duration", 4.0);
        gripper_motion_duration_ = this->declare_parameter<double>("gripper_motion_duration", 2.0);

        action_server_ = rclcpp_action::create_server<robotic_arm_msgs::action::RoboticArmTask>(
            this,
            "task_server",
            std::bind(&TaskServo::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TaskServo::handle_cancel, this, std::placeholders::_1),
            std::bind(&TaskServo::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Task servo action server is ready");
    }

private:
    using RoboticArmTask = robotic_arm_msgs::action::RoboticArmTask;
    using GoalHandle = rclcpp_action::ServerGoalHandle<RoboticArmTask>;

    struct JointTargets
    {
        double base_deg;
        double shoulder_deg;
        double elbow_deg;
        double gripper_deg;
    };

    static double baseDegreesToCommand(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    static double shoulderDegreesToCommand(double degrees)
    {
        return (180.0 - degrees) / 90.0;
    }

    static double elbowDegreesToCommand(double degrees)
    {
        return degrees / 90.0;
    }

    static double gripperDegreesToCommand(double degrees)
    {
        return -degrees * M_PI / 360.0;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const RoboticArmTask::Goal> goal)
    {
        (void)uuid;
        if(targets_.find(goal->task_number) == targets_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Received unknown task number: %d", goal->task_number);
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Accepted task request %d", goal->task_number);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&TaskServo::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<RoboticArmTask::Result>();

        auto target_it = targets_.find(goal->task_number);
        if(target_it == targets_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown task number: %d", goal->task_number);
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        const JointTargets & target = target_it->second;

        trajectory_msgs::msg::JointTrajectory arm_trajectory;
        arm_trajectory.header.stamp = this->now();
        arm_trajectory.joint_names = {"bicep_joint", "arm1_joint", "arm2_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint arm_point;
        arm_point.positions = {
            baseDegreesToCommand(target.base_deg),
            shoulderDegreesToCommand(target.shoulder_deg),
            elbowDegreesToCommand(target.elbow_deg)
        };
        arm_point.time_from_start = rclcpp::Duration::from_seconds(arm_motion_duration_);
        arm_trajectory.points.push_back(arm_point);

        trajectory_msgs::msg::JointTrajectory gripper_trajectory;
        gripper_trajectory.header.stamp = arm_trajectory.header.stamp;
        gripper_trajectory.joint_names = {"leftClaw_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
        gripper_point.positions = {gripperDegreesToCommand(target.gripper_deg)};
        gripper_point.time_from_start = rclcpp::Duration::from_seconds(gripper_motion_duration_);
        gripper_trajectory.points.push_back(gripper_point);

        RCLCPP_INFO(this->get_logger(),
                    "Publishing arm targets [deg]: base=%.1f, shoulder=%.1f, elbow=%.1f | gripper=%.1f",
                    target.base_deg, target.shoulder_deg, target.elbow_deg, target.gripper_deg);

        arm_publisher_->publish(arm_trajectory);
        gripper_publisher_->publish(gripper_trajectory);

        const double wait_seconds = std::max(arm_motion_duration_, gripper_motion_duration_);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(wait_seconds)));

        result->success = true;
        goal_handle->succeed(result);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher_;
    rclcpp_action::Server<RoboticArmTask>::SharedPtr action_server_;

    double arm_motion_duration_;
    double gripper_motion_duration_;

    const std::unordered_map<int, JointTargets> targets_{{
        {0, {0.0, 80.0, 100.0, 100.0}},   // Home: ready position with open gripper
        {1, {0.0, 157.0, 44.0, 0.0}},     // Pick: reach down and close gripper
        {2, {180.0, 157.0, 44.0, 100.0}}, // Place: rotate base and open gripper
        {3, {0.0, 80.0, 100.0, 0.0}},     // Sleep: park arm with closed gripper
    }};
};
}  // namespace robotic_arm_remote

RCLCPP_COMPONENTS_REGISTER_NODE(robotic_arm_remote::TaskServo)
