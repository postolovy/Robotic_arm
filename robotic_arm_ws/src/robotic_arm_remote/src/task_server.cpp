#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotic_arm_msgs/action/robotic_arm_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <cmath>
#include <map>
#include <memory>
#include <thread>
#include <vector>

using namespace std::placeholders;

namespace robotic_arm_remote
{
namespace
{
constexpr double kShoulderNeutralDeg = 90.0;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kElbowScale = M_PI / 360.0;  // maps 0-180 deg to 0-90 deg
constexpr double kGripperScale = -M_PI / 360.0;  // negative direction closes the gripper

double baseServoToRad(double servo_deg)
{
    return servo_deg * kDegToRad;
}

double shoulderServoToRad(double servo_deg)
{
    return (kShoulderNeutralDeg - servo_deg) * kDegToRad;
}

double elbowServoToRad(double servo_deg)
{
    return servo_deg * kElbowScale;
}

double gripperServoToRad(double servo_deg)
{
    return servo_deg * kGripperScale;
}
}

class TaskServer : public rclcpp::Node
{
public:
    explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("TaskServer", options)
    {
        action_server_ = rclcpp_action::create_server<robotic_arm_msgs::action::RoboticArmTask>(
            this,
            "task_server",
            std::bind(&TaskServer::goalCallback, this, _1, _2),
            std::bind(&TaskServer::cancelCallback, this, _1),
            std::bind(&TaskServer::acceptedCallback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Starting the Task Action Server");
    }

private:
    struct ServoTargets
    {
        double base_deg;
        double shoulder_deg;
        double elbow_deg;
        double gripper_deg;
    };

    using GoalHandle = rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::RoboticArmTask>;

    const std::map<int, ServoTargets> task_targets_{{
        // Values chosen to reproduce the working Arduino sequence
        {0, {0.0, 80.0, 100.0, 100.0}},   // Home
        {1, {0.0, 157.0, 44.0, 0.0}},     // Pick
        {2, {180.0, 157.0, 44.0, 100.0}}, // Place
        {3, {0.0, 80.0, 100.0, 0.0}},     // Sleep
    }};

    rclcpp_action::Server<robotic_arm_msgs::action::RoboticArmTask>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
    std::vector<double> arm_joint_goal_;
    std::vector<double> gripper_joint_goal_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const robotic_arm_msgs::action::RoboticArmTask::Goal> goal)
    {
        (void)uuid;
        if(task_targets_.count(goal->task_number) == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Received goal request with unknown task_number: %d", goal->task_number);
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Received goal request with task_number: %d", goal->task_number);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        if(arm_move_group_)
        {
            arm_move_group_->stop();
        }
        if(gripper_move_group_)
        {
            gripper_move_group_->stop();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        if(!arm_move_group_)
        {
            arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        }
        if(!gripper_move_group_)
        {
            gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        }

        auto result = std::make_shared<robotic_arm_msgs::action::RoboticArmTask::Result>();
        const auto goal = goal_handle->get_goal();

        const auto target_it = task_targets_.find(goal->task_number);
        if(target_it == task_targets_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown task number: %d", goal->task_number);
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        const ServoTargets& target = target_it->second;

        arm_joint_goal_ = {
            baseServoToRad(target.base_deg),
            shoulderServoToRad(target.shoulder_deg),
            elbowServoToRad(target.elbow_deg)
        };
        const double gripper_angle = gripperServoToRad(target.gripper_deg);
        gripper_joint_goal_ = {gripper_angle, -gripper_angle};

        RCLCPP_INFO(this->get_logger(),
                    "Setting arm servo targets [deg]: base=%.1f, shoulder=%.1f, elbow=%.1f | gripper=%.1f",
                    target.base_deg, target.shoulder_deg, target.elbow_deg, target.gripper_deg);

        arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
        gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

        const bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
        const bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);

        if(!arm_within_bounds || !gripper_within_bounds)
        {
            RCLCPP_ERROR(this->get_logger(), "Target joint position(s) out of bounds");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        const bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        const bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(this->get_logger(), "Planning results - Arm: %s, Gripper: %s",
                    arm_plan_success ? "SUCCESS" : "FAILED",
                    gripper_plan_success ? "SUCCESS" : "FAILED");

        if(arm_plan_success && gripper_plan_success)
        {
            arm_move_group_->move();
            gripper_move_group_->move();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "One or more planners failed");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(robotic_arm_remote::TaskServer)
