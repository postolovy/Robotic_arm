#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotic_arm_msgs/action/robotic_arm_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <memory> 
#include <thread> 

using namespace std::placeholders;

namespace robotic_arm_remote
{
class TaskActionServer : public rclcpp::Node 
{ 
public: 
    explicit TaskActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("TaskServer", options)
    { 
        action_server_ = rclcpp_action::create_server<robotic_arm_msgs::action::RoboticArmTask>(
            this, 
            "task_server", 
            std::bind(&TaskActionServer::goalCallback, this, _1, _2), 
            std::bind(&TaskActionServer::cancelCallback, this, _1), 
            std::bind(&TaskActionServer::acceptedCallback, this, _1)
        );
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting the Task Action Server"); 
    }
private: 
    rclcpp_action::Server<robotic_arm_msgs::action::RoboticArmTask>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
    std::vector<double> arm_joint_goal_, gripper_joint_goal_;  

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotic_arm_msgs::action::RoboticArmTask::Goal> goal)
    { 
        (void)uuid;  // Marking parameter as intentionally unused
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with task_number: " << goal->task_number);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::RoboticArmTask>> goal_handle)
    { 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel the goal");
        if(arm_move_group_){ 
            arm_move_group_->stop(); 
        }
        if(gripper_move_group_){ 
            gripper_move_group_->stop(); 
        }
        (void)goal_handle; 
        return rclcpp_action::CancelResponse::ACCEPT;  
    }

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::RoboticArmTask>> goal_handle)
    { 
        std::thread{std::bind(&TaskActionServer::execute, this, _1), goal_handle}.detach(); 
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::RoboticArmTask>> goal_handle)
    { 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal"); 
        if(!arm_move_group_)
        { 
            arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        }
        if(!gripper_move_group_)
        { 
            gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        }
        auto result = std::make_shared<robotic_arm_msgs::action::RoboticArmTask::Result>(); 

        switch (goal_handle->get_goal()->task_number)
        {
        case 0:
            arm_joint_goal_ = {0.0, 0.0, 0.0};
            gripper_joint_goal_ = {-0.7, 0.7}; 
            break;
        case 1: 
            arm_joint_goal_ = {-1.14, -0.6,-0.7};
            gripper_joint_goal_ = {0.0, 0.0};
            break;
        case 2: 
            arm_joint_goal_ = {-1.57, 0.0,-0.9};
            gripper_joint_goal_ = {0.0, 0.0};
            break; 
        default:
            RCLCPP_ERROR(get_logger(), "Unknown task number: %d", goal_handle->get_goal()->task_number);
            return;
        }

        arm_move_group_->setStartState(*arm_move_group_->getCurrentState()); 
        gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState()); 

        bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_); 
        bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);
        
        if(!arm_within_bounds || !gripper_within_bounds){ 
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Target postions out of boundaries");
            return; 
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan; 
        bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); 
        bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS); 

        if(arm_plan_success && gripper_plan_success){ 
            arm_move_group_->move(); 
            gripper_move_group_->move();
        } else { 
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
            return; 
        }

        result->success = true; 
        goal_handle->succeed(result);
    }
};
} 

RCLCPP_COMPONENTS_REGISTER_NODE(robotic_arm_remote::TaskActionServer)