#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotic_arm_msgs/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory> 
#include <thread> 

using namespace std::placeholders;

namespace sample_cpp
{
class SampleActionServer : public rclcpp::Node 
{ 
public: 
    explicit SampleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("sample_action_server", options)
    { 
        action_server_ = rclcpp_action::create_server<robotic_arm_msgs::action::Fibonacci>(
            this, 
            "fibonacci", 
            std::bind(&SampleActionServer::goalCallback, this, _1, _2), 
            std::bind(&SampleActionServer::cancelCallback, this, _1), 
            std::bind(&SampleActionServer::acceptedCallback, this, _1)
        );
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting the Action Server"); 
    }
private: 
    rclcpp_action::Server<robotic_arm_msgs::action::Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotic_arm_msgs::action::Fibonacci::Goal> goal)
    { 
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with order: " << goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::Fibonacci>> goal_handle)
    { 
        std::thread{std::bind(&SampleActionServer::execute, this, _1), goal_handle}.detach(); 
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::Fibonacci>> goal_handle)
    { 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal"); 
        rclcpp::Rate loop_rate(1); 

        const auto goal = goal_handle->get_goal(); 
        auto feedback = std::make_shared<robotic_arm_msgs::action::Fibonacci::Feedback>();
        auto& sequence = feedback->partial_sequence;

        sequence.push_back(0); 
        sequence.push_back(1);

        auto result = std::make_shared<robotic_arm_msgs::action::Fibonacci::Result>(); 
        for(int i = 1; (i < goal->order) && (rclcpp::ok()); i++)
        { 
            if(goal_handle->is_canceling())
            { 
                result->sequence = sequence; 
                goal_handle->canceled(result); 
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Canceled"); 
                return; 
            }

            sequence.push_back(sequence[i] + sequence[i-1]); 
            goal_handle->publish_feedback(feedback); 
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing Feedback");
            loop_rate.sleep(); 
        }

        if(rclcpp::ok())
        { 
            result->sequence = sequence; 
            goal_handle->succeed(result); 
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
        }
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotic_arm_msgs::action::Fibonacci>> goal_handle)
    { 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel the goal");
        return rclcpp_action::CancelResponse::ACCEPT;  
    }

};
} 

RCLCPP_COMPONENTS_REGISTER_NODE(sample_cpp::SampleActionServer)