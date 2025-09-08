#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotic_arm_msgs/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory> 

using namespace std::chrono_literals; 

using namespace std::placeholders; 

namespace sample_cpp
{
class SampleActionClient : public rclcpp::Node
{ 
public:
    explicit SampleActionClient(const rclcpp::NodeOptions& options) : Node("sample_action_client", options)
    { 
        client_ = rclcpp_action::create_client<robotic_arm_msgs::action::Fibonacci>(this, "fibonacci"); 
        timer_ = create_wall_timer(1s, std::bind(&SampleActionClient::timerCallback, this)); 
    }

    

private:
    rclcpp_action::Client<robotic_arm_msgs::action::Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_; 

    void timerCallback()
    { 
        timer_->cancel(); 
        
        if(!client_->wait_for_action_server())
        { 
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server is not available after waiting");
            rclcpp::shutdown(); 
        } 

        auto goal_msg = robotic_arm_msgs::action::Fibonacci::Goal(); 
        goal_msg.order = 10; 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending goal"); 

        auto send_goal_options = rclcpp_action::Client<robotic_arm_msgs::action::Fibonacci>::SendGoalOptions(); 
        send_goal_options.goal_response_callback = std::bind(&SampleActionClient::goalCallback, this, _1);
        send_goal_options.feedback_callback = std::bind(&SampleActionClient::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&SampleActionClient::resultCallback, this, _1);

        client_->async_send_goal(goal_msg, send_goal_options); 
    }

    void goalCallback(const rclcpp_action::ClientGoalHandle<robotic_arm_msgs::action::Fibonacci>::SharedPtr& goal_handle)
    { 
        if(!goal_handle)
        { 
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by the server"); 
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by the server, waiting for the result");
        }
    }

    void feedbackCallback(rclcpp_action::ClientGoalHandle<robotic_arm_msgs::action::Fibonacci>::SharedPtr, 
                              const std::shared_ptr<const robotic_arm_msgs::action::Fibonacci::Feedback> feedback)
    {  
        std::stringstream ss; 
        ss << "Next number in sequence received: "; 
        for(auto number : feedback->partial_sequence)
        { 
            ss << number << " "; 
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str()); 
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<robotic_arm_msgs::action::Fibonacci>::WrappedResult& result)
    { 
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED: 
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted"); 
                return; 
            case rclcpp_action::ResultCode::CANCELED: 
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was cancled");
                return;
            default: 
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
                return;
        }

        std::stringstream ss; 
        ss << "Result: "; 
        for(auto number : result.result->sequence)
        { 
            ss << number << " "; 
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str()); 
        rclcpp::shutdown();
    }
}; 
} 

RCLCPP_COMPONENTS_REGISTER_NODE(sample_cpp::SampleActionClient)