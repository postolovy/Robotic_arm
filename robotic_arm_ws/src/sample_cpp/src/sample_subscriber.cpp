#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 

using std::placeholders::_1;

class SampleSubscriber : public rclcpp::Node
{
public : 
    SampleSubscriber() : Node("sample_subscriber") 
    {
        sub_ = create_subscription<std_msgs::msg::String>("sample_topic", 10, std::bind(&SampleSubscriber::msgCallBack, this, _1)); //create subscriber
    }
    
    void msgCallBack(std_msgs::msg::String::SharedPtr msg) const //callback function for subscriber
    { 
        RCLCPP_INFO(get_logger(), "Received message: %s", msg->data.c_str()); //console print
    }

private: 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; 
};

int main(int argc, char*argv[])
{ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<SampleSubscriber>();
    rclcpp::spin(node);   
    rclcpp::shutdown(); 
    return(0); 
}