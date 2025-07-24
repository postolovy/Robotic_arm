#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 

using namespace std::place_holders::_1;

class SampleSubscriber : public rclcpp::Node
{
public : 
    SampleSubscriber() : Node("SampleSubscriber") counter_(0); 
    {
        sub_ = create_subscriber<std_msgs::msg::String>("topic_name", 10, std::bind(&SampleSubscriber::msgCallBack, this, _1)); //create subscriber
    }
    
    void msgCallBack(std::msgs::msg::Strinf &msg) const //callback function for subscriber
    { 
        RCLCPP_INFO(get_logger(), "Received message: %s", msg->data.c_str()); //console print
    }

private: 
    rclcpp::Subsrciption<std_msgs::String>::SharedPtr sub_; 
};

int main(int argc, char*argv[])
{ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<SampleSubscriber>();
    rclcpp::spin(node);   
    rclcpp::shutdown(); 
    return(0); 
}