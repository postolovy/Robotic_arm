#include <rclcpp/rclcpp.hpp> //main ros2 cpp library
#include <std_msgs/msg/string.hpp> 
#include <chrono> //library for timer 

using namespace std::chrono_literals; 

class SamplePublisher : public rclcpp::Node //inherited Node class 
{ 
    public: 
    SamplePublisher() : Node("sample_publisher"), counter_(0) //default constructor 
    { 
        pub_ = create_publisher<std_msgs::msg::String>("sample_topic", 10); //create publisher 
        timer_ = create_wall_timer(1s, std::bind(&SamplePublisher::timerCallBack, this)); //create timer function and binds it to the class

        RCLCPP_INFO(get_logger(), "Publising at 1 Hz");  //console print
    }

    void timerCallBack(){ 
        auto message = std_msgs::msg::String(); //assigns string type to message 
        message.data = "ROS2 Message - counter" + std::to_string(counter_++); //fills message data 
        pub_->publish(message); // publishes the message
    }

private: 
    unsigned int counter_; //counter declaration 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; //create pub_ shared pointer of type Publisher 
    rclcpp::TimerBase::SharedPtr timer_; ////create timer_ shared pointer of type TimerBase 
};


int main(int argc, char* argv[]){ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<SamplePublisher>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return(0); 
}