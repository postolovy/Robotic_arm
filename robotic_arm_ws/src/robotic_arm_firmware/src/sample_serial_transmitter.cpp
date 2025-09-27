#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
#include <libserial/SerialPort.h>

using std::placeholders::_1;

class SampleSerialTransmitter : public rclcpp::Node
{
public : 
    SampleSerialTransmitter() : Node("sample_serial_transmitter") 
    {
        declare_parameter<std::string>("port", "dev/ttyUSB0"); 
        std::string port_ = get_parameter("port").as_string(); 

        sub_ = create_subscription<std_msgs::msg::String>("serial_transmitter", 10, std::bind(&SampleSerialTransmitter::msgCallBack, this, _1)); //create subscriber
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    
    ~SampleSerialTransmitter()
    { 
        arduino_.Close(); 
    }

    void msgCallBack(std_msgs::msg::String::SharedPtr msg) //callback function for subscriber
    { 
        RCLCPP_INFO(get_logger(), "Received message: %s", msg->data.c_str()); //console print
        arduino_.Write(msg->data);
    }


private: 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; 
    LibSerial::SerialPort arduino_; 
};

int main(int argc, char*argv[])
{ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<SampleSerialTransmitter>();
    rclcpp::spin(node);   
    rclcpp::shutdown(); 
    return(0); 
}