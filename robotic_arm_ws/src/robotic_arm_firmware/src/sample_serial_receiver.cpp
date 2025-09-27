#include <rclcpp/rclcpp.hpp> //main ros2 cpp library
#include <std_msgs/msg/string.hpp> 
#include <chrono> //library for timer 
#include <libserial/SerialPort.h>

using namespace std::chrono_literals; 

class SampleSerialReceiver : public rclcpp::Node //inherited Node class 
{ 
    public: 
    SampleSerialReceiver() : Node("sample_serial_receiver") //default constructor 
    { 
        declare_parameter<std::string>("port", "/dev/ttyUSB0"); 
        std::string port = get_parameter("port").as_string(); 
        
        pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10); //create publisher 
        timer_ = create_wall_timer(0.01s, std::bind(&SampleSerialReceiver::timerCallBack, this)); //create timer function and binds it to the class

        arduino_.Open(port); 
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }

    ~SampleSerialReceiver()
    { 
        arduino_.Close(); 
    }

    void timerCallBack()
    { 
        auto message = std_msgs::msg::String(); 
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        { 
            arduino_.ReadLine(message.data); 
        }
        pub_->publish(message); 
    }

private: 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; //create pub_ shared pointer of type Publisher 
    rclcpp::TimerBase::SharedPtr timer_; ////create timer_ shared pointer of type TimerBase 
    LibSerial::SerialPort arduino_; 
};


int main(int argc, char* argv[]){ 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<SampleSerialReceiver>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return(0); 
}