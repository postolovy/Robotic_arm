#include <robotic_arm_controllers/robotic_arm_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp> 
#include <sstream>
#include <cmath>

namespace robotic_arm_controller
{ 

std::string compensateZeros(int value)
{ 
    std::string compensate_zeros = ""; 
    if(value < 10)
    { 
        compensate_zeros = "00"; 
    } else if (value < 100){
        compensate_zeros = "0";
    } 
    return compensate_zeros; 
}

RoboticArmInterface::RoboticArmInterface()
{ 

}

RoboticArmInterface::~RoboticArmInterface()
{ 
    if(arduino_.IsOpen())
    { 
        try
        {
            arduino_.Close(); 
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoboticArmInterface"), "Something went wrong while closing serial connection with port " << port_); 
        }
        
    }
}

CallbackReturn RoboticArmInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info); 

    if(result != CallbackReturn::SUCCESS)
    { 
        return result; 
    }

    try
    {
        port_ = info_.hardware_parameters.at("port"); 
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("RoboticArmInterface"), "No serial port provided! Aborting");
        return CallbackReturn::FAILURE; 
    }

    position_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size()); 
    prev_position_commands_.reserve(info_.joints.size()); 

    return CallbackReturn::SUCCESS; 
}

std::vector<hardware_interface::StateInterface> RoboticArmInterface::export_state_interfaces()
{ 
    std::vector<hardware_interface::StateInterface> state_interfaces; 
    for(size_t i = 0; i < info_.joints.size(); i++)
    { 
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboticArmInterface::export_command_interfaces()
{ 
    std::vector<hardware_interface::CommandInterface> command_interfaces; 
    for(size_t i = 0; i < info_.joints.size(); i++)
    { 
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }

    return command_interfaces; 
}

CallbackReturn RoboticArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Starting the robot's hardware"); 
    position_commands_ = {0.0, 0.0, 0.0, 0.0};  //may need to be changed as an initial state differs
    prev_position_commands_ = {0.0, 0.0, 0.0, 0.0};
    position_states_ = {0.0, 0.0, 0.0, 0.0};

    try
    { 
        arduino_.Open(port_); 
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    { 
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoboticArmInterface"), "Something went wrong while interacting with the port " << port_);
        return CallbackReturn::FAILURE; 
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Hardware started, ready to take commands"); 
    return CallbackReturn::SUCCESS; 
}

CallbackReturn RoboticArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{ 
    RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Deactivating the robot's hardware"); 

    if(arduino_.IsOpen())
    { 
        try
        {
            arduino_.Close(); 
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoboticArmInterface"), "Something went wrong while closing serial connection with port " << port_); 
            return CallbackReturn::FAILURE; 
        }

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Hardware stopped"); 
        return CallbackReturn::SUCCESS; 
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Hardware was already stopped"); 
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoboticArmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{ 
    position_states_ = position_commands_; //assume that it always perfectly reach the command point
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboticArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{ 
    std::ostringstream commands_stream;
    commands_stream << "[";
    for(size_t i = 0; i < position_commands_.size(); ++i)
    {
        commands_stream << position_commands_.at(i);
        if(i + 1 < position_commands_.size())
        {
            commands_stream << ", ";
        }
    }
    commands_stream << "]";
    RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Current position commands: %s", commands_stream.str().c_str());

    if(position_commands_ == prev_position_commands_)
    { 
        return hardware_interface::return_type::OK; 
    }

    std::string msg; //example:: b043,s092,e030,g000 base, shoulder, elbow
    int bicep_joint = static_cast<int>(std::lround(position_commands_.at(0) * (180.0 / M_PI) / 0.6)); // In angle(converted to steps), stepper motor 1 // change it !!!!
    msg.append("b");
    msg.append(compensateZeros(bicep_joint));
    msg.append(std::to_string(bicep_joint)); 
    msg.append(","); 
    int arm1_joint = 90 - static_cast<int>(std::lround(position_commands_.at(1) * 180.0 / M_PI));
    msg.append("s");
    msg.append(compensateZeros(arm1_joint)); 
    msg.append(std::to_string(arm1_joint)); 
    msg.append(","); 
    int arm2_joint = 90 + static_cast<int>(std::lround(position_commands_.at(2) * 180.0 / M_PI));
    msg.append("e");
    msg.append(compensateZeros(arm2_joint)); 
    msg.append(std::to_string(arm2_joint)); 
    msg.append(","); 
    int gripper = 90 + static_cast<int>(std::lround(position_commands_.at(3) * 180.0 / M_PI));
    msg.append("g");
    msg.append(compensateZeros(gripper)); 
    msg.append(std::to_string(gripper)); 
    msg.append(",");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RoboticArmInterface"), "Commands: " << msg);

    try
    {
        arduino_.Write(msg); 
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoboticArmInterface"), "Something went wrong while sending the message " << msg << " to the port " << port_);
        return hardware_interface::return_type::ERROR; 
    }
    
    prev_position_commands_ = position_commands_; 
    return hardware_interface::return_type::OK;
}
}

PLUGINLIB_EXPORT_CLASS(robotic_arm_controller::RoboticArmInterface, hardware_interface::SystemInterface); 

