#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory> 
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SampleLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{ 
public:
    explicit SampleLifeCycleNode(const std::string & node_name, bool intra_process_comms = false) 
                : LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    { 

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    { 
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SampleLifeCycleNode::msgCallback, this, _1));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifecycle node on_configure() called"); 
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    { 
        sub_.reset(); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifecycle node on_shutdown() called"); 
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    { 
        sub_.reset(); 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifecycle node on_cleanup() called"); 
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
    { 
        LifecycleNode::on_activate(state);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifecycle node on_activate() called");
        std::this_thread::sleep_for(2s); 
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
    { 
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifecycle node on_deactivate() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void msgCallback(const std_msgs::msg::String::SharedPtr msg)
    { 
        auto state = get_current_state(); 
        if(state.label() == "active")
        { 
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Lifecycle node heard: " << msg->data.c_str());
        }
    } 

private: 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; 
}; 

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv); 
    rclcpp::executors::SingleThreadedExecutor ste;
    auto sample_lifecycle_node = std::make_shared<SampleLifeCycleNode>("sample_lifecycle_node"); 
    ste.add_node(sample_lifecycle_node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown(); 
    return 0;
}