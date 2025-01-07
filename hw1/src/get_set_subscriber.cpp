#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std;
using std::placeholders::_1;

class Sub : public rclcpp::Node{
    public:
        Sub() : Node("sub"),get_count(0), set_count(0)
        {
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
            subscriber_ = this->create_subscription<std_msgs::msg::String>("/get_set",qos_profile,std::bind(&Sub::count_callback, this, _1));
            get_publisher_ = this->create_publisher<std_msgs::msg::String>("/get_count",qos_profile);
            set_publisher_ = this->create_publisher<std_msgs::msg::String>("/set_count",qos_profile);
            
            
        }
        

    private:
      
        void count_callback(const std_msgs::msg::String::SharedPtr msg) 
        {   
            

            RCLCPP_INFO(this->get_logger(), "read '%s'",msg->data.c_str());


            if(msg->data == "get"){
                // get_count++;
                auto recieve_msg = std_msgs::msg::String();
                recieve_msg.data = std::to_string(++get_count);
                get_publisher_->publish(recieve_msg);
            }
            else if(msg->data=="set"){
                auto recieve_msg = std_msgs::msg::String();
                recieve_msg.data = std::to_string(++set_count);
                set_publisher_->publish(recieve_msg);
            }
        }
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_publisher_;

        size_t get_count;
        size_t set_count;
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

