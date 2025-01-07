#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <fstream>

using namespace std;
using namespace std::chrono_literals;

class Pub : public rclcpp::Node{
    public:
        Pub() : Node("pub"), count(0){
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
            publisher_ = this->create_publisher<std_msgs::msg::String>("/get_set",qos_profile);
            timer = this->create_wall_timer(1000ms,std::bind(&Pub::timer_callback,this));

             //입력 스트림 객체
            file_.open("/home/jinju/ws/src/hw/hw1/src/query.dat");
            if (file_.is_open()){
                cout << "파일이 열림"<<endl;
            }
            else{
                cout << "파일이 안 열림"<<endl;
            }      
        }
        

    private:
        

        void timer_callback(){
            auto msg = std_msgs::msg::String();
            
            if(getline (file_,line)){
                if(line.find("get") != string::npos){//string::npos = -1
                    msg.data = "get";
                }
                
                else if(line.find("set") != string::npos){
                    msg.data = "set";
                }
            }
            else{
                cout << "not" << endl;
                msg.data = "not";
            }

            

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",msg.data.c_str());
            publisher_->publish(msg);
        }
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count;
        ifstream file_;
        string line;
        


        

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// int main(){
//     ifstream file; //입력 스트림 객체

//     // file.open("/home/jinju/ws/src/hw/hw1/src/query.dat"); //파일 열기
//     file.open("/home/jinju/ws/src/hw/hw1/src/text.txt"); //파일 열기

//     if (file.is_open()){
//         cout << "파일이 열림"<<endl;
//     }
//     else{
//         cout << "파일이 안 열림"<<endl;
//     }

//     string line;
  
//     while(getline (file,line)){
        
//         if(line.find("get") != string::npos){//string::npos = -1
//             cout << "get" << endl;
            
//         }
//         else if(line.find("set") != string::npos){
//             cout << "set" << endl;
//         }
//     }
//     file.close();
    

    
//     for (int i=0;i<10 ;i++){
        
//     }

// }