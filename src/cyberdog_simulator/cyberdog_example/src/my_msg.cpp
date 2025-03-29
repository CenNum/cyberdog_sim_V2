#include <iostream>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cyberdog_msg/msg/apply_force.hpp>

enum class ControlParameterValueKind : uint64_t {
  kDOUBLE = 1,
  kS64 = 2,
  kVEC_X_DOUBLE = 3,  // for template type derivation
  kMAT_X_DOUBLE = 4   // for template type derivation
};

class ExampleNode: public rclcpp::Node
    {
    public:
        ExampleNode(std::string node_name):Node(node_name){};
        ~ExampleNode(){};
    };

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto example_node_=std::make_shared<ExampleNode>("cyberdogmsg_node");
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr para_pub_;
    
    para_pub_=example_node_->create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter",10);


    auto param_message_ = cyberdog_msg::msg::YamlParam();

    sleep(1);

    //改变控制模式，使其可以被仿真
    param_message_.name = "use_rc";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(0);
    param_message_.is_user = 0;
    para_pub_->publish(param_message_);
    std::cout<<"switch to gamepad control model..."<<std::endl;

    sleep(1);

    //切换到复位站立控制模式
    param_message_.name = "control_mode";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(12);
    param_message_.is_user = 0;

    para_pub_->publish(param_message_);
    std::cout<<"recovery stand ..."<<std::endl;

    sleep(5);

    //切换到运动控制模式
    param_message_.name = "control_mode";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(11);
    param_message_.is_user = 0;
    para_pub_->publish(param_message_);
    std::cout<<"locomotion ..."<<std::endl;

    sleep(5);

    //改变身体的俯仰角度
    std::array<double, 12> vecxd_value_;
    vecxd_value_[0] = 0.2;
    vecxd_value_[2] = 0.25;

    param_message_.name = "des_roll_pitch_height";
    param_message_.kind = uint64_t(ControlParameterValueKind::kVEC_X_DOUBLE);
    param_message_.vecxd_value = vecxd_value_;
    param_message_.is_user = 1;
    para_pub_->publish(param_message_);
    std::cout<<"set roll angle to 0.2 ..."<<std::endl;

    sleep(7);

    //切换到复位站立控制模式
    param_message_.name = "control_mode";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(12);
    param_message_.is_user = 0;

    para_pub_->publish(param_message_);
    std::cout<<"recovery stand ..."<<std::endl;

    

    rclcpp::shutdown();
    return 0;
}