// ROS2関連ライブラリ
#include <rclcpp/rclcpp.hpp>
#include "kk_driver_msg/msg/epb_cmd.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"


class ControlNode : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;

    void update(){
        // 10ミリ秒ごとに実行される
    }
public:
    ControlNode(const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("controller_node",name_space,options){
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 0.01秒ごとに実行
            std::bind(&ControlNode::update, this));
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}