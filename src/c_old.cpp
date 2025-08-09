#include <rclcpp/rclcpp.hpp>
#include "kk_driver_msg/msg/gm6020_cmd.hpp"
#include "kk_driver_msg/msg/gm6020_status.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ControlNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<kk_driver_msg::msg::Gm6020Status>::SharedPtr gm_subscriber_;
    rclcpp::Publisher<kk_driver_msg::msg::Gm6020Cmd>::SharedPtr gm_publisher_;
    rclcpp::Publisher<kk_driver_msg::msg::PwmCmd>::SharedPtr pwm_publisher_;
    rclcpp::Publisher<kk_driver_msg::msg::MotorCmd>::SharedPtr motor_publisher_;

    sensor_msgs::msg::Joy::SharedPtr latest_joy_;
    kk_driver_msg::msg::Gm6020Status latest_gm6020;
    float prev_error_right_ = 0.0, prev_error_left_ = 0.0;
    float integral_right_ = 0.0, integral_left_ = 0.0;
    const float Kp = 5.0, Ki = 0.5, Kd = 1.0;
    const float dt = 0.01;
    int32_t gm6020_org[4] = {0, 0, 0, 0};

  void update() {
    if (!latest_joy_) return;

    // Joyメッセージが未初期化 or サイズ不足ならスキップ
    if (latest_joy_->axes.size() < 2 || latest_joy_->buttons.size() < 8) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Joy message too short");
        return;
    }

    static uint8_t init_counter = 0;
    if (latest_joy_->buttons[12]){
        init_counter = 0;
    }
    
    // タイや位置初期化
    if (init_counter < 60){
        init_counter++;
        auto front_cmd = kk_driver_msg::msg::Gm6020Cmd();
        front_cmd.motor_id = {1, 3};
        front_cmd.duty = {-0.3f, 0.3f};
        gm_publisher_->publish(front_cmd);
        
        for (uint8_t i = 0; i < 4; i++){
            gm6020_org[i] = latest_gm6020.position[i];
        }
        return;
    }
    int32_t gm6020_enc[4];
    for (uint8_t i = 0; i < 4; i++){
        gm6020_enc[i] = latest_gm6020.position[i] - gm6020_org[i];
    }
    

        static bool is_ex = false;
        static bool prev_b10 = false;
        bool b10 = latest_joy_->buttons[10]; // L_Z
        if (b10 && !prev_b10) is_ex = !is_ex;
        prev_b10 = b10;

        float lx = latest_joy_->axes[0]*0.3;
        float ly = latest_joy_->axes[1];
        float power = 0.6;
        float power_fix = 0.075;
        float pid_p = 0.1;

        float r_base = -power * (ly - lx);
        float l_base =  power * (ly + lx);
        float r_diff = pid_p * (gm6020_enc[0] + gm6020_enc[2]);
        float l_diff = pid_p * (gm6020_enc[1] + gm6020_enc[3]);  
        
        // 前輪制御
        auto front_cmd = kk_driver_msg::msg::Gm6020Cmd();
        bool b4 = latest_joy_->buttons[4]; // L_Z
        if (is_ex){
            front_cmd.motor_id = {1, 2, 3, 4, 5, 6};
            front_cmd.duty = {r_base* power_fix,r_base, l_base * power_fix, l_base , r_base* 0.2, l_base* 0.2};
            // front_cmd.duty = {r_base* power_fix,r_base, l_base * power_fix, l_base };
            if(b4){
                    front_cmd.duty = {r_base,r_base, l_base, l_base };
            }
        } else {
            front_cmd.motor_id = {1, 2, 3, 4, 5, 6};
            front_cmd.duty = {-r_base,r_base* power_fix, -l_base, l_base * power_fix , r_base* 0.2, l_base* 0.2};
            if(b4){
                front_cmd.duty = {-r_base,r_base, -l_base, l_base };
            }
        }
        gm_publisher_->publish(front_cmd);

        printf("(%d, %d)=>%d\n", gm6020_enc[1] , gm6020_enc[3], gm6020_enc[1] + gm6020_enc[3]);

        // 後輪 PID 制御
        // float ref_right = front_cmd.duty[0];
        // float ref_left = front_cmd.duty[1];
        // float error_r = ref_right;
        // float error_l = ref_left;
        // integral_right_ += error_r * dt;
        // integral_left_ += error_l * dt;
        // float diff_r = (error_r - prev_error_right_) / dt;
        // float diff_l = (error_l - prev_error_left_) / dt;
        // float output_r = Kp * error_r + Ki * integral_right_ + Kd * diff_r;
        // float output_l = Kp * error_l + Ki * integral_left_ + Kd * diff_l;
        // prev_error_right_ = error_r;
        // prev_error_left_ = error_l;

        // auto rear_cmd = kk_driver_msg::msg::Gm6020Cmd();
        // rear_cmd.motor_id = {2, 4};
        // rear_cmd.duty = {output_r, output_l};
        // gm_publisher_->publish(rear_cmd);

        // 後輪展開/収納
        // bool b1 = latest_joy_->buttons[1]; // 〇
        // bool b2 = latest_joy_->buttons[2]; // □
        // if (b1) {
        //     rear_cmd.duty = {1000.0f, 1000.0f};
        //     gm_publisher_->publish(rear_cmd);
        // } else if (b2) {
        //     rear_cmd.duty = {-1000.0f, -1000.0f};
        //     gm_publisher_->publish(rear_cmd);
        // }

        // 支援物資供給（PWM）
        static bool prev_b7 = false, prev_b6 = false;
        bool b7 = latest_joy_->buttons[7]; // R2
        bool b6 = latest_joy_->buttons[6]; // L2
        if ((b7 && !prev_b7) || (b6 && !prev_b6)) {
            auto pwm_cmd = kk_driver_msg::msg::PwmCmd();
            pwm_cmd.port = {0};
            pwm_cmd.spd = {15};
            pwm_cmd.target = {static_cast<uint16_t>(b7 ? 500 : 800)};
            pwm_publisher_->publish(pwm_cmd);
        }
        prev_b7 = b7;
        prev_b6 = b6;

        // ガス栓昇降（モータ）
        bool b3 = latest_joy_->buttons[1]; // △
        bool b0 = latest_joy_->buttons[0]; // ×
        auto motor_cmd = kk_driver_msg::msg::MotorCmd();
        motor_cmd.port = {0};  // 修正箇所
        if (b3)
            motor_cmd.duty = {0.9f};
        else if (b0)
            motor_cmd.duty = {-0.9f};
        else
            motor_cmd.duty = {0.0f};
        motor_publisher_->publish(motor_cmd);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        latest_joy_ = msg;
    }
    
    void gmCallback(const kk_driver_msg::msg::Gm6020Status::SharedPtr msg) {
         for (uint8_t i = 0; i< 7; i++){
            latest_gm6020.position[i] = msg->position[i];
            latest_gm6020.speed[i] = msg->speed[i];
            latest_gm6020.torque[i] = msg->torque[i];

        }
        latest_gm6020 = *msg;
    }

public:
    ControlNode() : Node("controller_node") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ControlNode::update, this));

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&ControlNode::joyCallback, this, std::placeholders::_1));

        gm_subscriber_ = this->create_subscription<kk_driver_msg::msg::Gm6020Status>(
            "/gm6020/status", rclcpp::QoS(10),
            std::bind(&ControlNode::gmCallback, this, std::placeholders::_1));
        latest_gm6020 = kk_driver_msg::msg::Gm6020Status();

        gm_publisher_ = this->create_publisher<kk_driver_msg::msg::Gm6020Cmd>("/gm6020/cmd", 10);
        pwm_publisher_ = this->create_publisher<kk_driver_msg::msg::PwmCmd>("/pwm/cmd", 10);
        motor_publisher_ = this->create_publisher<kk_driver_msg::msg::MotorCmd>("/mtr/cmd", 10);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
