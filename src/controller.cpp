#include <rclcpp/rclcpp.hpp>
#include "kk_driver_msg/msg/gm6020_cmd.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ControlNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<kk_driver_msg::msg::Gm6020Cmd>::SharedPtr gm_publisher_;
    rclcpp::Publisher<kk_driver_msg::msg::PwmCmd>::SharedPtr pwm_publisher_;
    rclcpp::Publisher<kk_driver_msg::msg::MotorCmd>::SharedPtr motor_publisher_;

    sensor_msgs::msg::Joy::SharedPtr latest_joy_;
    float prev_error_right_ = 0.0, prev_error_left_ = 0.0;
    float integral_right_ = 0.0, integral_left_ = 0.0;
    const float Kp = 5.0, Ki = 0.5, Kd = 1.0;
    const float dt = 0.01;

  void update() {
    if (!latest_joy_) return;

    // Joyメッセージが未初期化 or サイズ不足ならスキップ
    if (latest_joy_->axes.size() < 2 || latest_joy_->buttons.size() < 8) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Joy message too short");
        return;
    }

        float lx = latest_joy_->axes[0];
        float ly = latest_joy_->axes[1];
        float power = 0.3;

        // 前輪制御
        auto front_cmd = kk_driver_msg::msg::Gm6020Cmd();
        front_cmd.motor_id = {1, 3};
        front_cmd.duty = {-power * (ly - lx), power * (ly + lx)};
        gm_publisher_->publish(front_cmd);

        // 後輪 PID 制御
        float ref_right = front_cmd.duty[0];
        float ref_left = front_cmd.duty[1];
        float error_r = ref_right;
        float error_l = ref_left;
        integral_right_ += error_r * dt;
        integral_left_ += error_l * dt;
        float diff_r = (error_r - prev_error_right_) / dt;
        float diff_l = (error_l - prev_error_left_) / dt;
        float output_r = Kp * error_r + Ki * integral_right_ + Kd * diff_r;
        float output_l = Kp * error_l + Ki * integral_left_ + Kd * diff_l;
        prev_error_right_ = error_r;
        prev_error_left_ = error_l;

        auto rear_cmd = kk_driver_msg::msg::Gm6020Cmd();
        rear_cmd.motor_id = {2, 4};
        rear_cmd.duty = {output_r, output_l};
        gm_publisher_->publish(rear_cmd);

        // 後輪展開/収納
        bool b1 = latest_joy_->buttons[1]; // 〇
        bool b2 = latest_joy_->buttons[2]; // □
        if (b1) {
            rear_cmd.duty = {1000.0f, 1000.0f};
            gm_publisher_->publish(rear_cmd);
        } else if (b2) {
            rear_cmd.duty = {-1000.0f, -1000.0f};
            gm_publisher_->publish(rear_cmd);
        }

        // 支援物資供給（PWM）
        static bool prev_b7 = false, prev_b6 = false;
        bool b7 = latest_joy_->buttons[7]; // R2
        bool b6 = latest_joy_->buttons[6]; // L2
        if ((b7 && !prev_b7) || (b6 && !prev_b6)) {
            auto pwm_cmd = kk_driver_msg::msg::PwmCmd();
            pwm_cmd.port = {0};
            pwm_cmd.target = {static_cast<uint16_t>(b7 ? 1000 : 2000)};
            pwm_publisher_->publish(pwm_cmd);
        }
        prev_b7 = b7;
        prev_b6 = b6;

        // ガス栓昇降（モータ）
        bool b3 = latest_joy_->buttons[3]; // △
        bool b0 = latest_joy_->buttons[0]; // ×
        auto motor_cmd = kk_driver_msg::msg::MotorCmd();
        motor_cmd.port = {0};  // 修正箇所
        motor_cmd.ctrl = {1};  // 速度制御モード
        if (b3)
            motor_cmd.duty = {3000.0f};
        else if (b0)
            motor_cmd.duty = {-3000.0f};
        else
            motor_cmd.duty = {0.0f};
        motor_publisher_->publish(motor_cmd);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        latest_joy_ = msg;
    }

public:
    ControlNode() : Node("controller_node") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::update, this));

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&ControlNode::joyCallback, this, std::placeholders::_1));

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
