#pragma once

#include "device/imu.hpp"
#include "forwarder/cboard.hpp"
#include "tcp/tcp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visual/visual.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <ostream>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

namespace existing_color {
constexpr auto my_color        = fan::Visual::RED;
constexpr auto important_color = fan::Visual::BLACK;
constexpr auto dangerous_color = fan::Visual::YELLOW;
constexpr auto your_color      = fan::Visual::BLUE;
} // namespace existing_color

// TODO!!!!!!!!!!!!!!
// 组合 > 继承
class State_machine
    : public rclcpp::Node
    , public rmcs::forwarder::CBoard
    , public TCPServer {
public:
    explicit State_machine(
        uint16_t usb_pid, int cam_index = 2, const std::string& host = "127.0.0.1", int port = 9000)
        : Node{"car", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , CBoard(usb_pid)
        , TCPServer(host, port)
        , cap_(cam_index) {
        // Declare and acquire `turtlename` parameter
        auto temp_command_ = std::make_shared<Command>(
            0xa5, static_cast<float>(0), static_cast<float>(0), static_cast<float>(0), Event::None,
            false, 0xb6);
        command.store(temp_command_.get());
        pose_name_       = this->declare_parameter<std::string>("car", "mycar");
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image", 20);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        std::ostringstream stream;
        stream << "/" << pose_name_.c_str() << "/pose";
        std::string topic_name = stream.str();

        using namespace std::chrono_literals;
        publish_timer_ = this->create_wall_timer(10ms, [this] { timer_callback(); });
    }

    ~State_machine() {}

    enum class Event : uint8_t { None, Run, Slow, Stop, TurnLeft, TurnRight };

    // TODO!!!!!!!!!!!
    struct __attribute__((packed)) Command {
        const uint8_t header = 0xa5;
        float x              = float(0);
        float y              = float(0);
        float w              = float(0);
        Event event          = Event::None;
        // Angle angle_;
        bool roboarm      = false;
        const uint8_t end = 0xb6;
    };

    void send() {
        while (true) {
            auto temp_command = command.load(std::memory_order::relaxed);
            if (temp_command->roboarm)
                LOG_INFO("%.2f %.2f %.2f true", temp_command->x, temp_command->y,
                temp_command->w);
            else
                LOG_INFO("%.2f %.2f %.2f false", temp_command->x, temp_command->y,
                temp_command->w);
            transmit_buffer_.add_uart1_transmission(
                std::launder(reinterpret_cast<std::byte*>(&temp_command)), sizeof(Command));
            transmit_buffer_.trigger_transmission();
        }
    }

    void entrypoint() { rescue(); }

    void update() {

        while (tcp_flag.load(std::memory_order::relaxed)) {};
        rescue();
    }

    // TODO: use Eigen
    struct Vector {
        constexpr static inline Vector zero() { return {0, 0}; }
        double x, y;
    };

protected:
    void clamping() {
        if (tcp_flag.load(std::memory_order::relaxed)) {
            return;
        }
        // cap_.clear_position();
        // auto position = cap_.identify(my_color);
        // 控制机械臂进行夹取并检测是否成功夹取
        while (true) {}
        to_safe_zone();
    }

    void to_safe_zone() {
        if (tcp_flag.load(std::memory_order::relaxed)) {
            return;
        }
        // 读imu的数据到安全区松开机械臂并做校准

        const std::vector<fan::Visual::POSITION>* temp_position_;
        size_t rectangle_index = 0;
        /*distance具体参数要进行测试*/
        while (true) {
            if (tcp_flag.load(std::memory_order::relaxed)) {
                return;
            }

            while ([this, &temp_position_, &rectangle_index]() -> bool {
                cap_.clear_position();
                temp_position_ = cap_.get_position();
                if (temp_position_->empty()) {
                    return true;
                }
                for (size_t i = 0; i < temp_position_->size(); i++) {
                    if (temp_position_->at(i).shape == fan::Visual::rectangle) {
                        rectangle_index = i;
                        return false;
                    }
                }
                return true;
            }()) {
                using namespace std::chrono_literals;
                search_object(rescuing_color_, true, 0s);
            };

            Command temp_command_;
            temp_command_.x = static_cast<float>(temp_position_->at(rectangle_index).position.x);
            temp_command_.y = static_cast<float>(temp_position_->at(rectangle_index).position.y);
            temp_command_.w = static_cast<float>(0);
            temp_command_.event   = Event::None;
            temp_command_.roboarm = true;
            // command.angle_ = Angle{0, 0, 0, 0};
            command.store(&temp_command_);
        }
    }

    void rescue() {
        if (tcp_flag.load(std::memory_order::relaxed)) {
            return;
        }
        cap_.update_status();

        if (cap_.get_color_position()->find(existing_color::my_color)->second.empty())
            rescuing_color_ = existing_color::my_color;
        if (cap_.get_color_position()->find(existing_color::important_color)->second.empty())
            rescuing_color_ = existing_color::important_color;
        if (cap_.get_color_position()->find(existing_color::dangerous_color)->second.empty())
            rescuing_color_ = existing_color::dangerous_color;

        // 实现救援颜色策略
        // rescuing_color_;

        if (flag == 0) {
            flag            = 1;
            rescuing_color_ = existing_color::my_color;
        }
        /*用.进行一个数据的取*/
        // cap_.clear_position();
        // cap_.identify(rescuing_color_);

        /*distance具体参数要进行测试*/
        const std::vector<fan::Visual::POSITION>* temp_position_;
        int distance_min_index = 0;

        while (true) {
            if (tcp_flag.load(std::memory_order::relaxed)) {
                stop();
            }
            while ([this, &temp_position_, &distance_min_index]() -> bool {
                cap_.clear_position();
                cap_.identify(fan::Visual::BLUE);
                temp_position_ = cap_.get_position();
                if (temp_position_->empty()) {
                    return true;
                }
                for (size_t i = 0; i < temp_position_->size(); i++) {
                    if (temp_position_->at(i).shape == fan::Visual::circle) {
                        distance_min_index = i;
                        return false;
                    }
                }
                return true;
            }()) {
                using namespace std::chrono_literals;
                search_object(fan::Visual::BLUE, false, 2s);
                std::cout << "循环" << std::endl;
            };
            std::cout << "跳出循环" << std::endl;

            for (size_t i = distance_min_index; i < temp_position_->size(); i++) {
                if (temp_position_->at(distance_min_index).position.y
                        > temp_position_->at(i).position.y
                    && temp_position_->at(i).shape == fan::Visual::circle) {
                    distance_min_index = i;
                }
            }
            Command temp_command_;
            temp_command_.x =
                static_cast<float>(-temp_position_->at(distance_min_index).position.y / 360);
            temp_command_.y =
                static_cast<float>(temp_position_->at(distance_min_index).position.x / 360);
            temp_command_.w       = static_cast<float>(0);
            temp_command_.event   = Event::None;
            temp_command_.roboarm = true;

            command.store(&temp_command_);
        }
        clamping();
    }
    void stop() {
        // 停止（判断场上球的数量或者接收某个信号）
        if (tcp_flag.load(std::memory_order::relaxed)) {
            return;
        }
    }

private:
    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_accelerometer_status(x, y, z);
        pose_callback();
    }

    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_gyroscope_status(x, y, z);
        pose_callback();
    }

    void handle_client(int client_fd) override {
        char buffer[sizeof(Command)];
        while (tcp_flag.load(std::memory_order::relaxed)) {
            ssize_t bytes_received = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            if (bytes_received <= 0) {
                // 如果bytes_received小于等于0，表示客户端已断开连接
                Command temp_command_;
                temp_command_.x       = static_cast<float>(0);
                temp_command_.y       = static_cast<float>(0);
                temp_command_.w       = static_cast<float>(0);
                temp_command_.event   = Event::None;
                temp_command_.roboarm = false;
                command.store(&temp_command_);
                LOG_INFO("客户端断开连接\n");
                tcp_flag.store(false);
                break;
            }
            command.store(reinterpret_cast<Command*>(buffer));
        }
        close(client_fd);
    }

    void timer_callback() {
        std::shared_ptr<sensor_msgs::msg::Image> msg_(new sensor_msgs::msg::Image);
        cv_bridge::CvImage(
            std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, *cap_.debug())
            .toImageMsg(*msg_);
        image_publisher_->publish(*msg_);
    };

    void pose_callback() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id  = pose_name_.c_str();
        imu_.update_status();
        t.transform.rotation.x = imu_.q0();
        t.transform.rotation.y = imu_.q1();
        t.transform.rotation.z = imu_.q2();
        t.transform.rotation.w = imu_.q3();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void search_object(
        const fan::Visual::COLOR& color, const bool& roboarm,
        const std::chrono::seconds& wait_time) {
        using namespace std::chrono_literals;
        auto entry_time_ = std::chrono::steady_clock::now();
        Command temp_command_;
        while (cap_.get_position()->empty()) {
            cap_.clear_position();
            cap_.identify(color);
            std::cout << "search_object:" << std::endl;
            if (std::chrono::steady_clock::now() - entry_time_ >= wait_time) {
                temp_command_.x       = static_cast<float>(0);
                temp_command_.y       = static_cast<float>(0);
                temp_command_.w       = static_cast<float>(0.05);
                temp_command_.event   = Event::None;
                temp_command_.roboarm = roboarm;
                // command.angle_ = Angle{0, 0, 0, 0};
                command.store(&temp_command_);
            }
        }
        temp_command_.x       = static_cast<float>(0);
        temp_command_.y       = static_cast<float>(0);
        temp_command_.w       = static_cast<float>(0);
        temp_command_.event   = Event::None;
        temp_command_.roboarm = roboarm;
        // command.angle_ = Angle{0, 0, 0, 0};
        command.store(&temp_command_);
    }

    int flag = 0; // 开始标志位

    TransmitBuffer transmit_buffer_{*this, 16};

    rmcs::device::Imu imu_{1000.0, 0.2, 0.0};

    fan::Visual cap_;
    fan::Visual::COLOR rescuing_color_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<rclcpp::TimerBase> publish_timer_;

    std::string pose_name_;

    std::atomic<Command*> command;
};