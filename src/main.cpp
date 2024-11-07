// #include <cmath>
// #include <csignal>

// #include <numbers>

// #include "device/imu.hpp"
// #include "forwarder/cboard.hpp"
// #include "singleton/singleton.hpp"
// #include "utility/pid_calculator.hpp"
// #include "joystick/myjoystick.hpp"
// #include "state_machine/state_machine.hpp"

// class MyRobot : public rmcs::forwarder::CBoard {
// public:
//     explicit MyRobot(uint16_t usb_pid)
//         : CBoard(usb_pid) {}

//     void update() {
//         imu_.update_status();
//         double siny_cosp = 2 * (imu_.q0() * imu_.q3() + imu_.q1() *
//         imu_.q2()); double cosy_cosp = 1 - 2 * (imu_.q2() * imu_.q2() +
//         imu_.q3() * imu_.q3()); double yaw_angle = std::atan2(siny_cosp,
//         cosy_cosp); yaw_angle_ = yaw_angle_ +
//         get_minimum_angle_diff(yaw_angle, yaw_angle_);

//         joystick.update_status();
//         control_yaw_angle_ += 0.006 * joystick.joystick_left().y;

//         if (++divider_ == 4) { // Divide to 1000/4 = 250Hz
//             divider_ = 0;

//             double err = control_yaw_angle_ - yaw_angle_;
//             double control_yaw_velocity = yaw_angle_control_pid_.update(err);

//             enum class Event : uint8_t { None, Run, Slow, Stop, TurnLeft,
//             TurnRight }; struct __attribute__((packed)) Command {
//                 const uint8_t header = 0xa5;
//                 float x;
//                 float y;
//                 float w;
//                 Event event;
//                 const uint8_t end = 0xb6;
//             } command;

//             command.x = 1;
//             command.y = static_cast<float>(joystick.joystick_right().y);
//             command.w = static_cast<float>(std::clamp(control_yaw_velocity,
//             -1.0, 1.0));

//             transmit_buffer_.add_uart1_transmission(
//                 std::launder(reinterpret_cast<std::byte*>(&command)),
//                 sizeof(command));
//             transmit_buffer_.trigger_transmission();
//             LOG_INFO("%.2f %.2f %.2f", command.x, command.y, command.w);
//         }
//     }

// private:
//     static double get_minimum_angle_diff(double a, double b) {
//         double diff = std::fmod(a - b, 2 * std::numbers::pi);
//         if (diff < -std::numbers::pi)
//             diff += 2 * std::numbers::pi;
//         else if (diff > std::numbers::pi)
//             diff -= 2 * std::numbers::pi;
//         return diff;
//     }

//     void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z)
//     override {
//         imu_.store_accelerometer_status(x, y, z);
//     }

//     void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override
//     {
//         imu_.store_gyroscope_status(x, y, z);
//     }

//     int divider_ = 0;

//     double yaw_angle_ = 0.0, control_yaw_angle_ = 0.0;
//     rmcs::utility::PidCalculator yaw_angle_control_pid_{0.7, 0.0, 9.0};

//     rmcs::device::Imu imu_{1000.0, 0.2, 0.0};
//     fan::MyJoyStick joystick;

//     TransmitBuffer transmit_buffer_{*this, 16};
// };

// int main() {

//   static std::atomic<bool> running = true;
//   std::signal(SIGINT, [](int sig) {
//     (void)sig;
//     running = false;
//   });

//   MyRobot my_robot{0x4669};

//   std::thread thread{[&my_robot]() { my_robot.handle_events(); }};
//   thread.detach();

//   using namespace std::chrono_literals;
//   constexpr double update_rate = 1000.0;
//   constexpr auto update_period =
//       std::chrono::round<std::chrono::steady_clock::duration>(1.0s /
//                                                               update_rate);

//   auto next_iteration_time = std::chrono::steady_clock::now();
//   while (running.load(std::memory_order::relaxed)) {
//     my_robot.update();
//     next_iteration_time += update_period;
//     std::this_thread::sleep_until(next_iteration_time);
//     }
// }

#include "state_machine/state_machine.hpp"
#include <atomic>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    static std::atomic<bool> running = true;
    std::signal(SIGINT, [](int sig) {
        (void)sig;
        running = false;
    });

    auto statemachine_ = std::make_shared<State_machine>(0xce3b, 0, "10.31.2.219", 9999);

    std::thread handle_thread_{[&statemachine_]() { statemachine_->handle_events(); }};
    handle_thread_.detach();

    std::thread send_thread{[&statemachine_]() { statemachine_->entrypoint(); }};
    send_thread.detach();

    std::thread tcp_thread_{[&statemachine_]() { statemachine_->tcp_start(); }};
    tcp_thread_.detach();

    std::thread debug_thread_{[&statemachine_]() {
        rclcpp::spin(statemachine_);
        rclcpp::shutdown();
    }};
    debug_thread_.detach();

    using namespace std::chrono_literals;
    constexpr double update_rate = 1000.0;
    constexpr auto update_period =
        std::chrono::round<std::chrono::steady_clock::duration>(1.0s / update_rate);
    auto next_iteration_time = std::chrono::steady_clock::now();
    while (running.load(std::memory_order::relaxed)) {
        statemachine_->send();
        next_iteration_time += update_period;
        std::this_thread::sleep_until(next_iteration_time);
    }

    return 1;
}