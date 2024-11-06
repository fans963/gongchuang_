#pragma once

#include "joystick.hh"
#include <bit>

namespace fan {

class MyJoyStick : public Joystick {
public:
    explicit MyJoyStick()
        : Joystick() {
        if (!this->isFound()) {
            printf("open failed.\n");
        } else {
            this->sample(&event);
        }
    }

    explicit MyJoyStick(int joystickNumber)
        : Joystick(joystickNumber) {
        if (!this->isFound()) {
            printf("open failed.\n");
        } else {
            this->sample(&event);
        }
    };

    explicit MyJoyStick(std::string devicePath)
        : Joystick(devicePath) {
        if (!this->isFound()) {
            printf("open failed.\n");
        } else {
            this->sample(&event);
        }
    };

    explicit MyJoyStick(std::string devicePath, bool blocking)
        : Joystick(devicePath, blocking) {
        if (!this->isFound()) {
            printf("open failed.\n");
        } else {
            this->sample(&event);
        }
    };

    ~MyJoyStick(){};

    struct Vector {
        constexpr static inline Vector zero() { return {0, 0}; }
        double x, y;
    };

    struct __attribute__((packed)) Keyboard {
        constexpr static inline Keyboard zero() {
            constexpr char zero = 0;
            return std::bit_cast<Keyboard>(zero);
        }
        bool A : 1;
        bool B : 1;
        bool X : 1;
        bool Y : 1;
    };

    Vector joystick_right() { return joystick_right_; }
    Vector joystick_left() { return joystick_left_; }

    void update_status() {
        this->sample(&event);
        if (event.isAxis()) {
            if (event.number == 0) {
                joystick_left_.x = event.value;
            }
            if (event.number == 1) {
                joystick_left_.y = -event.value;
            }
            if (event.number == 3) {
                joystick_right_.x = event.value;
            }
            if (event.number == 4) {
                joystick_right_.y = -event.value;
            }
            if (event.number == 2) {
                trigger_left_.x = event.value;  // 扳机按下为正
            }
            if (event.number == 5) {
                trigger_right_.y = event.value; // 扳机按下为正
            }
            if (event.number == 6) {
                x_.x = event.value;             // 扳机按下为正
            }
            if (event.number == 7) {
                x_.y = -event.value;            // 扳机按下为正
            }
        }
        if (event.isButton()) {
            if (event.value == 0) {
                keyboard_   = Keyboard::zero();
                keyboard_.A = 0;
            }
            if (event.value == 0) {
                keyboard_   = Keyboard::zero();
                keyboard_.B = 0;
            }
            if (event.value == 0) {
                keyboard_   = Keyboard::zero();
                keyboard_.A = 0;
            }
            if (event.value == 0) {
                keyboard_   = Keyboard::zero();
                keyboard_.B = 0;
            }
        }
    };

private:
    JoystickEvent event;

    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_  = Vector::zero();
    Vector x_              = Vector::zero();

    Vector trigger_left_  = Vector::zero();
    Vector trigger_right_ = Vector::zero();

    Keyboard keyboard_ = Keyboard::zero();
};
} // namespace fan