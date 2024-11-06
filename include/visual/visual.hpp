#pragma once

#include "../utility/logging.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <ostream>
#include <unordered_map>
#include <vector>

namespace fan {

constexpr auto PI = 3.141592654;

class Visual {
public:
    explicit Visual()
        : cap_() {
        if (!cap_.isOpened()) {
            throw std::runtime_error{"Failed to open camera!"};
        }
        color_identify_pra_.insert(std::make_pair(COLOR::RED, COLORPRA{red_min_, red_max_}));
        color_identify_pra_.insert(std::make_pair(COLOR::BLUE, COLORPRA{blue_min_, blue_max_}));
        color_identify_pra_.insert(
            std::make_pair(COLOR::YELLOW, COLORPRA{yellow_min_, yellow_max_}));
        color_identify_pra_.insert(std::make_pair(COLOR::BLACK, COLORPRA{black_min_, black_max_}));
    }

    explicit Visual(int index)
        : cap_(index) {
        if (!cap_.isOpened()) {
            throw std::runtime_error{"Failed to open camera!"};
        }
        color_identify_pra_.insert(std::make_pair(COLOR::RED, COLORPRA{red_min_, red_max_}));
        color_identify_pra_.insert(std::make_pair(COLOR::BLUE, COLORPRA{blue_min_, blue_max_}));
        color_identify_pra_.insert(
            std::make_pair(COLOR::YELLOW, COLORPRA{yellow_min_, yellow_max_}));
        color_identify_pra_.insert(std::make_pair(COLOR::BLACK, COLORPRA{black_min_, black_max_}));
    }

    struct COLORPRA {
        cv::Scalar min;
        cv::Scalar max;
    };

    enum SHAPE { circle, rectangle };

    struct POSITION {
        SHAPE shape;
        cv::Point2d position;
        double distance;
    };

    ~Visual() { image_.release(); }

    enum COLOR { NONE, RED, BLUE, BLACK, YELLOW };

    const cv::Mat& get_image() {
        cap_.read(image_);
        return image_;
    }

    const cv::Mat& debug() { return debug_dst_; }

    void identify(const COLOR& color) {
        position_.clear();
        cap_.read(image_);

        cv::Mat hsv(image_.size(), image_.type());
        cv::Mat dst(image_.size(), image_.type());
        auto color_pra_ = color_identify_pra_.find(color)->second;

        cvtColor(image_, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        inRange(hsv, color_pra_.min, color_pra_.max, mask);
        // 掩模到原图的转换
        for (int r = 0; r < image_.rows; r++) {
            for (int c = 0; c < image_.cols; c++) {
                if (mask.at<uchar>(r, c) == 255) {
                    dst.at<cv::Vec3b>(r, c) = image_.at<cv::Vec3b>(r, c);
                }
            }
        }
        // cv::imshow("dst", dst);
        // cv::imshow("image", image_);

        // debug_dst_.release();
        dst.copyTo(debug_dst_);

        cvtColor(dst, dst, cv::COLOR_BGR2GRAY); // 转化为灰度图
        cv::medianBlur(dst, dst, 9);            // 高斯滤波

        cv::dilate(dst, dst, 9);
        // cv::imshow("dst_",dst);

        // cv::waitKey(200);

        std::vector<std::vector<cv::Point>> contours;
        contours.clear();
        findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        std::cout << "总个数：" << contours.size() << std::endl;

        // for (std::vector<std::vector<cv::Point>>::iterator it = contours.begin();
        //      it != contours.end(); ++it) {

        //   std::cout << it->size() << std::endl;
        //   int area = contourArea(*it);

        //   if (area <= 1000) {
        //     // 进行接下来的操作
        //     contours.erase(it);
        //   }
        // }

        for (size_t i = 0; i < contours.size(); i++) {
            int area = contourArea(contours[i]);
            // 打印出面积
            std::cout << area << std::endl;
            if (area <= 2000) {
                // 进行接下来的操作
                std::cout << "删除之前的个数：" << contours.size() << std::endl;
                contours.erase(contours.begin() + i);
                std::cout << "删除之后的个数：" << contours.size() << std::endl;
            }
        }

        // LOG_INFO("识别到的数量%d", contours.size());

        // 计算每个轮廓对象的矩
        std::vector<cv::Moments> contours_moments(contours.size());
        for (size_t i = 0; i < contours.size(); i++) {
            POSITION temp_position;
            // 计算矩
            contours_moments[i] = moments(contours[i]);

            temp_position.position = cv::Point(
                static_cast<float>(contours_moments[i].m10 / contours_moments[i].m00 - 320),
                static_cast<float>(contours_moments[i].m01 / contours_moments[i].m00 - 240));
            // 图像中心Center(x0, y0)=(m10/m00,m01/m00)
            // cv::RotatedRect rect = cv::minAreaRect(contours[i]);
            // cv::Size2d size = rect.size;

            // 计算轮廓的最大尺寸（假设轮廓是圆形的）
            // auto diameter_in_pixels = cv::max(size.width, size.height);

            // 计算距离
            double distance;
            // (focal_length * real_diameter) /
            // (diameter_in_pixels * sensor_horizontal_resolution / 1000);

            temp_position.distance = distance;

            float peri = arcLength(contours[i], true);

            double d = peri / PI;
            LOG_INFO("直径:%d", d);
            // cv::waitKey();
            // 角点向量，仅存储轮廓中的角点
            std::vector<std::vector<cv::Point>> conPoly(contours.size());
            approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);

            // 根据检测到的角点数量判断形状
            if (conPoly[i].size() > 4) {
                temp_position.shape = circle;
            } else if (conPoly[i].size() <= 4) {
                // 有明显的角点，可能是矩形或其他多边形
                temp_position.shape = rectangle;
            }

            position_.push_back(temp_position);
        }

        if (position_.empty())
            LOG_INFO("没有识别到物体");

        for (size_t i = 0; i < position_.size(); i++) {
            if (position_[i].shape == circle) {
            }
            // LOG_INFO("第%d个形状为：圆形", i + 1);
            else {}
            // LOG_INFO("第%d个形状为：方形", i + 1);
            // std::cout << "距离：" << position_[i].position << std::endl;
        }
        image_.release();
        dst.release();
        hsv.release();
        mask.release();
        contours_moments.clear();

        return;
    }

    void update_status() {

        identify(RED);
        color_position_.insert(
            std::pair<COLOR, std::vector<POSITION>>(RED, *(new std::vector<POSITION>(position_))));
        identify(BLUE);
        color_position_.insert(
            std::pair<COLOR, std::vector<POSITION>>(BLUE, *(new std::vector<POSITION>(position_))));
        identify(BLACK);
        color_position_.insert(std::pair<COLOR, std::vector<POSITION>>(
            BLACK, *(new std::vector<POSITION>(position_))));
        identify(YELLOW);
        color_position_.insert(std::pair<COLOR, std::vector<POSITION>>(
            YELLOW, *(new std::vector<POSITION>(position_))));
    }

    // auto get_position(){return &position_;}

    const std::unordered_map<COLOR, std::vector<POSITION>>* get_color_position() {
        return &color_position_;
    }

    const std::vector<POSITION>* get_position() { return &position_; }

private:
    cv::VideoCapture cap_;
    cv::Mat image_;
    cv::Mat debug_dst_;

    const cv::Scalar red_min_ = cv::Scalar(0, 115, 0);
    const cv::Scalar red_max_ = cv::Scalar(4, 255, 156);

    const cv::Scalar blue_min_ = cv::Scalar(107, 96, 47);
    const cv::Scalar blue_max_ = cv::Scalar(114, 255, 255);

    const cv::Scalar yellow_min_ = cv::Scalar(13, 127, 27);
    const cv::Scalar yellow_max_ = cv::Scalar(32, 255, 255);

    const cv::Scalar black_min_ = cv::Scalar(0, 0, 0);
    const cv::Scalar black_max_ = cv::Scalar(180, 255, 35);

    std::unordered_map<COLOR, COLORPRA> color_identify_pra_;
    std::vector<POSITION> position_;
    std::unordered_map<COLOR, std::vector<POSITION>> color_position_;

    // POSITION position_;
};

} // namespace fan