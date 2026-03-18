#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <algorithm>  // std::min, std::max

class Go2StreamViewer : public rclcpp::Node {
public:
    Go2StreamViewer() : Node("go2_stream_viewer") {
        // 실제 네트워크 인터페이스 이름으로 수정
        std::string iface = "enp3s0";

        std::string pipeline =
            "udpsrc address=230.1.1.1 port=1720 multicast-iface=" + iface +
            " ! application/x-rtp, media=video, encoding-name=H264 "
            " ! rtph264depay "
            " ! h264parse "
            " ! avdec_h264 "
            " ! videoconvert "
            " ! video/x-raw,width=1280,height=720,format=BGR "
            " ! appsink drop=1";

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "VideoCapture를 열지 못했습니다.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Go2 비디오 스트림 열기 성공");

        // obstacle_detected 퍼블리셔 생성
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/obstacle_detected", 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&Go2StreamViewer::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "1. timer_callback entered"
        );

        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "cap_ is not opened");
            return;
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "2. cap_.read(frame) failed"
            );
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "3. frame is empty"
            );
            return;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "4. frame read success: cols=%d rows=%d",
            frame.cols, frame.rows
        );

        // 1) grayscale 변환
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 2) 안전한 중앙 ROI 설정
        int roi_w = std::min(200, gray.cols);
        int roi_h = std::min(200, gray.rows);
        int roi_x = std::max(0, (gray.cols - roi_w) / 2);
        int roi_y = std::max(0, (gray.rows - roi_h) / 2);

        cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
        cv::Mat center_region = gray(roi);

        // 3) 평균 밝기 계산
        double mean_brightness = cv::mean(center_region)[0];

        // 4) threshold 비교
        bool obstacle_detected = (mean_brightness < 70.0);

        // 5) ROS2 Bool 메시지로 publish
        std_msgs::msg::Bool msg;
        msg.data = obstacle_detected;
        obstacle_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "5. ROI=(x=%d, y=%d, w=%d, h=%d), brightness=%.2f, obstacle_detected=%s",
            roi_x, roi_y, roi_w, roi_h,
            mean_brightness,
            obstacle_detected ? "true" : "false"
        );

        // 화면 표시 유지
        cv::imshow("Go2 Perception Test", frame);
        cv::waitKey(1);
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2StreamViewer>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}