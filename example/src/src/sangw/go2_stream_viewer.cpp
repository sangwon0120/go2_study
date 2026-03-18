#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <string>

class Go2StreamViewer : public rclcpp::Node {
public:
    Go2StreamViewer() : Node("go2_stream_viewer") {
        // 네 PC에서 Go2와 통신하는 네트워크 인터페이스 이름으로 바꿔야 함
        // 예: enp3s0, eth0, wlan0
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

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&Go2StreamViewer::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        if (!cap_.isOpened()) {
            return;
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "프레임 읽기 실패"
            );
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "빈 프레임 수신"
            );
            return;
        }

        cv::imshow("Go2 Front Camera Stream", frame);
        cv::waitKey(1);
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2StreamViewer>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}