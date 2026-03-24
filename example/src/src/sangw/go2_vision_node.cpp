#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// Custom AI Architecture
#include "AI_Inferencer.hpp"
#include "YOLOInferencer.hpp"

// Custom ROS2 Message Array (Assuming standard casing for generated header)
#include "unitree_go2_vision_msgs/msg/detection.hpp"
#include "unitree_go2_vision_msgs/msg/detection_array.hpp"

class Go2VisionNode : public rclcpp::Node {
 public:
  Go2VisionNode() : Node("go2_vision_node") {
    // 실제 네트워크 인터페이스 이름으로 수정 (환경에 맞게 변경)
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

    RCLCPP_INFO(this->get_logger(), "Go2 비디오 스트림 연결 성공");

    // Set up the Vision Data Publisher
    vision_pub_ =
        this->create_publisher<unitree_go2_vision_msgs::msg::DetectionArray>(
            "/vision_detections", 10);

    // Initialize YOLO Backend (Replace with Hailo later effortlessly via
    // Interface)
    std::string model_path =
        "/home/sangw/unitree_ros2/yolo_models/yolov4-tiny.weights";
    std::string config_path =
        "/home/sangw/unitree_ros2/yolo_models/yolov4-tiny.cfg";
    std::string classes_path =
        "/home/sangw/unitree_ros2/yolo_models/coco.names";

    try {
      inferencer_ = std::make_unique<YOLOInferencer>(model_path, config_path,
                                                     classes_path);
      RCLCPP_INFO(
          this->get_logger(),
          "YOLO 분류기(Opencv DNN) 로드 성공! 바운딩 박스 표시가 시작됩니다.");
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
                  "초기 테스트 설정: 실제 모델 경로가 없으므로 YOLO 로드 실패 "
                  "- 모델 경로 수정 필요!");
      // This allows the node to run and compile without throwing fatal errors
      // immediately if models aren't in pwd.
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 FPS to save Edge CPU
        std::bind(&Go2VisionNode::timer_callback, this));
  }

 private:
  void timer_callback() {
    if (!cap_.isOpened()) {
      return;
    }

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "비디오 프레임을 수신할 수 없습니다");
      return;
    }

    // [수정된 부분] 카메라 테두리(Edge) 하얗게 날아가는 오버익스포저 현상 방지
    // 처리 첫 프레임이 들어올 때 중앙은 1.0, 엣지는 0.4로 그라데이션이 들어간
    // 가우시안 마스크를 생성합니다.
    if (edge_darkening_mask_.empty()) {
      cv::Mat mask_x =
          cv::getGaussianKernel(frame.cols, frame.cols / 1.5, CV_32F);
      cv::Mat mask_y =
          cv::getGaussianKernel(frame.rows, frame.rows / 1.5, CV_32F);
      cv::Mat kernel = mask_y * mask_x.t();

      double min_val, max_val;
      cv::minMaxLoc(kernel, &min_val, &max_val);
      kernel = kernel / max_val;  // 중앙을 1.0으로 정규화

      std::vector<cv::Mat> channels = {kernel, kernel, kernel};
      cv::merge(channels, edge_darkening_mask_);

      // 중앙 밝기 비율 1.0, 가장자리는 0.4(40% 수준)로 확 어둡게 깎습니다.
      float edge_ratio = 0.2F;
      edge_darkening_mask_ =
          edge_darkening_mask_ * (1.0F - edge_ratio) + edge_ratio;

      RCLCPP_INFO(
          this->get_logger(),
          "가장자리 밝기 차단 마스크 생성 완료 (테두리 밝기 %d%% 로 제한)",
          static_cast<int>(edge_ratio * 100));
    }

    // 라즈베리 파이에서도 CPU 부하를 주지 않도록 매 틱마다 만들어둔 마스크를
    // 고속 곱셈 연산
    frame.convertTo(frame, CV_32FC3);
    cv::multiply(frame, edge_darkening_mask_, frame);
    frame.convertTo(frame, CV_8UC3);

    // 추가적인 전체 밝기 조절이 필요없으므로 이전 단순 convertTo는 삭제합니다.

    unitree_go2_vision_msgs::msg::DetectionArray msg_array;
    msg_array.header.stamp = this->get_clock()->now();
    msg_array.header.frame_id = "go2_front_camera";

    if (inferencer_) {
      // Run inference decoupled from everything else
      std::vector<DetectionResult> detections = inferencer_->infer(frame);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "객체 탐지 수: %zu", detections.size());

      // Populate ROS2 message array
      for (const auto& det : detections) {
        unitree_go2_vision_msgs::msg::Detection det_msg;
        det_msg.class_id = det.class_id;
        det_msg.class_name = det.class_name;
        det_msg.score = det.score;
        det_msg.x_min = det.x_min;
        det_msg.y_min = det.y_min;
        det_msg.x_max = det.x_max;
        det_msg.y_max = det.y_max;
        det_msg.center_x = det.center_x;
        det_msg.center_y = det.center_y;
        det_msg.distance = det.distance;

        msg_array.detections.push_back(det_msg);

        // Draw bounding box for visual debugging
        cv::rectangle(frame, cv::Point(det.x_min, det.y_min),
                      cv::Point(det.x_max, det.y_max), cv::Scalar(0, 255, 0),
                      2);
        cv::putText(frame,
                    det.class_name + " " +
                        std::to_string(det.distance).substr(0, 4) + "m",
                    cv::Point(det.x_min, det.y_min - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      }
    }

    // Publish to Control Node
    vision_pub_->publish(msg_array);

    cv::imshow("Go2 Edge Vision", frame);
    cv::waitKey(1);
  }

  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_go2_vision_msgs::msg::DetectionArray>::SharedPtr
      vision_pub_;
  std::unique_ptr<AI_Inferencer> inferencer_;
  cv::Mat edge_darkening_mask_;  // 테두리 밝기 조절 전용 마스크
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2VisionNode>();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
