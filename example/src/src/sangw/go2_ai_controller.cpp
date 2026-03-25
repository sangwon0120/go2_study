#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "common/ros2_sport_client.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

// AI Vision Detections Custom Message
#include "unitree_go2_vision_msgs/msg/detection_array.hpp"
#include "unitree_go2_vision_msgs/msg/detection.hpp"

using namespace std::chrono_literals;

enum class RobotState : std::uint8_t {
  SEARCHING,
  APPROACHING,
  STOPPED
};

/**
 * @brief Decoupled AI Controller Node for Unitree Go2
 * Subscribes to /vision_detections from the Vision Node.
 * This guarantees the Control Loop operates quickly on a Raspberry Pi, 
 * independent of heavy YOLO/Hailo Inference cycles.
 */
class Go2AIController : public rclcpp::Node {
 public:
  Go2AIController()
      : Node("go2_ai_controller"),
        sport_client_(this),
        current_state_(RobotState::SEARCHING)
  {
    RCLCPP_INFO(this->get_logger(), "Go2 AI 제어 기반 노드 시작");

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Go2AIController::timer_callback, this));

    // 오도메트리/로봇 상태 구독
    state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "lf/sportmodestate", 10,
        [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
          latest_state_ = *msg;
          state_received_ = true;
        }
    );

    // 완벽히 분리된(Decoupled) 비전 AI 결과 구독
    vision_sub_ = this->create_subscription<unitree_go2_vision_msgs::msg::DetectionArray>(
        "/vision_detections", 10,
        [this](const unitree_go2_vision_msgs::msg::DetectionArray::SharedPtr msg) {
          latest_detections_ = *msg;
        }
    );
  }

 private:
  void timer_callback()
  {
    auto now = this->now();
    unitree_api::msg::Request req;

    if (!state_received_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "SportModeState 대기 중...");
      return;
    }

    bool target_detected = false;
    float target_distance = 0.0F;
    float target_center_x = 0.0F;

    // AI가 판단한 객체 정보 기반으로 판단 (ex. 'person' 찾기)
    for (const auto& det : latest_detections_.detections) {
      if (det.class_name == "person" && det.score > 0.5F) {
        target_detected = true;
        target_distance = det.distance;
        target_center_x = det.center_x;
        break; // 가장 먼저 발견된 사람을 추적
      }
    }

    // 상태 전이 논리
    if (target_detected) {
      if (target_distance < 1.0F) { // 사람용 100cm 정지 기준
        current_state_ = RobotState::STOPPED;
      } else {
        current_state_ = RobotState::APPROACHING;
      }
    } else {
      current_state_ = RobotState::SEARCHING;
    }

    // 제어 액션 실행
    if (current_state_ == RobotState::STOPPED) {
      sport_client_.StopMove(req);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "목표물 접근 제한 거리 이내 (거리: %.2fm). 정지합니다.", target_distance);
                           
    } else if (current_state_ == RobotState::APPROACHING) {
      // 화면 중앙(640)과 바운딩 박스 중심 간의 오차를 구하여 Yaw 회전 속도로 변환하는 간단한 P 제어
      float error_x = 640.0F - target_center_x;
      float yaw_cmd = error_x * 0.0015F; 
      
      sport_client_.Move(req, 0.3F, 0.0F, yaw_cmd);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "사람 추적 중... 거리: %.2fm, Yaw 속도 명령: %.3f", target_distance, yaw_cmd);
                           
    } else if (current_state_ == RobotState::SEARCHING) {
      sport_client_.Move(req, 0.0F, 0.0F, 0.4F);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "사람을 탐색하기 위해 회전 중입니다...");
    }
  }

  SportClient sport_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  RobotState current_state_;

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
  unitree_go::msg::SportModeState latest_state_;
  bool state_received_ = false;

  rclcpp::Subscription<unitree_go2_vision_msgs::msg::DetectionArray>::SharedPtr vision_sub_;
  unitree_go2_vision_msgs::msg::DetectionArray latest_detections_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2AIController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
