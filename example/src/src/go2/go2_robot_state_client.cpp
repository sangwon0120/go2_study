/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 NOTICE: This client is only available on ROS2 Humble.
***********************************************************************/
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp>

#include "common/ros2_robot_state_client.h"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

class Go2RobotStateClientNode : public rclcpp::Node {
 public:
  //explicit : 원치 않는 자동 형변환으로 생성자 호출되는 걸 막아라.
  explicit Go2RobotStateClientNode()
      : Node("go2_robot_state_client_node"), robot_state_client_(this) {
    t1_ = std::thread([this] { // 별도 쓰레드 생성. 
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      RobotControl(); // 0.5초 기다렸다가 RobotControl()호출 
    }); // ROS2에서 spin은 메인 스레드에서 돌고 로봇 제어/서비스 호출은 t1_ 스레드에서 실행됨. 
  }

  void RobotControl() {
    int32_t status = 0;
    int32_t ret = robot_state_client_.SetReportFreq(3, 30);// 반환값 ret은 보통 0이면 성공 음서면 실패
    std::cout << "Call SetReportFreq[3,30] ret:" << ret << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 5초 대기 
    const auto *serviceName = "sport_mode"; 
    ret = robot_state_client_.ServiceSwitch(serviceName, 0, status); // 서비스를 0으로 스위치. 0은 off/disable 의미.
    std::cout << "Call ServiceSwitch[" << serviceName << ",0] ret:" << ret
              << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    ret = robot_state_client_.ServiceSwitch(serviceName, 1, status); // 이번엔 1로 스위치. on/enable
    std::cout << "Call ServiceSwitch[" << serviceName << ",1] ret:" << ret
              << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::vector<ServiceState> serviceStateList; // ServiceState 구조체들의 리스트를 담을 벡터
    ret = robot_state_client_.ServiceList(serviceStateList); // 로봇이 가진 서비스 목록을 받아옴.
    std::cout << "Call ServiceList ret:" << ret << std::endl;

    size_t i = 0; //size_t는 배열 인덱스/크기용 타입. 
    size_t count = serviceStateList.size(); // count에 벡터 크기 저장 
    std::cout << "serviceStateList size:" << count << std::endl;

    for (i = 0; i < count; i++) { // 서비스 목록 반복 
      const ServiceState &serviceState = serviceStateList[i];// i번째 원소를 serviceState라는 참조로 잡음. 복사를 피하려고 참조로 받는 것. const라서 읽기 전용
      std::cout << "name:" << serviceState.name
                << ", status:" << serviceState.status
                << ", protect:" << serviceState.protect << std::endl;
    }
  }

 private:
  RobotStateClient robot_state_client_; // 핵심 개체. 실제 ROS2 토픽/서비스로 Unitree API를 호출하는 래퍼. 
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_; // Response 메시지를 구독하기 위한 subscription 포인터
  unitree_api::msg::Request req_; // 요청 메시지 객체
  std::thread t1_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2RobotStateClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}