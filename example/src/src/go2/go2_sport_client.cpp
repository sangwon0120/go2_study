/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono> // 시간관련 유틸 std::chrono::milliseconds(500)같은 시간 단위를 쓰기 위해 필요
#include <cmath> // 수학 함수 
#include <memory> // 스마트 포인터 쓰기 위해 필요 
#include <rclcpp/executors/multi_threaded_executor.hpp> //ros2 실행기(executor) 중 멀티스레드 실행기를 쓰기 위해 필요 
#include <rclcpp/rclcpp.hpp>//ros2 C++ 핵심
#include <std_msgs/msg/string.hpp>// 표준 string타입 
#include <thread> // std::thread 사용을 위한 헤더 
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp> // SportModeState 메시지의 내부 구조체 정의 헤더 


/***************************************
!핵심! 
SportClient라는 클래스를 제공하는 헤더. 
이 클래스가 Unitree API Request 메시지를 "걷기,일어서기" 같은 명령으로 채워주는 역할을 함
****************************************/
#include "common/ros2_sport_client.h" 

// Go2의 고수준 상태 토픽 메시지 타입. 아래 subscription이 이 타입을 구독함 
#include "unitree_go/msg/sport_mode_state.hpp"

//Go2의 SportMode 상태가 발행되는 토픽 이름 문자열. lf/sportmodestate를 구독함/ 
#define TOPIC_HIGHSTATE "lf/sportmodestate"


// test_mode_ 값으로 어떤 동작을 시킬지 고르는 메뉴
enum TestMode {
  /*---Basic motion---*/
  NORMAL_STAND,
  BALANCE_STAND,
  VELOCITY_MOVE,
  STAND_DOWN,
  STAND_UP,
  DAMP,
  RECOVERY_STAND,
  /*---Special motion ---*/
  SIT,
  RISE_SIT,
  MOVE,
  STOP_MOVE,
  
};

/* Ros2 노드 정의. 이 노드가 Go2 상태 토픽을 구독하고 SportClient로 Request(명령)을 만들고
로봇에 제어 명령을 보내는 구조 */
class Go2SportClientNode : public rclcpp::Node {
 public:
  explicit Go2SportClientNode(int test_mode) //explicit은 int하나 받는 생성자에서 암시적 변환 방지용 
      : Node("go2_sport_client_node"), // 노드 이름 설정 
        sport_client_(this), // SportClient가 내부에서 Ros2 node handle을 쓰도록 this를 넘김 
        test_mode_(test_mode) { // main에서 받는 모드를 저장 

    
    // 상태 토픽 구독 생성 
    // subscriber 포인터. 구독 메시지 타입은 SportModeState
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1, // lf/sportmodestate를 구독하고 queue depth는 1(최신 것만 보겠다는 뜻)
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          HighStateHandler(data); // 람다 함수로 콜백 함수 구현. 토픽이 올 때 마다 HighStateHandler가 실행됨. 
        });

    // 별도 스레드 생성 
    t1_ = std::thread([this] {
      // wait for ros2 spin
      // spin이 돌기 시작할 때 까지 0.5초 기다린뒤 RobotControl()을 딱 한 번 호출함. 
      std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
      RobotControl(); // 보통은 타이머로 주기적으로 publish하는데 example이라 명령 한번 보내고 끝인 것 같음 . 
    });
  }

  /*******************************************
  중요한 점!
  여기서는 sport_client_XXX(req_)만 호출하고 있음 -> req_메시지를 채우기만 했고 publish가 안보임
  즉, SportClient 내부가 생성자에서 publisher를 만들고 StandUp(req_)같은 함수가 내부에서 publish까지 해주는 구조임
  -> SportClient가 publish까지 책임지는 래퍼 구조
  ********************************************/
  void RobotControl() {
    ct_ += dt_; // 누적 시간 처럼 쓰려는 변수. 타이머에 사용하려고 만들었나?
    switch (test_mode_) { // 실행 모드에 따라 다른 행동 수행
      case NORMAL_STAND:
        sport_client_.StandUp(req_); // 명령을 req_ 에 채움. 
        break;
      case BALANCE_STAND:
        sport_client_.BalanceStand(req_);
        break;
      case VELOCITY_MOVE:
        sport_client_.Move(req_, 0.3, 0, 0.3); // Move(vx,vy,vyaw). 예시는 앞으로 가며 회전 
        break;
      case STAND_DOWN:
        sport_client_.StandDown(req_);
        break;
      case STAND_UP:
        sport_client_.StandUp(req_); 
        break;
      case DAMP:
        sport_client_.Damp(req_); // 힘 빼는 모드 
        break;
      case RECOVERY_STAND:
        sport_client_.RecoveryStand(req_); // 넘어졌을 때 복구해서 서는 동작 
        break;
      case SIT:
        if (flag_ == 0) {
          sport_client_.Sit(req_); // flag_가 0일때만 1번 실행되는 구조.
          flag_ = 1;
        }
        break;
      case RISE_SIT:
        if (flag_ == 0) {
          sport_client_.RiseSit(req_); // 마찬가지로 1번만 실행되도록 설계 
          flag_ = 1;
        }
        break;
      case MOVE:
        sport_client_.Move(req_, 0.3, 0, 0); 
        break;
      case STOP_MOVE:
        sport_client_.StopMove(req_);
        break;
      default:
        sport_client_.StopMove(req_); // 잘못된 모드면 기본으로 StopMove
    }
  }
  // 초기 상태 저장 함수 
  void GetInitState() { // 현재 받은 state_를 기준으로 초기 위치/yaw를 저장 
    px0_ = state_.position[0];
    py0_ = state_.position[1];
    yaw0_ = state_.imu_state.rpy[2];
    RCLCPP_INFO(this->get_logger(),
                "initial position: x0: %f, y0: %f, yaw0: %f", px0_, py0_,
                yaw0_);
  }

  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
    state_ = *msg; // 들어온 메시지를 state_에 복사 
    RCLCPP_INFO(this->get_logger(), "Position: %f, %f, %f", state_.position[0],
                state_.position[1], state_.position[2]);
    RCLCPP_INFO(this->get_logger(), "IMU rpy: %f, %f, %f",
                state_.imu_state.rpy[0], state_.imu_state.rpy[1],
                state_.imu_state.rpy[2]);
  }

 //private 멤버 변수들
 private:
  unitree_go::msg::SportModeState state_; // state_ : 최신 상태 저장소 
  SportClient sport_client_; // 제어 명령 생성/전송 담당 객체 
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_; // 상태 토픽 구독자
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_; // response 구독자

  rclcpp::TimerBase::SharedPtr timer_; // 타이머 포인터. 주기 제어 하려고 만든 듯. 
  unitree_api::msg::Request req_;  // Unitree Go2 ROS2 request message. 로봇에게 보낼 요청 메시지 저장소. SportClient 함수들이 이걸 채움. 
  double px0_{}, py0_{}, yaw0_{}; // 초기 위치/자세 저장 
  double ct_{};// 누적 시간 
  int flag_{}; // 1회 실행용 플래그 
  float dt_ = 0.1;// 시간 간격 의도 값 
  int test_mode_; // 선택한 테스트 모드 
  std::thread t1_; // RobotControl을 호출하기 위해 만든 쓰레드 
};

//main 함수 
int main(int argc, char **argv) {
  if (argc < 2) { // 인자가 없으면 사용법 출력 후 종료 
    std::cerr << "Usage: " << argv[0] << " <test_mode>" << std::endl;
    std::cerr << "Available test modes:" << std::endl;
    std::cerr << "  0: NORMAL_STAND" << std::endl;
    std::cerr << "  1: BALANCE_STAND" << std::endl;
    std::cerr << "  2: VELOCITY_MOVE" << std::endl;
    std::cerr << "  3: STAND_DOWN" << std::endl;
    std::cerr << "  4: STAND_UP" << std::endl;
    std::cerr << "  5: DAMP" << std::endl;
    std::cerr << "  6: RECOVERY_STAND" << std::endl;
    std::cerr << "  7: SIT" << std::endl;
    std::cerr << "  8: RISE_SIT" << std::endl;
    std::cerr << "  9: MOVE" << std::endl;
    std::cerr << "  10: STOP_MOVE" << std::endl;
    return 1;
  }

  int test_mode = std::atoi(argv[1]); // NOLINT // argv[1] 문자열을 int로 변환해서 test_mode에 저장 

  rclcpp::init(argc, argv); // ros2 초기화 
  auto node = std::make_shared<Go2SportClientNode>(test_mode); // 노드 생성하면서 test_mode 전달. 
  node->GetInitState(); // 초기 상태 저장 시도. 

  rclcpp::executors::MultiThreadedExecutor executor; // 멀티스레드 executor로 노드를 돌림.
  executor.add_node(node); // 멀티스레드 executor로 노드를 돌림. 
  executor.spin(); // subscription 콜백이 여기서 실행됨. 
  rclcpp::shutdown(); // 종료. 
  return 0;
}