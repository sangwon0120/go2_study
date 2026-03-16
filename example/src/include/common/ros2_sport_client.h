/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef _ROS2_SPORT_CLIENT_
#define _ROS2_SPORT_CLIENT_
#include <iostream> // C++ 출력용. printf에 대응 
#include <rclcpp/rclcpp.hpp>

#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp" // 유니트리 API 요청 ROS2 메시지 타입 
#include "unitree_api/msg/response.hpp"// 유니트리 API 응답 ROS2 메시지 타입 

// API ID 상수들 
// 어떤 동작을 시킬지를 나타내는 명령 번호표
// req.header.identity.api_id에 이 값을 넣으면 로봇이 명령을 구분함. 
const int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
const int32_t ROBOT_SPORT_API_ID_EULER = 1007;
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
const int32_t ROBOT_SPORT_API_ID_SIT = 1009;
const int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
const int32_t ROBOT_SPORT_API_ID_HELLO = 1016;
const int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;
const int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;
const int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;
const int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;
const int32_t ROBOT_SPORT_API_ID_POSE = 1028;
const int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;
const int32_t ROBOT_SPORT_API_ID_HEART = 1036;
const int32_t ROBOT_SPORT_API_ID_STATICWALK = 1061;
const int32_t ROBOT_SPORT_API_ID_TROTRUN = 1062;
const int32_t ROBOT_SPORT_API_ID_ECONOMICGAIT = 1063;
const int32_t ROBOT_SPORT_API_ID_LEFTFLIP = 2041;
const int32_t ROBOT_SPORT_API_ID_BACKFLIP = 2043;
const int32_t ROBOT_SPORT_API_ID_HANDSTAND = 2044;
const int32_t ROBOT_SPORT_API_ID_FREEWALK = 2045;
const int32_t ROBOT_SPORT_API_ID_FREEBOUND = 2046;
const int32_t ROBOT_SPORT_API_ID_FREEJUMP = 2047;
const int32_t ROBOT_SPORT_API_ID_FREEAVOID = 2048;
const int32_t ROBOT_SPORT_API_ID_CLASSICWALK = 2049;
const int32_t ROBOT_SPORT_API_ID_WALKUPRIGHT = 2050;
const int32_t ROBOT_SPORT_API_ID_CROSSSTEP = 2051;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_SET = 2054;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_GET = 2055;
const int32_t ROBOT_SPORT_API_ID_SWITCHAVOIDMODE = 2058;

#pragma pack(1) // #pragma pack(1)은 구조체 멤버 사이에 들어가는 패딩 바이트를 없애서 1바이트 단위로 딱 붙게 만드는 옵션 
struct PathPoint {
  float timeFromStart;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack()

/* 
go2_sport_client.cpp에서 사용되는 SportClient. 
SportClient는 ROS2 토픽으로 request를 publish하고 response를 subscribe해서 결과를 받는 도우미
go2_sport_client.cpp에서 req_만 채워서 보내던 이유가 여기 req_puber_가 publish를 해주기 때문. 
req_suber_는 토픽을 구독하는 subscription. 

*/
class SportClient { 
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;
  rclcpp::Node *node_; // ROS2에서는 publisher,subscription 만들려면 Node가 필요하니까 생성자에서 받은 노드 포인터를 저장. 

 public: // SportClient를 만들면 request 토픽 publisher가 준비된다는 뜻. 
  explicit SportClient(rclcpp::Node *node) : node_(node) {
    req_puber_ = node_->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);
  }
  /******************************************
  핵심 : Call() 템플릿 함수
  이 함수는 요청을 publish하고 같은 api_id의 응답이 올 때까지 기다린 다음,
  JSON으로 파싱해서 돌려준다가 목적.
  *******************************************/
  template <typename Request, typename Response>
  nlohmann::json Call(const Request &req) {
    // promise랑 future은 나중에 값이 들어올 걸 기다리는 장치 
    std::promise<typename Response::SharedPtr> response_promise; // reponse_promise : 응답오면 여기에 넣을게
    auto response_future = response_promise.get_future(); // 응답 올 때 까지 멈춰서 기다릴게
    auto api_id = req.header.identity.api_id;
    // 응답 토픽 구독 생성
    auto req_suber_ = node_->create_subscription<Response>(
        "/api/sport/response", 1, // 여기서 api/sport/response를 구독함. 
        //콜백 람다 함수 
        [&response_promise, api_id](const typename Response::SharedPtr data) {
          if (data->header.identity.api_id == api_id) {
            // 응답 data가 들어오면 그 응답의 api_id가 내가 보낸 api_id와 같을 때만 promise에 값을 넣어 future을 깨움 
            response_promise.set_value(data); 
            // 주의 ! : 이 방식은 콜백이 실제로 실행되어야 future가 풀림.
            // 노드가 spin 되고 있어야함.(executor가 돌고 있어야 함.)
            // 그리고 응답이 안오면 response_future.get()에서 영원히 멈출 수 있음
          }
        });

    req_puber_->publish(req); //api/sport/request로 요청 발행. 

    auto response = *response_future.get(); // 여기서 응답 올 때까지 멈춤. 
    // 응답 메시지 안의 response.data는 문자열/바이트열 형태고 그걸 parse()로 json객체로 바꿈 
    nlohmann::json js = nlohmann::json::parse(response.data.data()); 
    req_suber_.reset(); // 요청-응답이 되면 더 이상 응답 토픽을 듣지 않겠다는 뜻. 
    return js;
  }

  /*
   * @brief Damp
   * @api: 1001
   */
  void Damp(unitree_api::msg::Request &req); // go2_sport_client.cpp에서 쓸 함수 

  /*
   * @brief BalanceStand
   * @api: 1002
   */
  void BalanceStand(unitree_api::msg::Request &req);

  /*
   * @brief StopMove
   * @api: 1003
   */
  void StopMove(unitree_api::msg::Request &req);

  /*
   * @brief StandUp
   * @api: 1004
   */
  void StandUp(unitree_api::msg::Request &req);

  /*
   * @brief StandDown
   * @api: 1005
   */
  void StandDown(unitree_api::msg::Request &req);

  /*
   * @brief RecoveryStand
   * @api: 1006
   */
  void RecoveryStand(unitree_api::msg::Request &req);

  /*
   * @brief Euler
   * @api: 1007
   */
  void Euler(unitree_api::msg::Request &req, float roll, float pitch,
             float yaw);

  /*
   * @brief Move
   * @api: 1008
   */
  void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw);

  /*
   * @brief Sit
   * @api: 1009
   */
  void Sit(unitree_api::msg::Request &req);

  /*
   * @brief RiseSit
   * @api: 1010
   */
  void RiseSit(unitree_api::msg::Request &req);

  /*
   * @brief SpeedLevel
   * @api: 1015
   */
  void SpeedLevel(unitree_api::msg::Request &req, int level);

  /*
   * @brief Hello
   * @api: 1016
   */
  void Hello(unitree_api::msg::Request &req);

  /*
   * @brief Stretch
   * @api: 1017
   */
  void Stretch(unitree_api::msg::Request &req);

  /*
   * @brief SwitchJoystick
   * @api: 1027
   */
  void SwitchJoystick(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief Content
   * @api: 1020
   */
  void Content(unitree_api::msg::Request &req);

  /*
   * @brief Pose
   * @api: 1028
   */
  void Pose(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief Scrape
   * @api: 1029
   */
  void Scrape(unitree_api::msg::Request &req);

  /*
   * @brief FrontFlip
   * @api: 1030
   */
  void FrontFlip(unitree_api::msg::Request &req);

  /*
   * @brief FrontJump
   * @api: 1031
   */
  void FrontJump(unitree_api::msg::Request &req);

  /*
   * @brief FrontPounce
   * @api: 1032
   */
  void FrontPounce(unitree_api::msg::Request &req);

  /*
   * @brief Dance1
   * @api: 1022
   */
  void Dance1(unitree_api::msg::Request &req);

  /*
   * @brief Dance2
   * @api: 1023
   */
  void Dance2(unitree_api::msg::Request &req);

  /*
   * @brief Heart
   * @api: 1036
   */
  void Heart(unitree_api::msg::Request &req);

  /*
   * @brief StaticWalk
   * @api: 1061
   */
  void StaticWalk(unitree_api::msg::Request &req);

  /*
   * @brief TrotRun
   * @api: 1062
   */
  void TrotRun(unitree_api::msg::Request &req);

  /*
   * @brief EconomicGait
   * @api: 1063
   */
  void EconomicGait(unitree_api::msg::Request &req);

  /*
   * @brief LeftFlip
   * @api: 2041
   */
  void LeftFlip(unitree_api::msg::Request &req);

  /*
   * @brief BackFlip
   * @api: 2043
   */
  void BackFlip(unitree_api::msg::Request &req);

  /*
   * @brief Handstand
   * @api: 2044
   */
  void HandStand(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeWalk
   * @api: 2045
   */
  void FreeWalk(unitree_api::msg::Request &req);

  /*
   * @brief FreeBound
   * @api: 2046
   */
  void FreeBound(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeJump
   * @api: 2047
   */
  void FreeJump(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeAvoid
   * @api: 2048
   */
  void FreeAvoid(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief ClassicWalk
   * @api: 2049
   */
  void ClassicWalk(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief WalkUpright
   * @api: 2050
   */
  void WalkUpright(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief CrossStep
   * @api: 2051
   */
  void CrossStep(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief AutoRecoverySet
   * @api: 2054
   * @param flag: true to enable, false to disable
   */
  void AutoRecoverySet(unitree_api::msg::Request &req, bool flag);
  /*
   * @brief AutoRecoveryGet
   * @api: 2055
   * NOTICE!!!: This function cannot be used in ros2 callback.
   *
   */
  void AutoRecoveryGet(unitree_api::msg::Request &req, bool &flag);
  /*
   * @brief SwitchAvoidMode
   * @api: 2058
   * @param flag: true to enable, false to disable
   */
  void SwitchAvoidMode(unitree_api::msg::Request &req);
};

#endif