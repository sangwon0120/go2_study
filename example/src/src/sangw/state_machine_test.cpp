#include <chrono>
#include <memory>

#include "common/ros2_sport_client.h" // SportClient 클래스 선언이 들어있는 헤더파일. 
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp" // 고수준 제어에 필요한 메시지 타입. 

using namespace std::chrono_literals; // 100ms처럼 간단하게 쓰려고 추가. 없으면 더 길게 써야됨. 

enum class RobotState {
    FORWARD1,
    STOP1,
    TURN_LEFT,
    STOP2,
    FORWARD2,
    DONE
};
class Go2Test : public rclcpp::Node { // 노드를 상속 받겠다
    public:
        Go2Test()
        : Node("my_go2_test_node"), sport_client_(this), current_state_(RobotState::FORWARD1) // 초기화. 
        {
            RCLCPP_INFO(this->get_logger(),"Go2 test node started"); // 로그 찍히게 하는 코드 

            state_start_time_ = this->now();

            timer_ = this->create_wall_timer( // 100ms마다 callback함수를 실행하는 타이머. 
                100ms,
                std::bind(&Go2Test::timer_callback, this)
            );
        }
    private:
        void timer_callback() // callback 함수. 
        {
            unitree_api::msg::Request req; // 요청 메시지 생성 
            
            rclcpp::Duration state_elapsed = this->now() - state_start_time_; // 시작 후 지난 시간 계산.

            if(current_state_== RobotState::FORWARD1 && state_elapsed.seconds()>=3.0){
                current_state_= RobotState::STOP1;
                RCLCPP_INFO(this->get_logger(),"FORWARD1->STOP1");
                state_start_time_ = this->now();
            }
            else if(current_state_== RobotState::STOP1 && state_elapsed.seconds()>=1.0){
                current_state_= RobotState::TURN_LEFT;
                RCLCPP_INFO(this->get_logger(),"STOP1->TURN_LEFT");
                state_start_time_ = this->now();
            }
            else if(current_state_== RobotState::TURN_LEFT && state_elapsed.seconds()>=4.3){
                current_state_ = RobotState::STOP2;
                RCLCPP_INFO(this->get_logger(),"TURN_LEFT->STOP2");
                state_start_time_ = this->now();
            }
            else if(current_state_== RobotState::STOP2 && state_elapsed.seconds()>=1.0){
                current_state_= RobotState::FORWARD2;
                RCLCPP_INFO(this->get_logger(),"STOP2->FORWARD2");
                state_start_time_ = this->now();
            }
            else if(current_state_ == RobotState::FORWARD2 && state_elapsed.seconds()>=3.0){
                current_state_ = RobotState::DONE;
                RCLCPP_INFO(this->get_logger(),"FORWARD2->DONE");
                state_start_time_ = this->now();
            }

            if(current_state_ == RobotState::FORWARD1 || current_state_ == RobotState::FORWARD2){
                sport_client_.Move(req,0.3f,0.0f,0.0f);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Moving forward.."
                );
            }
            else if(current_state_== RobotState::STOP1){
                sport_client_.StopMove(req);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "StopMove sent"
                );
            }
            else if(current_state_== RobotState::TURN_LEFT){
                sport_client_.Move(req,0.0f,0.0f,0.8f);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Turning left.."
                );
            }
            else if(current_state_== RobotState::STOP2){
                sport_client_.StopMove(req);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Second StopMove sent"
                );
            }
            else if(current_state_ == RobotState::DONE){
                sport_client_.StopMove(req);
                sport_client_.Damp(req);
                RCLCPP_INFO(this->get_logger(), "Last StopMove sent");
                timer_->cancel();
            }
        }
    private:
        SportClient sport_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time state_start_time_;
        RobotState current_state_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv); // 초기화. 
    auto node = std::make_shared<Go2Test>(); // ros에서 노드만들때 보통 이렇게 만든다. 
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}