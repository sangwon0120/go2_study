#include <chrono>
#include <memory>

#include "common/ros2_sport_client.h" // SportClient 클래스 선언이 들어있는 헤더파일. 
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp" // 고수준 제어에 필요한 메시지 타입. 

using namespace std::chrono_literals; // 100ms처럼 간단하게 쓰려고 추가. 없으면 더 길게 써야됨. 

class MoveForward : public rclcpp::Node { // 노드를 상속 받겠다
    public:
        MoveForward()
        : Node("my_go2_forward_node"), sport_client_(this), stopped_(false) // 초기화. 
        {
            RCLCPP_INFO(this->get_logger(),"Go2 forward node started"); // 로그 찍히게 하는 코드 

            start_time_ = this->now(); // 시작 시간 저장.

            timer_ = this->create_wall_timer( // 100ms마다 callback함수를 실행하는 타이머. 
                100ms,
                std::bind(&MoveForward::timer_callback, this)
            );
        }
    private:
        void timer_callback() // callback 함수. 
        {
            unitree_api::msg::Request req; // 요청 메시지 생성 
            
            rclcpp::Duration elapsed = this->now() - start_time_; // 시작 후 지난 시간 계산.

            //시작 후 3초 동안은 앞으로 이동.
            if(elapsed.seconds() < 3.0){
                sport_client_.Move(req,0.3f,0.0f,0.0f);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Moving forward.."
                );
            }
            //시작 후 3초가 지나면 
            else if(!stopped_){
                sport_client_.StopMove(req);
                stopped_ = true;
                RCLCPP_INFO(this->get_logger(),"StopMove sent");

                timer_->cancel();
            }
        }
    private:
        SportClient sport_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time start_time_;
        bool stopped_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv); // 초기화. 
    auto node = std::make_shared<MoveForward>(); // ros에서 노드만들때 보통 이렇게 만든다. 
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}