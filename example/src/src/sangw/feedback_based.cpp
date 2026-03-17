#include <chrono>
#include <memory>
#include <cmath>

#include "common/ros2_sport_client.h" // SportClient 클래스 선언이 들어있는 헤더파일. 
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp" // 고수준 제어에 필요한 메시지 타입. 
#include "unitree_go/msg/sport_mode_state.hpp"
using namespace std::chrono_literals; // 100ms처럼 간단하게 쓰려고 추가. 없으면 더 길게 써야됨. 

enum class RobotState {
    FORWARD1,
    STOP1,
    TURN_LEFT,
    STOP2,
    FORWARD2,
    DONE,
    EMERGENCY_STOP
};
class Go2Test : public rclcpp::Node { // 노드를 상속 받겠다
    public:
        Go2Test()
        : Node("my_go2_test_node"), sport_client_(this), current_state_(RobotState::FORWARD1) // 초기화. 
        {
            RCLCPP_INFO(this->get_logger(),"Go2 test node started"); // 로그 찍히게 하는 코드 

            state_start_time_ = this->now();
            last_feedback_log_time_ = this->now();

            timer_ = this->create_wall_timer( // 100ms마다 callback함수를 실행하는 타이머. 
                100ms,
                std::bind(&Go2Test::timer_callback, this)
            );

            state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>( //subscriber 생성. 
                "lf/sportmodestate", // 구독할 토픽
                10, // 큐 크기 
                std::bind(&Go2Test::state_callback,this, std::placeholders::_1) // callback 함수 연결 
            );

        }
    private:
        void timer_callback() // callback 함수. 
        {
            auto now = this->now();
            unitree_api::msg::Request req; // 요청 메시지 생성 
            
            rclcpp::Duration state_elapsed = this->now() - state_start_time_; // 시작 후 지난 시간 계산.

            
            if(!state_received_){
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Haven't received SportModeState yet.."
                );
                return;
            }

            
            if((now - last_feedback_log_time_).seconds() >= 0.5){
                RCLCPP_INFO(
                    this->get_logger(),
                    "state = %d | planar_speed = %.3f | min_obstacle =%.3f |vel = [%.3f, %.3f, %.3f] | yaw_speed = %.3f | pos = [%.3f, %.3f, %.3f] | mode = %u | gait = %u",
                    static_cast<int>(current_state_),
                    planar_speed(),
                    min_obstacle_distance(),
                    latest_state_.velocity[0],
                    latest_state_.velocity[1],
                    latest_state_.velocity[2],
                    latest_state_.yaw_speed,
                    latest_state_.position[0],
                    latest_state_.position[1],
                    latest_state_.position[2],
                    latest_state_.mode,
                    latest_state_.gait_type
                );
                last_feedback_log_time_ = now;
            }

            /************************************************************
            *************************************************************
            *************************************************************
            상태 전이
            *************************************************************
            *************************************************************
            *************************************************************/
            if((current_state_ == RobotState::FORWARD1 || current_state_ == RobotState::FORWARD2)&&
                    obstacle_too_close()){
                        current_state_ = RobotState::EMERGENCY_STOP;
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Obstacle too close! EMERGENCY_STOP triggered.  min dist = %.3f",
                            min_obstacle_distance()
                        );
                        state_start_time_=this->now();
                    }
            if(current_state_== RobotState::FORWARD1 && 
                    forward1_distance_from_start() >= 1.0f &&
                    forward1_initialized_){
                current_state_= RobotState::STOP1;
                RCLCPP_INFO(this->get_logger(),"FORWARD1->STOP1");
                state_start_time_ = this->now();
            }

            //FORWARD1 timeout
            else if (current_state_ == RobotState::FORWARD1 &&
                    state_elapsed.seconds() >= 6.0) {
                current_state_ = RobotState::STOP1;
                RCLCPP_WARN(
                    this->get_logger(),
                    "FORWARD1->STOP1 (timeout fallback, distance=%.3f m)",
                    forward1_distance_from_start()
                    );
                state_start_time_ = this->now();
            }

            else if(current_state_== RobotState::STOP1 
                    && state_elapsed.seconds()>=0.5
                    && is_robot_stopped()){
                current_state_= RobotState::TURN_LEFT;
                turn_initialized_ = false;
                RCLCPP_INFO(this->get_logger(),"STOP1->TURN_LEFT");
                state_start_time_ = this->now();
            }

            else if(current_state_== RobotState::TURN_LEFT && // 로봇의 상태 기반으로 다음 상태 전이. 
                    turn_initialized_&&
                    yaw_delta_from_start() >= 2.6f){ // 180도 이상 돌았고, 시작 yaw값이 저장되어있야 한다는 조건. 
                current_state_ = RobotState::STOP2;
                RCLCPP_INFO(
                    this->get_logger(),
                    "TURN_LEFT->STOP2 (target yaw reached, delta = %.3f rad)",
                    yaw_delta_from_start()
                );
                state_start_time_ = this->now();
            }

            else if(current_state_ == RobotState::TURN_LEFT && // 센서의 이상처럼 예상치 못한 상황 때문에 TURN_LEFT가 영원히 안끝날 수도 있기 때문에 timeout을 같이 둔다
                    state_elapsed.seconds()>=6.0){
                current_state_ = RobotState::STOP2;
                RCLCPP_WARN(
                    this->get_logger(),
                    "TURN LEFT -> STOP2 (timeout fallback, delta = %.3f rad)",
                    yaw_delta_from_start()
                );
                state_start_time_= this->now();
            }

            else if(current_state_== RobotState::STOP2 
                    && state_elapsed.seconds()>=0.5
                    && is_robot_stopped()){
                current_state_= RobotState::FORWARD2;
                forward2_initialized_ = false;
                RCLCPP_INFO(this->get_logger(),"STOP2->FORWARD2");
                state_start_time_ = this->now();
            }

            else if(current_state_ == RobotState::FORWARD2 
                    && forward2_distance_from_start() >= 1.0f 
                    && forward2_initialized_){
                current_state_ = RobotState::DONE;
                RCLCPP_INFO(this->get_logger(),
                    "FORWARD2->DONE (target distance reached: %.3f m)",
                    forward2_distance_from_start()
                    );
                state_start_time_ = this->now();
            }

            //FORWARD2 timeout
            else if (current_state_ == RobotState::FORWARD2 &&
                    state_elapsed.seconds() >= 6.0) {
                current_state_ = RobotState::DONE;
                RCLCPP_WARN(
                    this->get_logger(),
                    "FORWARD2->DONE (timeout fallback, distance=%.3f m)",
                    forward2_distance_from_start()
                    );
                state_start_time_ = this->now();
            }

            if(current_state_ == RobotState::TURN_LEFT && turn_initialized_){
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "TURN_LEFT | current_yaw=%.3f | start_yaw=%.3f | yaw_delta=%.3f | yaw_speed=%.3f",
                    current_yaw(),
                    turn_start_yaw_,
                    yaw_delta_from_start(),
                    latest_state_.yaw_speed
                );
            }

            if(current_state_ == RobotState::TURN_LEFT && !turn_initialized_){ // TURN_LEFT 상태로 들어간 첫 타이머 틱에서 한번만 실행된다. 
                turn_start_yaw_ = current_yaw();
                turn_initialized_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "TURN_LEFT start yaw initialized : %.3f rad",
                    turn_start_yaw_
                    );
            }

            //FORWARD1에 진입했을때 시작 위치를 초기화
            if(current_state_==RobotState::FORWARD1 && !forward1_initialized_){
                forward1_start_x_ = current_x();
                forward1_start_y_ = current_y();
                forward1_initialized_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "FORWARD1 start position initialized: x=%.3f, y=%.3f",
                    forward1_start_x_,
                    forward1_start_y_
                );
            }
            
            //전진 중 실제 이동거리 확인하는 log
            if(current_state_ == RobotState::FORWARD1 && forward1_initialized_){
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "FORWARD1 | x=%.3f | y=%.3f | distance=%.3f",
                    current_x(),
                    current_y(),
                    forward1_distance_from_start()
                );
            }
            
            //FORWARD2에 진입했을때 시작 위치를 초기화
            if(current_state_==RobotState::FORWARD2 && !forward2_initialized_){
                forward2_start_x_ = current_x();
                forward2_start_y_ = current_y();
                forward2_initialized_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "FORWARD2 start position initialized: x=%.3f, y=%.3f",
                    forward2_start_x_,
                    forward2_start_y_
                );
            }
            
            //전진 중 실제 이동거리 확인하는 log
            if(current_state_ == RobotState::FORWARD2 && forward2_initialized_){
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "FORWARD2 | x=%.3f | y=%.3f | distance=%.3f",
                    current_x(),
                    current_y(),
                    forward2_distance_from_start()
                );
            }


            /************************************************************
            *************************************************************
            *************************************************************
            Action
            *************************************************************
            *************************************************************
            *************************************************************/
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
                sport_client_.Move(req,0.0f,0.0f,0.5f);
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
                RCLCPP_INFO(this->get_logger(), "Last StopMove sent");
                timer_->cancel();
            }
            else if(current_state_ == RobotState::EMERGENCY_STOP){
                sport_client_.StopMove(req);
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "Emergency stop activate"
                );
            }
        }

        void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg){
            latest_state_ = *msg;
            state_received_= true;
        }

        float planar_speed() const {
            float vx = latest_state_.velocity[0];
            float vy = latest_state_.velocity[1];
            return std::sqrt(vx * vx + vy * vy);
        }

        bool is_robot_stopped() const{
            float speed = planar_speed();
            float yaw = std::fabs(latest_state_.yaw_speed);

            return (speed < 0.05f) && (yaw < 0.05f);
        }

        float current_yaw() const{
            return latest_state_.imu_state.rpy[2];
        } 

        float normalize_angle(float angle)const{ // 각도를 -pi ~ pi 범위로 집어넣는 함수. 
            while(angle > M_PI){
                angle -= 2.0f * M_PI;
            }
            while(angle < -M_PI){
                angle += 2.0f * M_PI;
            }
            return angle;
        }

        float yaw_delta_from_start() const{ // TURN_LEFT이후 얼마나 돌았는지 알려주는 함수. 
            return normalize_angle(current_yaw() - turn_start_yaw_);
        }

        float current_x() const {return latest_state_.position[0];}
        float current_y() const {return latest_state_.position[1];}
        float forward2_distance_from_start() const{
            float dx = current_x() - forward2_start_x_;
            float dy = current_y() - forward2_start_y_;
            return std::sqrt(dx*dx + dy*dy);
        }
        float forward1_distance_from_start() const{
            float dx = current_x() - forward1_start_x_;
            float dy = current_y() - forward1_start_y_;
            return std::sqrt(dx*dx + dy*dy);
        }

        //장애물 거리 중 최소 구하는 함수.
        float min_obstacle_distance() const{
            float min_dist = latest_state_.range_obstacle[0];
            for(int i=0;i<4;i++){
                if(latest_state_.range_obstacle[i] < min_dist){
                    min_dist = latest_state_.range_obstacle[i];
                }
            }
            return min_dist;
        }
        //위험 판정 함수
        bool obstacle_too_close() const{
            return min_obstacle_distance() < 0.0005f;
        }
    private:
        SportClient sport_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time state_start_time_;

        RobotState current_state_;

        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
        unitree_go::msg::SportModeState latest_state_;
        bool state_received_ = false;

        rclcpp::Time last_feedback_log_time_;

        float turn_start_yaw_ = 0.0f; // TURN_LEFT에 들어갈때의 시작 yaw
        bool turn_initialized_ = false; // 시작 yaw를 이미 저장했는지 여부

        float forward2_start_x_ = 0.0f;
        float forward2_start_y_ = 0.0f;
        bool forward2_initialized_ = false;

        float forward1_start_x_ = 0.0f;
        float forward1_start_y_ = 0.0f;
        bool forward1_initialized_ = false;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv); // 초기화. 
    auto node = std::make_shared<Go2Test>(); // ros에서 노드만들때 보통 이렇게 만든다. 
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}