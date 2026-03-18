#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/go2_front_video_data.hpp"
#include <opencv2/opencv.hpp> // 화면 출력을 위한 OpenCV

// C++ 프로그램에서 순수 C언어로 작성된 헤더를 불러올 때는 extern "C" 로 감싸주어야 합니다.
// "이 부분은 C++의 이름 변환(Name mangling)을 하지 말고 C언어 규칙 그대로 링크해라"는 뜻입니다.
extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

class CameraTestNode : public rclcpp::Node {
public:
    CameraTestNode() : Node("go2_camera_test_node") {
        RCLCPP_INFO(this->get_logger(), "카메라 테스트(디버깅 뷰어) 노드 시작");

        // 1. FFmpeg 디코더 초기화 함수 호출
        init_decoder();

        // 2. 영상 구독기 생성
        video_sub_ = this->create_subscription<unitree_go::msg::Go2FrontVideoData>(
            "/frontvideostream", 10,
            std::bind(&CameraTestNode::video_callback, this, std::placeholders::_1)
        );
    }

    // 소멸자: 프로그램이 끝날 때 동적 할당된 C 구조체 메모리를 해제합니다 (C언어의 free 역할)
    ~CameraTestNode() {
        if (codec_context_) avcodec_free_context(&codec_context_);
        if (frame_) av_frame_free(&frame_);
        if (packet_) av_packet_free(&packet_);
        if (sws_context_) sws_freeContext(sws_context_);
        cv::destroyAllWindows();
    }

private:
    // FFmpeg H.264 디코더 세팅 (C언어 포인터 다루기)
    void init_decoder() {
        // H.264 디코더를 찾습니다.
        const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            RCLCPP_ERROR(this->get_logger(), "H.264 디코더를 찾을 수 없습니다!");
            return;
        }

        // 디코더의 상태를 저장할 구조체 할당 (malloc 역할)
        codec_context_ = avcodec_alloc_context3(codec);
        avcodec_open2(codec_context_, codec, nullptr);

        // 압축된 데이터를 담을 바구니(packet)와 압축 풀린 이미지를 담을 바구니(frame) 할당
        packet_ = av_packet_alloc();
        frame_ = av_frame_alloc();
    }

    void video_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg) {
        if (msg->video720p.empty()) return;

        // 1. ROS 메시지로 들어온 byte 배열의 포인터와 크기를 FFmpeg 패킷에 연결
        // C++의 std::vector 내부 데이터 포인터를 잠시 C 포인터로 캐스팅해서 넘겨줍니다.
        packet_->data = const_cast<uint8_t*>(msg->video720p.data());
        packet_->size = msg->video720p.size();

        // 2. 압축된 패킷을 디코더로 전송
        int ret = avcodec_send_packet(codec_context_, packet_);
        if (ret < 0) return; // 에러나면 무시하고 다음 패킷 대기

        // 3. 디코더에서 압축이 풀린 프레임을 꺼내옵니다.
        // (하나의 패킷에서 여러 프레임이 나올 수도 있어서 while문 사용)
        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_context_, frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break; // 더 이상 꺼낼 프레임이 없음
            } else if (ret < 0) {
                break; // 에러 발생
            }

            // 4. FFmpeg 프레임(YUV 포맷)을 OpenCV 이미지(BGR 포맷)로 변환
            cv::Mat bgr_img = convert_frame_to_mat(frame_);
            
            if (!bgr_img.empty()) {
                // 화면에 출력!
                cv::imshow("Go2 Front Camera", bgr_img);
                // 1ms 동안 키보드 입력을 대기하며 UI를 갱신해줍니다 (OpenCV 필수 코드)
                cv::waitKey(1); 
            }
        }
    }

    // 색상 변환 함수 (YUV -> BGR)
    cv::Mat convert_frame_to_mat(AVFrame* frame) {
        int width = frame->width;
        int height = frame->height;

        // BGR 색상을 담을 OpenCV 빈 이미지 공간 생성
        cv::Mat bgr_img(height, width, CV_8UC3);

        // 색상 변환기(sws_context)가 없으면 초기화
        if (!sws_context_) {
            sws_context_ = sws_getContext(
                width, height, codec_context_->pix_fmt, // 원본 포맷
                width, height, AV_PIX_FMT_BGR24,        // 목표 포맷 (OpenCV BGR)
                SWS_BILINEAR, nullptr, nullptr, nullptr
            );
        }

        // 변환 실행 (frame의 데이터를 bgr_img의 데이터 공간으로 복사하며 변환)
        uint8_t* dest[4] = { bgr_img.data, nullptr, nullptr, nullptr };
        int dest_linesize[4] = { static_cast<int>(bgr_img.step[0]), 0, 0, 0 };
        sws_scale(sws_context_, frame->data, frame->linesize, 0, height, dest, dest_linesize);

        return bgr_img;
    }

    rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr video_sub_;
    
    // FFmpeg에서 사용하는 C 구조체 포인터들
    AVCodecContext* codec_context_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* frame_ = nullptr;
    SwsContext* sws_context_ = nullptr;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}