#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/go2_front_video_data.hpp"
#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

class CameraTestNode : public rclcpp::Node {
public:
    CameraTestNode() : Node("go2_camera_test_node") {
        RCLCPP_INFO(this->get_logger(), "카메라 테스트(파서 적용) 노드 시작");
        init_decoder();
        video_sub_ = this->create_subscription<unitree_go::msg::Go2FrontVideoData>(
            "/frontvideostream", 10,
            std::bind(&CameraTestNode::video_callback, this, std::placeholders::_1)
        );
    }

    ~CameraTestNode() {
        if (parser_context_) av_parser_close(parser_context_); // 파서 해제 추가
        if (codec_context_) avcodec_free_context(&codec_context_);
        if (frame_) av_frame_free(&frame_);
        if (packet_) av_packet_free(&packet_);
        if (sws_context_) sws_freeContext(sws_context_);
        cv::destroyAllWindows();
    }

private:
    void init_decoder() {
        const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            RCLCPP_ERROR(this->get_logger(), "H.264 디코더를 찾을 수 없습니다!");
            return;
        }

        // [추가된 부분] 바이트 배열을 프레임 단위로 잘라줄 가위(Parser) 생성
        parser_context_ = av_parser_init(codec->id);
        
        codec_context_ = avcodec_alloc_context3(codec);
        avcodec_open2(codec_context_, codec, nullptr);

        packet_ = av_packet_alloc();
        frame_ = av_frame_alloc();
    }

    void video_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg) {
        if (msg->video720p.empty()) return;

        // C언어 포인터 연산을 위해 데이터 시작점과 남은 크기를 변수에 담습니다.
        uint8_t* data = const_cast<uint8_t*>(msg->video720p.data());
        int data_size = msg->video720p.size();

        // 수신된 바이트 배열을 전부 읽을 때까지 반복
        while (data_size > 0) {
            // 1. 파서를 통해 데이터 뭉치에서 의미 있는 패킷 하나를 잘라냅니다.
            int parsed_bytes = av_parser_parse2(
                parser_context_, codec_context_,
                &packet_->data, &packet_->size,
                data, data_size,
                AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0
            );

            if (parsed_bytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "파싱 에러 발생");
                break; // 에러 나면 이번 메시지는 포기
            }

            // 읽은 만큼 포인터를 앞으로 이동시키고 남은 크기를 줄입니다. (C언어 스타일)
            data += parsed_bytes;
            data_size -= parsed_bytes;

            // 2. 파서가 온전한 패킷을 하나 만들어 냈다면 디코더로 보냅니다.
            if (packet_->size > 0) {
                int ret = avcodec_send_packet(codec_context_, packet_);
                if (ret < 0) continue;

                // 3. 디코더에서 프레임 꺼내기
                while (ret >= 0) {
                    ret = avcodec_receive_frame(codec_context_, frame_);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF || ret < 0) {
                        break;
                    }

                    // [추가된 안전장치] 높이와 너비가 정상적으로 잡힌(깨지지 않은) 프레임만 OpenCV로 변환합니다.
                    if (frame_->width > 0 && frame_->height > 0) {
                        cv::Mat bgr_img = convert_frame_to_mat(frame_);
                        if (!bgr_img.empty()) {
                            cv::imshow("Go2 Front Camera", bgr_img);
                            cv::waitKey(1);
                        }
                    }
                }
            }
        }
    }

    cv::Mat convert_frame_to_mat(AVFrame* frame) {
        int width = frame->width;
        int height = frame->height;
        cv::Mat bgr_img(height, width, CV_8UC3);

        if (!sws_context_) {
            sws_context_ = sws_getContext(
                width, height, codec_context_->pix_fmt,
                width, height, AV_PIX_FMT_BGR24,
                SWS_BILINEAR, nullptr, nullptr, nullptr
            );
        }

        uint8_t* dest[4] = { bgr_img.data, nullptr, nullptr, nullptr };
        int dest_linesize[4] = { static_cast<int>(bgr_img.step[0]), 0, 0, 0 };
        sws_scale(sws_context_, frame->data, frame->linesize, 0, height, dest, dest_linesize);

        return bgr_img;
    }

    rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr video_sub_;
    
    // 구조체 포인터들
    AVCodecParserContext* parser_context_ = nullptr; // 새로 추가된 파서
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