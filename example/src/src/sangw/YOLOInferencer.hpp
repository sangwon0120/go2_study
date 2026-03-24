#pragma once

#include "AI_Inferencer.hpp"
#include <opencv2/dnn.hpp>

/**
 * @brief YOLO Implementation using OpenCV DNN Module
 */
class YOLOInferencer : public AI_Inferencer {
public:
    YOLOInferencer(const std::string& model_path, const std::string& config_path, const std::string& classes_path);
    ~YOLOInferencer() override = default;

    std::vector<DetectionResult> infer(const cv::Mat& frame) override;

private:
    cv::dnn::Net net_;
    std::vector<std::string> classes_;
    float conf_threshold_ = 0.5F;
    float nms_threshold_ = 0.4F;

    void load_classes(const std::string& classes_path);
    std::vector<cv::String> get_outputs_names(const cv::dnn::Net& net);
};
