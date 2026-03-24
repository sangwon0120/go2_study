#include "YOLOInferencer.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>

YOLOInferencer::YOLOInferencer(const std::string& model_path, const std::string& config_path, const std::string& classes_path) {
    load_classes(classes_path);
    
    // Load Darknet model or ONNX depending on provided paths.
    if (!config_path.empty() && config_path.find(".cfg") != std::string::npos) {
        net_ = cv::dnn::readNetFromDarknet(config_path, model_path);
    } else {
        net_ = cv::dnn::readNetFromONNX(model_path);
    }
    
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU); // Switch to CUDA or OpenVINO if available on target
}

void YOLOInferencer::load_classes(const std::string& classes_path) {
    std::ifstream ifs(classes_path);
    std::string line;
    while (std::getline(ifs, line)) {
        classes_.push_back(line);
    }
}

std::vector<cv::String> YOLOInferencer::get_outputs_names(const cv::dnn::Net& net) {
    static std::vector<cv::String> names;
    if (names.empty()) {
        std::vector<int> out_layers = net.getUnconnectedOutLayers();
        std::vector<cv::String> layers_names = net.getLayerNames();
        names.resize(out_layers.size());
        for (size_t i = 0; i < out_layers.size(); ++i) {
            names[i] = layers_names[out_layers[i] - 1];
        }
    }
    return names;
}

std::vector<DetectionResult> YOLOInferencer::infer(const cv::Mat& frame) {
    std::vector<DetectionResult> results;
    if (frame.empty()) {
        return results;
    }

    cv::Mat blob;
    // Standard YOLOv4/v3 preprocessing: 1/255.0 scaling, 416x416 resolution
    cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);

    std::vector<cv::Mat> outs;
    net_.forward(outs, get_outputs_names(net_));

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i) {
        const cv::Mat& out = outs[i];
        float* data = (float*)out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols) {
            cv::Mat scores = out.row(j).colRange(5, out.cols);
            cv::Point class_id_point;
            double confidence;
            cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &class_id_point);

            if (confidence > conf_threshold_) {
                int center_x = static_cast<int>(data[0] * frame.cols);
                int center_y = static_cast<int>(data[1] * frame.rows);
                int width = static_cast<int>(data[2] * frame.cols);
                int height = static_cast<int>(data[3] * frame.rows);
                int left = center_x - width / 2;
                int top = center_y - height / 2;

                class_ids.push_back(class_id_point.x);
                confidences.push_back(static_cast<float>(confidence));
                boxes.emplace_back(left, top, width, height);
            }
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        
        DetectionResult det;
        det.class_id = class_ids[idx];
        det.class_name = classes_.empty() ? "unknown" : classes_[det.class_id];
        det.score = confidences[idx];
        det.x_min = static_cast<float>(box.x);
        det.y_min = static_cast<float>(box.y);
        det.x_max = static_cast<float>(box.x + box.width);
        det.y_max = static_cast<float>(box.y + box.height);
        det.center_x = static_cast<float>(box.x) + static_cast<float>(box.width) / 2.0F;
        det.center_y = static_cast<float>(box.y) + static_cast<float>(box.height) / 2.0F;
        
        // Pseudo-distance estimation based on bounding box height 
        // Can be replaced with depth camera or LiDAR later
        float focal_length = 500.0F;
        float real_height = 0.5F;
        if (det.class_name == "person") {
            real_height = 1.7F;
        } else if (det.class_name == "bottle") {
            real_height = 0.25F; // 일반적인 생수병(25cm) 높이 적용
        }
        det.distance = (real_height * focal_length) / std::max(1.0F, static_cast<float>(box.height));

        results.push_back(det);
    }
    return results;
}
