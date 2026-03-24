#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// A decoupled struct to hold detection results independent of ROS2 messages.
struct DetectionResult {
    int class_id;
    std::string class_name;
    float score;
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    float center_x;
    float center_y;
    float distance; // Estimated distance based on bounding box height or depth sensor
};

/**
 * @brief Abstract Base Class for AI Inference
 * 
 * Design guarantees that backends (YOLO, Hailo) can be swapped seamlessly.
 * This decoupled architecture is essential for edge computing on Raspberry Pi + Hailo.
 */
class AI_Inferencer {
public:
    virtual ~AI_Inferencer() = default;

    /**
     * @brief The primary interface method to map a cv::Mat to a list of detections
     * @param frame input image from camera stream
     * @return std::vector<DetectionResult> list of detected objects
     */
    virtual std::vector<DetectionResult> infer(const cv::Mat& frame) = 0;
};
