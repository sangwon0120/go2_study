# Antigravity Rules: Unitree Go2 Vision & Control Project

## 1. Project Overview

The objective of this project is to develop a C++ ROS2 (or Eclipse CycloneDDS) node for the Unitree Go2 robot. The node will subscribe to the front camera topic, process the video feed using OpenCV and YOLO for object recognition, and use the extracted data to command the Go2 robot's control node.

## 2. Tech Stack & Environment

- **Language**: C++ (C++14/17 standard recommended)
- **Middleware**: ROS2 / Eclipse CycloneDDS (based on the provided `unitree_ros2` repository)
- **Vision Processing**: OpenCV (for image decoding and preprocessing), YOLO (for object detection and bounding box extraction)
- **Target Edge Hardware (Current & Future)**: Currently developing for general environments, but the system will ultimately be deployed on a **Raspberry Pi equipped with a Hailo AI accelerator**. The codebase must be highly optimized for edge computing, capable of handling both vision recognition and robot control simultaneously on this board.

## 3. Codebase Reference Guide (`unitree_ros2`)

All generated code and modifications must strictly adhere to the structure and message types of the existing `unitree_ros2` workspace.

- **Message Types**:
  - Location: `cyclonedds_ws/src/unitree/unitree_go/msg/`
  - Vision Data: Use `Go2FrontVideoData.msg` (or equivalent) to receive video frames.
  - High-level Control: `SportModeCmd.msg`, `Req.msg`
  - Low-level Control: `MotorCmd.msg`, `LowCmd.msg`
- **Examples & Client Implementation**:
  - Location: `example/src/src/go2/`, `example/src/src/common/`
  - Utilize provided header files such as `ros2_sport_client.h` and `ros2_robot_state_client.h`. Construct publishers and subscribers based on the existing C++ implementations (e.g., `go2_sport_client.cpp`).

## 4. Core Implementation Pipeline

Ensure the following pipeline operates sequentially and efficiently:

1. **Subscribing (Video Reception)**:
   - Create a subscriber node that listens to the Go2 front camera topic using the exact `unitree_go` message types.
   - Implement the logic to decode the incoming byte arrays or compressed video formats into an OpenCV `cv::Mat` object.
2. **Processing (Vision/AI)**:
   - Pass the decoded `cv::Mat` to the YOLO model.
   - _Architecture Note for Hailo Integration_: Because AI inference will eventually run on a Raspberry Pi + Hailo accelerator, ensure the inference logic is fully decoupled from the main loop and video processing. This modularity is required to easily integrate the HailoRT (Hailo Runtime) C++ APIs later.
   - Extract precise numerical data: bounding boxes, center coordinates (X, Y), and distance estimations of the target objects.
3. **Publishing / Control (Command Execution)**:
   - Determine the required robot movement (e.g., Yaw rotation, forward/backward velocity) based on the calculated vision data.
   - Instantiate and populate `SportModeCmd` (or relevant control messages) and publish them to the Go2 control topic.

## 5. C++ Coding & Development Guidelines

- **Accuracy & Efficiency**: Real-time performance in the vision processing and control loop is critical, especially given the resource constraints of a Raspberry Pi. Minimize unnecessary memory allocations/copies and use pointers or references where appropriate.
- **Modularity**: Clearly separate the video decoding, AI inference (YOLO/Hailo), and robot control logic into distinct components or classes.
- **Explicit Typing**: Avoid the overuse of the `auto` keyword. Ensure explicit type casting and robust error handling, especially during conversions between ROS2/CycloneDDS message types and OpenCV data structures.
- **Documentation**: Provide factual, highly accurate technical comments. Explain exactly how data is being processed and which message types are involved. Focus on delivering precise technical information without using analogies or metaphorical language.

## 6. Prompt Response Guidelines for Antigravity

- When requested to write vision processing code, you must include the explicit conversion logic demonstrating how to decode `unitree` messages (like `Go2FrontVideoData`) into `cv::Mat`.
- When requested to write control code, you must construct the message headers and structures strictly based on the Go2 Sport/Low-level control client examples found in `example/src`.
- When generating AI inference code, always structure it so that the YOLO backend can be seamlessly substituted with the Hailo C++ API for the Raspberry Pi environment in the future.
