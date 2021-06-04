#include <ros/package.h>
#include <ros/ros.h>

#include <filesystem>
#include <object_detection/object_detector.hpp>
#include <object_detection/video_loader.hpp>

namespace fs = std::filesystem;

static const cv::String KEYS =
    "{help h usage ?| | print this message                              }"
    "{video v       | | video specified for tracking. Default: webcam   }"
    "{model m       | | deep learning model type for object detection   }";

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection_node");
    fs::path package_dir = ros::package::getPath("object_detection");

    // OpenCV command line parser
    cv::CommandLineParser parser(argc, argv, KEYS);
    parser.about("The object detection node");
    if (parser.has("help")) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    // determine using webcam or video
    bool use_webcam         = !parser.has("video");
    unsigned int webcam_idx = 0;

    // video source indicator
    cv::String video_path = (use_webcam) ? std::to_string(webcam_idx)
                                         : parser.get<cv::String>("video");
    // object detection model indicator
    fs::path model_path = package_dir, config_path = package_dir;
    if (parser.has("model")) {
        if (parser.get<cv::String>("model") == "darknet") {
            model_path /= fs::path("models/yolov4-tiny.weights");
            config_path /= fs::path("models/yolov4-tiny.cfg");
        } else {
            ROS_ERROR_STREAM(parser.get<cv::String>("model")
                             << " DNN type is unimplemented");
            throw ros::Exception("Unimplemented DNN model type");
        }
    }

    // parser error check
    if (!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    object_detection::VideoLoader video_loader(video_path, use_webcam);
    object_detection::ObjectDetector detector(
        model_path.string(), object_detection::DNNType::DARKNET,
        object_detection::DatasetType::COCO, config_path.string());

    while (video_loader.nextFrame()) {
        const auto detection_results = detector.infer(video_loader.getFrame());
        video_loader.visualize(detection_results);
    }

    return EXIT_SUCCESS;
}
