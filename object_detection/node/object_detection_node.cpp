#include <ros/ros.h>

#include <object_detection/video_loader.hpp>

static const cv::String KEYS =
    "{help h usage ?| | print this message                              }"
    "{video v       | | video specified for tracking. Default: webcam   }";

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection_node");

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

    // parser error check
    if (!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    object_detection::VideoLoader video_loader(video_path, use_webcam);

    while (video_loader.nextFrame()) {
        video_loader.visualize();
    }

    return EXIT_SUCCESS;
}
