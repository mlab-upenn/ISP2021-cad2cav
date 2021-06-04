#include <object_detection/video_loader.hpp>

namespace object_detection {

VideoLoader::VideoLoader(const std::string video_path, bool use_webcam)
    : frame_idx_(0), manual_termination_(false) {
    if (use_webcam) {
        ROS_INFO_STREAM("Using webcam as video source...");
    } else {
        ROS_INFO_STREAM("Retrieving video according to path: " << video_path);
    }

    bool stream_successful = (use_webcam) ? capture_.open(std::stoi(video_path))
                                          : capture_.open(video_path);
    if (use_webcam) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, 1024);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 768);
        if (!stream_successful)
            ROS_FATAL("Webcam %d not connected. Please verify!\n",
                      std::stoi(video_path));
    } else if (!stream_successful) {
        ROS_FATAL_STREAM("Filename '" << video_path << "' not found. "
                                      << "Please verify!\n");
    }

    num_frames_ = capture_.isOpened() ? displayVideoProperties() : -1;
}

const cv::Mat& VideoLoader::getFrame() const { return current_frame_; }

bool VideoLoader::nextFrame() {
    bool is_videoEnd = (num_frames_ >= 0) ? (frame_idx_ >= num_frames_) : false;
    if (capture_.isOpened() && !manual_termination_ && !is_videoEnd) {
        capture_ >> current_frame_;
        ++frame_idx_;
        return true;
    }

    return false;
}

int VideoLoader::displayVideoProperties() {
    // detect video properties
    ROS_INFO_STREAM("\t Width: " << capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    ROS_INFO_STREAM("\t Height: " << capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO_STREAM("\t FourCC: " << capture_.get(cv::CAP_PROP_FOURCC));
    ROS_INFO_STREAM("\t Frame rate: " << capture_.get(cv::CAP_PROP_FPS));
    ROS_INFO_STREAM(
        "\t Number of Frames: " << capture_.get(cv::CAP_PROP_FRAME_COUNT));

    return capture_.get(cv::CAP_PROP_FRAME_COUNT);
}

void VideoLoader::visualize() {
    if (current_frame_.empty()) {
        ROS_ERROR("Empty frame %d for display!", frame_idx_);
        return;
    }

    cv::imshow("Video Source", current_frame_);
    char ch             = cv::waitKey(30);
    manual_termination_ = (ch == 'q' || ch == 'Q') ? true : false;
}

void VideoLoader::visualize(const std::vector<BoundingBox>& bbox_list) {
    for (const auto& bbox : bbox_list) {
        cv::putText(current_frame_, bbox.class_name,
                    cv::Point(bbox.left, bbox.top), cv::FONT_HERSHEY_SIMPLEX, 1,
                    cv::Scalar(0, 0, 255), 2);
        cv::rectangle(current_frame_, bbox.getBBox(), cv::Scalar(0, 0, 255));
    }

    visualize();
}

}  // namespace object_detection
