#include <ros/console.h>
#include <ros/exception.h>
#include <ros/package.h>

#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <object_detection/object_detector.hpp>

namespace fs = std::experimental::filesystem;

namespace object_detection {

ObjectDetector::ObjectDetector(const std::string model_path, DNNType dnn_type,
                               DatasetType dataset_type,
                               const std::string cfg_path)
    : net_type_(dnn_type),
      dataset_type_(dataset_type),
      img_input_h_(-1),
      img_input_w_(-1) {
    if (dnn_type == DNNType::DARKNET) {
        assert(!cfg_path.empty());

        // load Darknet model
        net_ = std::make_unique<cv::dnn::Net>(
            cv::dnn::readNetFromDarknet(cfg_path, model_path));
        ROS_INFO_STREAM("Loaded Darknet model from: \n"
                        << "\t\t" << model_path << "\n\tconfig from:\n"
                        << "\t\t" << cfg_path);

        // read in config path for Darknet parameters
        //  Darknet parameters is stored as an INI-like config file
        //  Reference: https://gist.github.com/randomphrase/10801888
        std::ifstream config_file(cfg_path, std::ios::in);
        if (!config_file) {
            ROS_FATAL_STREAM("I/O error on config file: " << cfg_path);
            throw ros::Exception("I/O error on Darknet config file");
        }

        namespace po = boost::program_options;

        // init program options
        po::options_description global_opt("Global Options");
        global_opt.add_options()("width", po::value<int>()->default_value(416),
                                 "Image Width")(
            "height", po::value<int>()->default_value(416), "Image Height");
        // init variable maps & store parsed configs into Boost variable map
        po::variables_map vm;
        po::store(po::parse_config_file(config_file, global_opt, true), vm);
        // retrieve desired fields
        img_input_h_ = vm["height"].as<int>();
        img_input_w_ = vm["width"].as<int>();
        ROS_INFO_STREAM("Read in DNN input blob dimension [w*h]: "
                        << img_input_w_ << "x" << img_input_h_);
        config_file.close();
    } else {
        ROS_ERROR("DNN type has not been supported.");
        throw ros::Exception("DNN type has not been supported");
    }

#ifdef USE_CUDA
    net_->setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net_->setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
#else
    net_->setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    net_->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif

    loadDatasetClassNames();
}

void ObjectDetector::loadDatasetClassNames() {
    fs::path package_dir = ros::package::getPath("object_detection");
    fs::path label_path;

    if (dataset_type_ == DatasetType::COCO) {
        label_path = package_dir / "models/coco.names";
    } else {
        ROS_ERROR_STREAM("Dataset type has not been supported");
        throw ros::Exception("Dataset type has not been supported");
    }

    std::ifstream label_file(label_path.string(), std::ios::in);
    if (label_file.is_open()) {
        std::string label;
        while (std::getline(label_file, label)) {
            class_names_.emplace_back(label);
        }
        ROS_INFO("Loaded dataset class names. %ld classes in total",
                 class_names_.size());
        label_file.close();
    } else {
        ROS_FATAL_STREAM("I/O error on label file: " << label_path);
        throw ros::Exception("I/O error on loading dataset label file");
    }
}

std::vector<BoundingBox> ObjectDetector::infer(const cv::Mat& frame) {
    // Reference:
    //  https://github.com/opencv/opencv/blob/8c25a8eb7b10fb50cda323ee6bec68aa1a9ce43c/samples/dnn/object_detection.cpp#L192-L221
    //  https://funvision.blogspot.com/2020/04/simple-opencv-tutorial-for-yolo-darknet.html
    //  https://learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/

    std::vector<BoundingBox> ret;

    // check if frame is empty
    if (frame.empty()) {
        ROS_ERROR("Empty image input for object detector!");
        return ret;
    }

    // create batch from image
    bool swap_RB = true, crop = false;
    cv::Mat blob =
        cv::dnn::blobFromImage(frame, 1, cv::Size(img_input_h_, img_input_w_),
                               cv::Scalar(), swap_RB, crop);

    // normalize batch; feed batch into network
    double scale    = 1. / 255.0;
    cv::Scalar mean = 0;
    net_->setInput(blob, "", scale, mean);

    // get output
    cv::Mat out = net_->forward();

    // decode output
    //  YOLO output is an Nx(C+5) matrix defined by the following:
    //  [objectness, x, y, w, h, class 1, class 2, ..., class C]
    std::vector<cv::Rect> list_boxes;
    std::vector<float> list_conf_scores;
    std::vector<int> list_class_ids;
    for (int obj_id = 0; obj_id < out.rows; obj_id++) {
        // class confidence score list
        cv::Mat conf_score_list = out.row(obj_id).colRange(5, out.cols);

        // get highest confidence class ID
        cv::Point class_id_pt;
        double max_conf_score;
        cv::minMaxLoc(conf_score_list, nullptr, &max_conf_score, nullptr,
                      &class_id_pt);

        if (max_conf_score > CONF_THRESHOLD) {
            int c_x  = static_cast<int>(out.at<float>(obj_id, 0) * frame.cols);
            int c_y  = static_cast<int>(out.at<float>(obj_id, 1) * frame.rows);
            int w    = static_cast<int>(out.at<float>(obj_id, 2) * frame.cols);
            int h    = static_cast<int>(out.at<float>(obj_id, 3) * frame.rows);
            int left = c_x - w / 2;
            int top  = c_y - h / 2;

            list_boxes.push_back(cv::Rect(left, top, w, h));
            list_conf_scores.push_back(max_conf_score);
            list_class_ids.push_back(class_id_pt.x);
        }
    }

    // perform NMS (Non-Max Suppression) using OpenCV utility function
    std::vector<int> list_nms_result;
    cv::dnn::NMSBoxes(list_boxes, list_conf_scores, CONF_THRESHOLD,
                      NMS_THRESHOLD, list_nms_result);

    // construct elements for return
    for (const auto& i : list_nms_result) {
        ret.push_back({list_boxes[i].x, list_boxes[i].y, list_boxes[i].width,
                       list_boxes[i].height, list_class_ids[i],
                       class_names_[list_class_ids[i]], list_conf_scores[i]});
    }

    return ret;
}

}  // namespace object_detection
