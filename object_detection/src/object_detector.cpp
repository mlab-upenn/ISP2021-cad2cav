#include <ros/console.h>
#include <ros/exception.h>

#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <fstream>
#include <object_detection/object_detector.hpp>

namespace object_detection {

ObjectDetector::ObjectDetector(const std::string model_path, DNNType dnn_type,
                               const std::string cfg_path)
    : net_type_(dnn_type), img_input_h_(-1), img_input_w_(-1) {
    if (dnn_type == DNNType::DARKNET) {
        assert(!cfg_path.empty());

        // load Darknet model
        net_ = cv::dnn::readNetFromDarknet(cfg_path, model_path);

        // read in config path for Darknet parameters
        //  Darknet parameters is stored as an INI-like config file
        //  Reference: https://gist.github.com/randomphrase/10801888
        std::ifstream config_file(cfg_path);
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

    } else {
        ROS_ERROR("DNN type has not been supported.");
    }

    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}

}  // namespace object_detection
