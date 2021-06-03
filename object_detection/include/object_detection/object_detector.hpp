#ifndef __OBJECT_DETECTION_OBJECT_DETECTOR_HPP__
#define __OBJECT_DETECTION_OBJECT_DETECTOR_HPP__

/**
 * @file object_detector.hpp
 * @author Zhihao Ruan (ruanzh@seas.upenn.edu)
 *
 *
 * @brief Object detector is responsible for loading a pre-trained deep neural
 * network from file, performing inference on video frame, outputing detection
 * results.
 *
 * For reference on loading Darknet deep neural networks with OpenCV:
 *  https://funvision.blogspot.com/2020/04/simple-opencv-tutorial-for-yolo-darknet.html
 *
 *
 * @date 2021-06-01
 */

#include <opencv4/opencv2/dnn/dnn.hpp>
#include <string>

namespace object_detection {

enum class DNNType {
    CAFFE = 0,
    DARKNET,
    MODEL_OPTIMIZER,
    ONNX,
    TENSORFLOW,
    TORCH7
};

class ObjectDetector {
public:
    /**
     * @brief Construct a new Object Detector object
     *
     * @param model_path:   path to model weights file (deployed binaries)
     * @param dnn_type:     type of network architecture
     *                      (supports Caffe, Darknet, Model Optimizer, Tensor RT
     *                      ONNX, Tensorflow, Torch7 scientific format)
     * @param cfg_path:     path to network descrition file.
     *                      Optional in ONNX networks.
     */
    explicit ObjectDetector(const std::string model_path, DNNType dnn_type,
                            const std::string cfg_path = "");

private:
    cv::dnn::Net net_;
    DNNType net_type_;

    int img_input_h_;
    int img_input_w_;
};
};  // namespace object_detection

#endif /* __OBJECT_DETECTION_OBJECT_DETECTOR_HPP__ */
