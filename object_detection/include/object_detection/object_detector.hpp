#ifndef __OBJECT_DETECTION_OBJECT_DETECTOR_HPP__
#define __OBJECT_DETECTION_OBJECT_DETECTOR_HPP__

#include <memory>

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

enum class DatasetType { COCO = 0, PASCAL_VOC, ADE20K };

struct BoundingBox {
    int left{-1};
    int top{-1};
    int width{-1};
    int height{-1};
    int class_id{-1};
    std::string class_name;
    double confidence{0.};

    BoundingBox() = default;

    BoundingBox(int l, int t, int w, int h, int id, std::string name,
                double conf)
        : left(l),
          top(t),
          width(w),
          height(h),
          class_id(id),
          class_name(name),
          confidence(conf) {}

    cv::Rect getBBox() const { return cv::Rect(left, top, width, height); }
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
     * @param dataset_type: type of dataset
     *                      (supports COCO, PASCAL_VOC, ADE20K)
     * @param cfg_path:     path to network description file.
     *                      Optional in ONNX networks.
     */
    explicit ObjectDetector(const std::string model_path, DNNType dnn_type,
                            DatasetType dataset_type,
                            const std::string cfg_path = "");

    /**
     * @brief Run inference on the current video frame
     *
     * @param frame:                    current video frame
     * @return std::vector<cv::Rect>:   list of bounding boxes
     *                                  (top, left, width, height)
     */
    std::vector<BoundingBox> infer(const cv::Mat& frame);

    // object detection confidence threshold
    static constexpr double CONF_THRESHOLD = 0.4;
    // object detection NMS (Non-Max Suppression) Threshold
    static constexpr double NMS_THRESHOLD = 0.6;

private:
    /**
     * @brief Loads dataset class names & change class_names_
     *
     */
    void loadDatasetClassNames();

    std::unique_ptr<cv::dnn::Net> net_;
    DNNType net_type_;

    std::vector<std::string> class_names_;
    DatasetType dataset_type_;

    int img_input_h_;
    int img_input_w_;
};
};  // namespace object_detection

#endif /* __OBJECT_DETECTION_OBJECT_DETECTOR_HPP__ */
