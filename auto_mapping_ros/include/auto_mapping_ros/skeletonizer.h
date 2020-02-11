#ifndef AUTO_MAPPING_ROS_SKELETONIZER_H
#define AUTO_MAPPING_ROS_SKELETONIZER_H

#include <opencv2/core.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

class Skeletonizer
{
public:
    Skeletonizer(): map_jpg()
    {}

    void read_map(const std::string& map_filename)
    {
        map_jpg = cv::imread(map_filename, 0);
        //display_img(map_jpg);
    }

    void display_img(cv::Mat img, std::string name = "Result")
    {
        namedWindow( name, cv::WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( name, img );
        cv::waitKey(0);
    }

    void skeletonize()
    {
        /*
        for(int i=0; i< map_jpg.rows; i++) {
            for (int j = 0; j < map_jpg.cols; j++) {
                std::cout << static_cast<int>(map_jpg.at<uchar>(i, j)) << " ";
            }
            std::cout << "\n";
        }
        */

        cv::threshold(map_jpg, map_jpg, 230, 255, cv::THRESH_BINARY);
        //display_img();

        cv::Mat skel(map_jpg.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat temp(map_jpg.size(), CV_8UC1);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

        bool done;
        do
        {
            cv::morphologyEx(map_jpg, temp, cv::MORPH_OPEN, element);
            cv::bitwise_not(temp, temp);

            //display_img(temp);

            cv::bitwise_and(map_jpg, temp, temp);
            cv::bitwise_or(skel, temp, skel);
            cv::erode(map_jpg, map_jpg, element);

            //display_img(skel);

            double max;
            cv::minMaxLoc(map_jpg, 0, &max);
            done = (max == 0);

        } while (!done);

        display_img(skel);

    }

private:
    cv::Mat map_jpg;
};

#endif //AUTO_MAPPING_ROS_SKELETONIZER_H
