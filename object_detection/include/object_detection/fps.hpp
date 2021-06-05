#ifndef __OBJECT_DETECTION_FPS_HPP__
#define __OBJECT_DETECTION_FPS_HPP__

/**
 * @file fps.hpp
 * @author Zhihao Ruan (ruanzh@grasp.upenn.edu)
 * @brief An FPS counter class for calculating the FPS of the video.
 *      This class implements the behavior from Python's imutils FPS class.
 *
 *      Citation: imutils.video, fps.py
 *
 * @date 2020-07-09
 * @copyright Zhihao Ruan (c) 2020
 *
 */

#include <ros/console.h>

#include <chrono>

using namespace std::chrono;

namespace object_detection {

class FPS {
public:
    /**
     * @brief Construct a new FPS object
     *
     */
    FPS();

    /**
     * @brief Starts the timer
     *
     */
    void start();

    /**
     * @brief Stops the timer
     *
     */
    void stop();

    /**
     * @brief Updates the number of frames count
     *
     */
    void update();

    /**
     * @brief Returns the time duration in seconds.
     *
     * @return double:  time duration. Unit: s
     */
    double elapsed() const;

    /**
     * @brief Returns the average FPS calculated
     *
     * @return double:  average FPS calculated
     */
    double fps() const;

    /**
     * @brief Calculates average FPS and print to ROS console
     *
     */
    void printFPSMessage() const;

private:
    high_resolution_clock::time_point _start;
    high_resolution_clock::time_point _end;
    unsigned int _numframes;
};

}  // namespace object_detection

#endif /* __OBJECT_DETECTION_FPS_HPP__ */
