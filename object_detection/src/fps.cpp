#include <object_detection/fps.hpp>

namespace object_detection {

FPS::FPS() : _numframes(0) {}

void FPS::start() { _start = high_resolution_clock::now(); }

void FPS::stop() { _end = high_resolution_clock::now(); }

void FPS::update() { ++_numframes; }

double FPS::elapsed() const {
    duration<double> diff = _end - _start;
    return diff.count();
}

double FPS::fps() const { return _numframes / elapsed(); }

void FPS::printFPSMessage() const {
    ROS_INFO("\tElapsed time: %lf", elapsed());
    ROS_INFO("\tAverage FPS: %lf", fps());
}

}  // namespace object_detection
