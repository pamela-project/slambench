//
// Created by mspear on 07/08/2019.
//

#ifndef SLAMBENCH2_REPOSITORY_FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UTILS_TIMER_H_
#define SLAMBENCH2_REPOSITORY_FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UTILS_TIMER_H_

#include <chrono>

class Timer {

 private:
  std::chrono::high_resolution_clock::time_point start_time_;
  double timestamp = 0.f;

 public:
  Timer() = default;

  void start() {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  void stop() {
    timestamp = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - start_time_).count() * 1000;
  }

  const double& get() {
    return timestamp;
  }
};

#endif  // SLAMBENCH2_REPOSITORY_FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UTILS_TIMER_H_
