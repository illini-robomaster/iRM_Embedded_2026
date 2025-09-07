#include "moving_average.h"
#include <deque>
// implementation for moving average class

MovingAverage::MovingAverage() {
  window_size_ = 10;
  sum_ = 0;
}

MovingAverage::MovingAverage(int window_size) {
  window_size_ = window_size;
  sum_ = 0;
}

void MovingAverage::AddSample(double sample) {
  samples_.push_back(sample);
  sum_ += sample;
  if (samples_.size() > window_size_) {
    sum_ -= samples_.front();
    samples_.pop_front();
  }
}

void MovingAverage::SetSampleSize(int window_size) {
  window_size_ = window_size;
}

double MovingAverage::GetAverage() const {
  return sum_ / samples_.size();
}

