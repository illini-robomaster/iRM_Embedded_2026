#include <deque>
#include <cstdint>
class MovingAverage {
 public:
  MovingAverage();
  MovingAverage(int window_size);
  void AddSample(double sample);
  double GetAverage() const;
  void SetSampleSize(int window_size);

 private:
  uint16_t window_size_;
  std::deque<double> samples_;
  double sum_;

};