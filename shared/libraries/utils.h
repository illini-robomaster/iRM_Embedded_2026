/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#pragma once

#include "stdint.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <deque>
/**
 * @brief clip a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to the clipped
 * @param min   range min
 * @param max   range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T clip(T value, T min, T max) {
  return value < min ? min : (value > max ? max : value);
}

/**
 * @brief map a value between -1 and 1 to a given range
 *
 * @tparam T    type of the value
 * @param value value to the clipped
 * @param min   range min
 * @param max   range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T map(T value, T min, T max) {
  return value * (max - min) / 2 + (max - min) / 2 + min;
}

/**
 * @brief absolute value
 *
 * @tparam T type of the value
 * @param val value to be taken absolute value of
 *
 * @return absolute value of the value
 */
// template <typename T>
// T abs(T val) {
//   return val < 0 ? -val : val;
// }

/**
 * @brief wrap around a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to be wrapped around
 * @param min   range min
 * @param max   range max
 *
 * @return wrapped around value that falls in the range [min, max]
 *
 * @note undefined behavior if value is more than one cycle away from min or max
 */
template <typename T>
T wrap(T value, T min, T max) {
  const T range = max - min;
  return value < min ? value + range : (value > max ? value - range : value);
}

/**
 * @brief wrap around a value to fall into a given range, can deal with offset of >1 cycle
 *
 * @tparam T    type of the value
 * @param value value to be wrapped around
 * @param min   range min
 * @param max   range max
 *
 * @return wrapped around value that falls in the range [min, max]
 *
 * @note undefined behavior if value is more than one cycle away from min or max
 */
template <typename T>
T hard_wrap(T value, T min, T max) {
  const T range = max - min;
  while(value < min) value += range;
  while(value > max) value -= range;
  return value;
}


/**
 * @brief clip a value to fall into a given range; can wrap around domain
 *                        (designed for angle-based control 2*pi!)
 *
 * @tparam T        type of the value
 * @param value     value to the clipped
 * @param min       clipping range min
 * @param max       clipping range max
 * @param range_min domain range min
 * @param range_max domain range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T wrapping_clip(T value, T min, T max, T range_min, T range_max) {
  value = wrap<T>(value, range_min, range_max);
  // if mim and max are too close, directly return value for omni
  if (abs(max - min) < 1e-4) {
    return value;
  }
  if (max >= min) {
    return value < min ? min : (value > max ? max : value);
  } else {
    // min > max; wrap around
    T middle = (min + max) / 2;
    if (value <= max && value >= range_min) {
      return value;
    } else if (value >= min && value <= range_max) {
      return value;
    } else if (value > max && value < middle) {
      return max;
    } else {
      return min;
    }
  }
}

/**
 * @brief max of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the max of the values
 */
template <typename T>
T max(T value1, T value2) {
  return value1 < value2 ? value2 : value1;
}

/**
 * @brief min of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the min of the values
 */
template <typename T>
T min(T value1, T value2) {
  return value1 < value2 ? value1 : value2;
}

/**
 * @brief get sign of a value
 *
 * @tparam T    type of the value
 * @param value value to be tested
 * @param zero  zero point of that value
 *
 * @return -1 for less than, 1 for greater than, and 0 for equal to
 */
template <typename T>
int sign(T value, T zero) {
  return value < zero ? -1 : (value > zero ? 1 : 0);
}

class BoolEdgeDetector {
 public:
  BoolEdgeDetector(bool initial);
  void input(bool signal);
  bool edge();
  bool posEdge();
  bool negEdge();

 private:
  bool prev_;
  bool posEdge_;
  bool negEdge_;
};

class FloatEdgeDetector {
 public:
  FloatEdgeDetector(float initial, float threshold);
  void input(float signal);
  bool edge();
  bool posEdge();
  bool negEdge();

 private:
  float prev_;
  float threshold_;
  bool posEdge_;
  bool negEdge_;
};

/**
 * @brief Converts a float to an unsigned int, given range and number of bits;
   *      see m4310 V1.2 document for detail
 * @param x value to be converted
 * @param x_min minimum value of the current parameter
 * @param x_max maximum value of the current parameter
 * @param bits size in bits
 * @return value converted from float to unsigned int
 */
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

/**
 * @brief Converts an unsigned int to a float, given range and number of bits;
 *      see m4310 V1.2 document for detail
 * @param x value to be converted
 * @param x_min minimum value of the current parameter
 * @param x_max maximum value of the current parameter
 * @param bits size in bits
 * @return value converted from unsigned int to float
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/**
 * @brief A moving average filter
 * @param window_size the size of sliding window for average filtering
 * @return the filtered value
 */
template <typename T>
class MovingAverageFilter {
public:
    MovingAverageFilter(size_t window_size):window_size_(window_size){}

    T update(T measurement){
      if (window.size() >= window_size_) {
        window.pop_front();
      }
      window.push_back(measurement);
      T sum = 0;
      for (const T &val: window){
        sum += val;
      }
      return sum / window.size();
    }

private:
    size_t window_size_;
    std::deque<T> window;
};
