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

#include "utils.h"

BoolEdgeDetector::BoolEdgeDetector(bool initial) { prev_ = initial; }

void BoolEdgeDetector::input(bool signal) {
  posEdge_ = false;
  negEdge_ = false;
  if (!prev_ && signal)
    posEdge_ = true;
  else if (prev_ && !signal)
    negEdge_ = true;
  prev_ = signal;
}

bool BoolEdgeDetector::edge() { return posEdge_ || negEdge_; }

bool BoolEdgeDetector::posEdge() { return posEdge_; }

bool BoolEdgeDetector::negEdge() { return negEdge_; }

FloatEdgeDetector::FloatEdgeDetector(float initial, float threshold) {
  prev_ = initial;
  threshold_ = threshold;
}

void FloatEdgeDetector::input(float signal) {
  posEdge_ = false;
  negEdge_ = false;
  float diff = signal - prev_;
  if (diff > threshold_)
    posEdge_ = true;
  else if (diff < -threshold_)
    negEdge_ = true;
  prev_ = signal;
}

bool FloatEdgeDetector::edge() { return posEdge_ || negEdge_; }

bool FloatEdgeDetector::posEdge() { return posEdge_; }

bool FloatEdgeDetector::negEdge() { return negEdge_; }

uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t) ((x-offset) * ((float)((1<<bits)-1))/span);
}
// BUG: this does not produce the correct result
// e.g. x_min = -PI and x_max = PI, the result is from -2PI to 0 instead if bits = 16
// probably has to do with int is 16bit and when the bits param is 16

/**
 * @brief Convert an unsigned integer to a float
 * @param bits at most 16
*/
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  int32_t x_int_longer = 0x0000FFFF & x_int; // zero extend 16 bit to 32 bit
  return ((float)x_int_longer) * span / ((float)((1 << bits) - 1)) + offset;
}


//==================================================================================================
// Hero 2024 Gimbal pitch angle calculation:
//==================================================================================================


const double g = 9.81;
const double pi = M_PI;
const double c = 0.05; // drag coefficient; need update
const double m = 0.145; // mass of the projectile; need update


struct State {
    double x;
    double y;
    double vx;
    double vy;
    double h2_minus_h1;
};

State derivatives(const State& state) {
  double v = std::sqrt(state.vx * state.vx + state.vy * state.vy);
  return { state.vx, state.vy, - c / m * v * state.vx, -g - c / m * v * state.vy ,state.h2_minus_h1};
}

State rk4_step(const State& state, double dt) {
  State k1 = derivatives(state);
  State k2 = derivatives({ state.x + k1.x * dt / 2, state.y + k1.y * dt / 2, state.vx + k1.vx * dt / 2, state.vy + k1.vy * dt / 2 ,state.h2_minus_h1});
  State k3 = derivatives({ state.x + k2.x * dt / 2, state.y + k2.y * dt / 2, state.vx + k2.vx * dt / 2, state.vy + k2.vy * dt / 2 ,state.h2_minus_h1});
  State k4 = derivatives({ state.x + k3.x * dt, state.y + k3.y * dt, state.vx + k3.vx * dt, state.vy + k3.vy * dt, state.h2_minus_h1});

  return {
          state.x + dt / 6 * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
          state.y + dt / 6 * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
          state.vx + dt / 6 * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx),
          state.vy + dt / 6 * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy),
          state.h2_minus_h1
  };
}

double simulate(double theta0, double V0, double diff) {
  State state = { 0, 0, V0 * cos(theta0), V0 * sin(theta0) , diff};
  double dt = 0.01;
  double x_final = 0;


  while (state.y >= state.h2_minus_h1) {
    state = rk4_step(state, dt);
    if (state.y <= state.h2_minus_h1) {
      x_final = state.x;
      break;
    }
  }
  return x_final;
}

double find_optimal_angle(double V0, double S, double height_diff) {

  double angle_step = 0.1 * pi / 180; // 0.1 degrees in radians
  double best_angle = 0;
  double min_diff = std::numeric_limits<double>::max();

  for (double theta = 10 * pi / 180; theta <= 80 * pi / 180; theta += angle_step) {
    double distance = simulate(theta, V0, height_diff);
    double diff = std::abs(distance - S);
    if (diff < min_diff) {
      min_diff = diff;
      best_angle = theta;
    }
  }
  return best_angle * 180 / pi; // Convert radians back to degrees
}

