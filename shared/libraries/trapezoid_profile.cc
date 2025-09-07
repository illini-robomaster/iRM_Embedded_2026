#include "trapezoid_profile.h"
#include <math.h>
#include "utils.h"

kinematics_state TrapezoidProfile::direct(kinematics_state in)
{
    return {in.position * direction, in.velocity * direction};
}

// update the internal target and return a new target that satisfy the acceleration and velocity limit.
kinematics_state TrapezoidProfile::calculate(float new_target, int delta_t_ms, kinematics_state current_state)
{
    // modified from FRC wpilib TrapezoidProfile
    direction = new_target > current_state.position ? 1 : -1;
    m_current_state = direct(current_state);
    kinematics_state goal_state = direct({new_target, 0});
    float t = delta_t_ms/1000.0;

    if(m_current_state.velocity > cruise_vel) {
        m_current_state.velocity = cruise_vel;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    float cutoffBegin = m_current_state.velocity / acc; // time from hypothetical start point to current point (vel 0)
    float cutoffDistBegin = cutoffBegin * cutoffBegin * acc / 2.0;   // 0.5 * a * t^2

    float cutoffEnd = goal_state.velocity / acc;  // similar
    float cutoffDistEnd = cutoffEnd * cutoffEnd * acc / 2.0;  

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    float fullTrapezoidDist =
        cutoffDistBegin + (goal_state.position - m_current_state.position) + cutoffDistEnd;
    float accelerationTime = cruise_vel / acc;

    float fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * acc; // total dist - (acceleration) 0.5 * a * t^2 - (decceleration) 0.5 * a * t^2

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
        // 0.5 * a * t^2 = half of trangle area
      accelerationTime = sqrt(fullTrapezoidDist / acc); // v-t profile is an equilateral triangle, half the length of base is the acc time
      fullSpeedDist = 0;
    }

    // current time = 0
    end_acc_time = accelerationTime - cutoffBegin; // if already full speed, this will be 0
    end_cruise_time = end_acc_time + fullSpeedDist / cruise_vel;
    end_decc_time = end_cruise_time + accelerationTime - cutoffEnd;
    kinematics_state result = {m_current_state.position, m_current_state.velocity};

    if (t < end_acc_time) { // if t is in acceleration phase
      result.velocity += t * acc;  // v = v0 + at
      result.position += (m_current_state.velocity + t * acc / 2.0) * t; // v0 * t + 0.5 * a * t^2
    } else if (t < end_cruise_time) { // if t is in cruise phase
      result.velocity = cruise_vel; // constant velocity
      result.position +=
          (m_current_state.velocity + end_acc_time * acc / 2.0) * end_acc_time // distance if acceleration not finished yet
              + cruise_vel * (t - end_acc_time); // cruise phase distance
    } else if (t <= end_decc_time) { // decceleration phase
      result.velocity = goal_state.velocity + (end_decc_time - t) * acc;
      float timeLeft = end_decc_time - t;
      result.position =
          goal_state.position
              - (goal_state.velocity + timeLeft * acc / 2.0) * timeLeft;
    } else { // near finished
      result = goal_state;
    }

    return direct(result);
}