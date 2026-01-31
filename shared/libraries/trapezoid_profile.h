#pragma once
#include <math.h>
typedef struct {
    float position;
    float velocity;
} kinematics_state;

class TrapezoidProfile 
{
    private:
        // Kinematics Constraints
        float acc;
        float cruise_vel;

        // kinematics states
        kinematics_state m_current_state;

        // estimated critical timestamps
        float end_acc_time;
        float end_cruise_time;
        float end_decc_time; 

        int direction;

    public:
        /**
         * @brief Construct a new Trapezoid Profile object, with acceleration and cruise velocity. 
         * @param acceleration the designed acceleration of the system. Unit is velocity unit per second.
         * @param cruise_velocity the maximum/cruise velocity of the system. Unit is arbitrary. But should be consistent with acceleration
        */
        TrapezoidProfile(float acceleration, float cruise_velocity) : acc(acceleration), cruise_vel(abs(cruise_velocity)){}
        
        /**
        * @brief update the internal target and return a new target that satisfy the acceleration and velocity limit.
        * @param new_target the new target position, if want to keep the current target, set this to the current target.
        * @param delta_t_ms the time that you want to advance to the next tick, can be the typical loop time.
        * @param current_state the current state of the system, which contains position and velocity. Could be the last output of this function.
        * @return kinematics_state which contains position and velocity of the current setpoint at time t.
        */
        kinematics_state calculate(float new_target, int delta_t_ms, kinematics_state current_state);

        /**
         * @brief make sure the direction of the output state matches the 
        */
        kinematics_state direct(kinematics_state in);

};