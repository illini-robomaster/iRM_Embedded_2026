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

#include "bsp_can.h"
#include "bsp_pwm.h"
#include "controller.h"
#include "utils.h"

namespace control {

    constexpr int MOTOR_RANGE = 30000;  // TODO: 32767 or 30000?

/**
 * @brief two modes for GetTheta()
 *  absolute_mode: -inf to +inf (Default option)
 *  relative_mode: 0 - 2pi
**/
    enum GetThetaMode {absolute_mode, relative_mode};

    int16_t ClipMotorRange(float output);

    typedef enum {
        MIT = 0,
        POS_VEL = 1,
        VEL = 2,
    } mode_t;

//==================================================================================================
// MotorBase
//==================================================================================================

/**
 * @brief base class for motor representation
 */
    class MotorBase {
    public:
        MotorBase() : output_(0) {}
        virtual ~MotorBase() {}

        virtual void SetOutput(int16_t val) { output_ = val; }

    protected:
        int16_t output_;
    };

//==================================================================================================
// MotorCanBase(DJI Base)
//==================================================================================================

/**
 * @brief base class for CAN motor representation
 */
    class MotorCANBase : public MotorBase {
    public:
        /**
         * @brief base constructor
         *
         * @param can    CAN instance
         * @param rx_id  CAN rx id
         */
        MotorCANBase(bsp::CAN* can, uint16_t rx_id);

        /**
         * @brief base constructor for non-DJI motors
         * @param can     CAN instance
         * @param rx_id   CAN rx id
         * @param type    motor type
         */
        MotorCANBase(bsp::CAN* can, uint16_t rx_id, uint16_t type);

        /**
         * @brief update motor feedback data
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        virtual void UpdateData(const uint8_t data[]) = 0;

        /**
         * @brief print out motor data
         */
        virtual void PrintData() const = 0;

        /**
         * @brief get rotor (the cap spinning on the back of the motor) angle, in [rad]
         *
         * @return radian angle, range between [0, 2PI]
         */
        virtual float GetTheta() const;

        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        virtual float GetThetaDelta(const float target) const;

        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @return angular velocity
         */
        virtual float GetOmega() const;

        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        virtual float GetOmegaDelta(const float target) const;

        virtual int16_t GetCurr() const;

        virtual uint16_t GetTemp() const;

        virtual float GetTorque() const;

        /**
         * @brief transmit CAN message for setting motor outputs
         *
         * @param motors[]    array of CAN motor pointers
         * @param num_motors  number of motors to transmit
         */
        static void TransmitOutput(MotorCANBase* motors[], uint8_t num_motors);
        /**
         * @brief set ServoMotor as friend of MotorCANBase since they need to use
         *        many of the private parameters of MotorCANBase.
         */
        friend class ServoMotor;
        friend class Steering6020;

        volatile bool connection_flag_ = false;

    protected:
        volatile float theta_;
        volatile float omega_;

    private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
    };

//==================================================================================================
// Motor2006
//==================================================================================================

/**
 * @brief DJI 2006 motor class
 */
    class Motor2006 : public MotorCANBase {
    public:
        /* constructor wrapper over MotorCANBase */
        Motor2006(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

    private:
        volatile int16_t raw_current_get_ = 0;
    };

//==================================================================================================
// Motor3508
//==================================================================================================

/**
 * @brief DJI 3508 motor class
 */
    class Motor3508 : public MotorCANBase {
    public:
        /* constructor wrapper over MotorCANBase */
        Motor3508(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;



    private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
    };



//==================================================================================================
// Motor 3510
//==================================================================================================

/**
 * @brief DJI 3510 motor class
 */
    class Motor3510 : public MotorCANBase {
    public:
        /* constructor wrapper over MotorCANBase */
        Motor3510(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;


    private:
        volatile float torque_ = 0; /* Torque Value*/
        volatile float previous_position_ = 0;

    };

//==================================================================================================
// Motor6020
//==================================================================================================

/**
 * @brief DJI 6020 motor class
 */
    class Motor6020 : public MotorCANBase {
    public:
        /* constructor wrapper over MotorCANBase */
        Motor6020(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;

    private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
    };

//==================================================================================================
// Motor6623
//==================================================================================================

/**
 * @brief DJI 6623 motor class
 */
    class Motor6623 : public MotorCANBase {
    public:
        /* constructor wrapper over MotorCANBase */
        Motor6623(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;
        /* override default implementation with not implemented */
        float GetOmega() const override final;
        float GetOmegaDelta(const float target) const override final;

    private:
        volatile int16_t raw_current_get_ = 0;
        volatile int16_t raw_current_set_ = 0;

        static const int16_t CURRENT_CORRECTION = -1;  // current direction is reversed
    };

//==================================================================================================
// MotorPWMBase
//==================================================================================================

/**
 * @brief PWM motor base class
 */
    class MotorPWMBase : public MotorBase {
    public:
        /**
         * @brief constructor
         *
         * @param htim           HAL timer handle
         * @param channel        HAL timer channel, from [0, 4)
         * @param clock_freq     clock frequency associated with the timer, in [Hz]
         * @param output_freq    desired output frequency, in [Hz]
         * @param idle_throttle  idling pulse width, in [us]
         *
         * @note M3508 have idle_throttle about 1500, snail have idle_throttle about 1100
         */
        MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq,
                     uint32_t idle_throttle);

        /**
         * @brief set and transmit output
         *
         * @param val offset value with respect to the idle throttle pulse width, in [us]
         */
        virtual void SetOutput(int16_t val) override;

    private:
        bsp::PWM pwm_;
        uint32_t idle_throttle_;
    };

//==================================================================================================
// PDIHV
//==================================================================================================

/**
 * @brief Hero Trigger servo
 */
    class PDIHV : public MotorPWMBase {
    public:
        PDIHV(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq,
              uint32_t idle_throttle);
        /* override base implementation with max angle */
        void SetOutPutAngle(float deg);
        void SetOutput(int16_t val) override final;
    };

//==================================================================================================
// Motor2305
//==================================================================================================

/**
 * @brief DJI snail 2305 motor class
 */
    class Motor2305 : public MotorPWMBase {
    public:
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;
    };



//==================================================================================================
// ServoMotor
//==================================================================================================

/**
 * @brief servomotor turning mode
 * @note the turning direction is determined as if user is facing the motor, may subject to
 *       change depending on motor type
 */
    typedef enum {
        SERVO_CLOCKWISE = -1,   /* Servomotor always turn clockwisely                      */
        SERVO_NEAREST = 0,      /* Servomotor turn in direction that make movement minimum */
        SERVO_ANTICLOCKWISE = 1 /* Servomotor always turn anticlockwisely                  */
    } servo_mode_t;

/**
 * @brief servomotor status
 * @note the turning direction is determined as if user is facing the motor, may subject to
 *       change depending on motor type
 */
    typedef enum {
        TURNING_CLOCKWISE = -1,   /* Servomotor is turning clockwisely         */
        INPUT_REJECT = 0,         /* Servomotor rejecting current target input */
        TURNING_ANTICLOCKWISE = 1 /* Servomotor is turning anticlockwisely     */
    } servo_status_t;

/**
 * @brief transmission ratios of DJI motors, reference to motor manuals for more details
 */
#define M3508P19_RATIO (3591.0 / 187) /* Transmission ratio of M3508P19 */
#define M2006P36_RATIO 36             /* Transmission ratio of M2006P36 */

    typedef struct {
        servo_mode_t mode; /* turning mode of servomotor, refer to type servo_mode_t */
        float speed;       /* motor shaft turning speed                              */
    } servo_jam_t;

    class ServoMotor;  // declare first for jam_callback_t to have correct param type
/**
 * @brief jam callback template
 */
    typedef void (*jam_callback_t)(ServoMotor* servo, const servo_jam_t data);

/**
 * @brief structure used when servomotor instance is initialized
 */
    typedef struct {
        MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
        float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
        float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
        float transmission_ratio; /* transmission ratio of motor                       */
        float* omega_pid_param;   /* pid parameter used to control speed of motor      */
        float max_iout;
        float max_out;
    } servo_t;

/**
 * @brief wrapper class for motor to enable the motor shaft angle to be precisely controlled with
 *        possible external gearbox present
 * @note this is a calculation class that calculate the motor output for desired output, but it does
 * not directly command a motor to turn.
 */
    class ServoMotor {
    public:
        /**
         * @brief base constructor
         *
         * @param servo         initialization struct, refer to type servo_t
         * @param proximity_in  critical difference angle for the motor to enter hold state
         * @param proximity_out critical difference angle for the motor to exit hold state
         *
         * @note proximity_out should be greater than proximity_in
         */
        ServoMotor(servo_t data, float align_angle = -1, float proximity_in = 0.05,
                   float proximity_out = 0.15);

        /**
         * @brief set next target for servomotor, will have no effect if last set target has not been
         * achieved
         * @note if motor is not holding, call to this function will have no effect unless override is
         * true
         *
         * @param target   next target for the motor in [rad]
         * @param override if true, override current target even if motor is not holding right now
         * @return current turning mode of motor
         */
        servo_status_t SetTarget(const float target, bool override = false);

        void ResetTheta();

        /**
         * @brief set turning speed of motor when moving
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxSpeed(const float max_speed);

        void SetMaxCurrent(servo_t data, const float max_current);

        /**
         * @brief set acceleration of motor when moving
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_acceleration speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxAcceleration(const float max_acceleration);


        MotorCANBase* GetMotor() const;


        bool ZeroingExit() const;
        /**
         * @brief calculate the output of the motors under current configuration
         * @note this function will not command the motor, it only calculate the desired input
         */
        void CalcOutput(int *value = nullptr);

        /**
         * @brief if the motor is holding
         *
         * @return true  the motor is holding (i.e. not turning)
         * @return false the motor is not holding (i.e. turning)
         */
        bool Holding() const;

        /**
         * @brief get current servomotor target, in [rad]
         *
         * @return current target angle, range between [0, 2PI]
         */
        float GetTarget() const;

        /**
         * @brief register a callback function that would be called if motor is jammed
         * @note Jam detection uses a moving window across inputs to the motor. It uses a circular buffer
         * of size detect_period to store history inputs and calculates a rolling average of the inputs.
         *       Everytime the average of inputs is greater than
         *       effect_threshold * 32768(maximum command a motor can accept), the jam callback function
         * will be triggered once. The callback will only be triggered once each time the rolling average
         *       cross the threshold from lower to higher. For a standard jam callback function, refer to
         *       example motor_m3508_antijam
         *
         * @param callback         callback function to be registered
         * @param effort_threshold threshold for motor to be determined as jammed, ranged between (0, 1)
         * @param detect_period    detection window length
         */
        void RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                 uint8_t detect_period = 50);

        /**
         * @brief print out motor data
         */
        void PrintData() const;

        /**
         * @brief get motor angle, in [rad]
         *
         * @ param mode   if mode == absolute_mode, range -> [-inf to +inf]
         *                if mode == relative_mode, range -> [0 to 2pi]
         *
         * @return radian angle, range between [-inf, +inf]
         */
        float GetTheta(GetThetaMode mode = absolute_mode) const;

        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        float GetThetaDelta(const float target) const;

        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @return angular velocity
         */
        float GetOmega() const;

        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        float GetOmegaDelta(const float target) const;

        /**
         * @brief update the current theta for the servomotor
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        void UpdateData(const uint8_t data[]);



        friend class SteeringMotor;


    private:
        // refer to servo_t for details
        MotorCANBase* motor_;
        float max_speed_;
        float max_acceleration_;
        float transmission_ratio_;
        float proximity_in_;
        float proximity_out_;

        bool zeroing_exit; /* flag to exit zeroing */
        // angle control
        bool hold_; /* true if motor is holding now, otherwise moving now                      */
        uint32_t start_time_;
        float target_angle_; /* desired target angle, range between [0, 2PI] in [rad]                   */
        float align_angle_;  /* motor angle when a instance of this class is created with that motor    */
        float motor_angle_;  /* current motor angle in [rad], with align_angle subtracted               */
        float offset_angle_; /* cumulative offset angle of motor shaft, range between [0, 2PI] in [rad] */
        float servo_angle_;  /* current angle of motor shaft, range between [0, 2PI] in [rad]           */
        float cumulated_angle_;

        // jam detection
        jam_callback_t jam_callback_; /* callback function that will be invoked if motor jammed */
        int detect_head_;     /* circular buffer current head                                       */
        int detect_period_;   /* circular buffer length                                             */
        int detect_total_;    /* rolling sum of motor inputs                                        */
        int jam_threshold_;   /* threshold for rolling sum for the motor to be considered as jammed */
        int16_t* detect_buf_; /* circular buffer                                                    */

        // pid controllers
        ConstrainedPID omega_pid_; /* pid for controlling speed of motor */

        // edge detectors
        FloatEdgeDetector* inner_wrap_detector_; /* detect motor motion across encoder boarder */
        FloatEdgeDetector* outer_wrap_detector_; /* detect motor motion across encoder boarder */
        BoolEdgeDetector* hold_detector_; /* detect motor is in mode toggling, reset pid accordingly  */
        BoolEdgeDetector* jam_detector_;  /* detect motor jam toggling, call jam callback accordingly */
    };

//==================================================================================================
// SteeringMotor
//==================================================================================================

// function pointer for the calibration function of the steering motor, return True when calibrated
    typedef bool (*align_detect_t)(void);

/**
 * @brief structure used when steering motor instance is initialized
 */
    typedef struct {
        MotorCANBase* motor;              /* motor instance to be wrapped as a servomotor       */
        float max_speed;                  /* max turning speed of motor shaft, in [rad/s]       */
        float max_acceleration;           /* desired acceleration of motor shaft, in [rad/s^2]  */
        float transmission_ratio;         /* transmission ratio of motor                        */
        float* omega_pid_param;           /* pid parameter used to control speed of motor       */
        float max_iout;
        float max_out;
        align_detect_t align_detect_func; /* function pointer for calibration function*/
        float calibrate_offset;           /* angle from calibration sensor to starting location */
    } steering_t;

// mode that can turn relative angles in [rad]
    class SteeringMotor {
    public:
        SteeringMotor(steering_t data);
        /**
         * @brief Call ServoMotor::GetTheta() and return theta in [rad]
         *
         * @ param mode   if mode == absolute_mode, range -> [-inf to +inf]
         *                if mode == relative_mode, range -> [0 to 2pi]
         */
        float GetTheta(GetThetaMode mode = absolute_mode) const;

        /**
         * @brief Print out motor data
         */
        void PrintData() const;

        /**
         * @brief Set the target to a relative angle in [rad]
         *        if the underlying servomotor is not holding, the motor won't turn
         * @param override if true, override current target even if motor is not holding right now
         * @return 0 when the command is accepted, 1 otherwise
         */
        int TurnRelative(float angle, bool override = false);

        /**
         * @brief Turn the motor to the aligned position
         *        Do nothing if the motor doesn't have an aligned position
         * @return 2 if the motor doesn't have an aligned position
         *         1 if the motor has one but failed to reach it
         *         0 if success
         */
        int ReAlign();

        /**
         * @brief find the aligned position
         *        If the motor doesn't have an aligned position, the motor keeps rotating until the detector returns True.
         *        If the motor has one, the motor does nothing and return True.
         * @note  Calibrate() only find the aligned position, it doesn't guarantee the motor ends in the aligned position or stops
         *
         *
         * @return True when the motor has a aligned position
         */
        bool Calibrate();

        /**
         * @brief set turning speed of motor when moving
         *        Call ServoMotor::SetMaxSpeed()
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxSpeed(const float max_speed);

        /**
         * @brief Call Servomotor::CalcOutput()
         */
        void CalcOutput();

        /**
         * @brief Call ServoMotor::UpdateData()
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        void UpdateData(const uint8_t data[]);

        /**
         * @brief Set align_complete_ of the motor to False
         */
        void SetAlignFalse();

        /**
         * @brief Check the motor's align_complete_ flag
         * @return True if motor's align_complete_ flag is true, false otherwise
         */
        bool CheckAlignment();

    private:
        ServoMotor* servo_;

        align_detect_t align_detect_func_;/* function pointer for the calibration sensor, see comments for align_detect_t typedef */
        float calibrate_offset_;          /* difference between calibration sensor and desired starting position                  */
        float current_target_;            /* current absolute position in [rad] to drive the underlying servo motor               */

        float align_angle_;               /* store calibration angle                                                              */
        bool align_complete_;             /* if calibration is previously done, use the align_angle_                              */
    };


// =================================================================================================
// ServoMotorWithLG 
// =================================================================================================
// This is a ServoMotor with light gate calibration and forward & reverse soft limits

/**
 * @brief structure used when steering motor instance is initialized
 */
    typedef struct {
        MotorCANBase* motor;              /* motor instance to be wrapped as a servomotor       */
        float max_speed;                  /* max turning speed of motor shaft, in [rad/s]       */
        float max_acceleration;           /* desired acceleration of motor shaft, in [rad/s^2]  */
        float transmission_ratio;         /* transmission ratio of motor                        */
        float* omega_pid_param;           /* pid parameter used to control speed of motor       */
        float max_iout;
        float max_out;
        align_detect_t align_detect_func; /* function pointer for calibration function*/
        float calibrate_offset;           /* angle from calibration sensor to starting location */
        float reverse_soft_limit;
        float forward_soft_limit;
        bool align_dir_invert;
    } servoLG_t;

// mode that can turn relative angles in [rad]
    class ServoMotorWithLG {// light gate
    public:
        ServoMotorWithLG(servoLG_t data);
        /**
         * @brief Call ServoMotor::GetTheta() and return theta in [rad]
         *
         * @ param mode   if mode == absolute_mode, range -> [-inf to +inf]
         *                if mode == relative_mode, range -> [0 to 2pi]
         */
        float GetTheta(GetThetaMode mode = absolute_mode) const;

        /**
         * @brief Print out motor data
         */
        void PrintData() const;

        /**
         * @brief Set the target to a relative angle in [rad]
         *        if the underlying servomotor is not holding, the motor won't turn
         * @param override if true, override current target even if motor is not holding right now
         * @return 0 when the command is accepted, 1 otherwise
         */
        int TurnRelative(float angle, bool override = false);


        /**
         * @brief Set the target to an absolute angle in [rad]
         *        if the underlying servomotor is not holding, the motor won't turn
         * @param override if true, override current targeteven if motor is not holding right now
         * @return 0 when the command is accepted, 1 otherwise
        */
        int TurnAbsolute(float angle_rad, bool override = false);


        /**
         * @brief Turn the motor to the aligned position
         *        Do nothing if the motor doesn't have an aligned position
         * @return 2 if the motor doesn't have an aligned position
         *         1 if the motor has one but failed to reach it
         *         0 if success
         */
        int ReAlign();

        /**
         * @brief find the aligned position
         *        If the motor doesn't have an aligned position, the motor keeps rotating until the detector returns True.
         *        If the motor has one, the motor does nothing and return True.
         * @note  Calibrate() only find the aligned position, it doesn't guarantee the motor ends in the aligned position or stops
         *
         *
         * @return True when the motor has a aligned position
         */
        bool Calibrate();

        /**
         * @brief set turning speed of motor when moving
         *        Call ServoMotor::SetMaxSpeed()
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxSpeed(const float max_speed);

        /**
         * @brief Call Servomotor::CalcOutput()
         */
        void CalcOutput();

        /**
         * @brief Call ServoMotor::UpdateData()
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        void UpdateData(const uint8_t data[]);

        /**
         * @brief Set align_complete_ of the motor to False
         */
        void SetAlignFalse();

        /**
         * @brief Check the motor's align_complete_ flag
         * @return True if motor's align_complete_ flag is true, false otherwise
         */
        bool CheckAlignment();

        ServoMotor* GetServo();

       private:
        ServoMotor* servo_;

        align_detect_t align_detect_func_;/* function pointer for the calibration sensor, see comments for align_detect_t typedef */
        float calibrate_offset_;          /* difference between calibration sensor and desired starting position                  */
        float current_target_;            /* current absolute position in [rad] to drive the underlying servo motor               */
        float reverse_soft_limit;         /* forward soft limit relative to calib position */
        float forward_soft_limit;         /* reverse soft limit relative to calib position */

        float align_angle_;               /* angle relative to motor start position                                                              */
        bool align_complete_;             /* if calibration is previously done, use the align_angle_                              */
        bool align_dir_invert_;
    };


// =================================================================================================

//==================================================================================================
// Motor4310
//==================================================================================================

/**
 * @brief m4310 motor class
 */
class Motor4310 {
 public:
  /** constructor wrapper over MotorCANBase
   *  CAN frame id for different modes:
   *      MIT:                  actual CAN id.
   *      position-velocity:    CAN id + 0x100.
   *      velocity:             CAN id + 0x200.
   *  @param can    CAN object
   *  @param rx_id  Master id
   *  @param tx_id  CAN id *** NOT the actual CAN id but the id configured in software ***
   *  @param mode   0: MIT
   *                1: position-velocity
   *                2: velocity
   */
  Motor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, mode_t mode);

  Motor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, mode_t mode, float P_MAX);

        /* implements data update callback */
        void UpdateData(const uint8_t data[]);

        /* enable m4310; MUST be called after motor is powered up, otherwise SetOutput commands are ignored */
        void MotorEnable();
        /* disable m4310 */
        void MotorDisable();

        /** sets current motor position as zero position (when motor is powered). M4310 remembers this position
         * when powered off. */
        void SetZeroPos();

        /**
         * implements transmit output specifically for 4310
         * @param motor m4310 motor object
         * @param mode operation modes:
         *                0: MIT
         *                1: position-velocity
         *                2: velocity
         */
        static void TransmitOutput(control::Motor4310* motors[], uint8_t num_motors);

        /* implements data printout */
        void PrintData();

        /**
         * Sets output parameters for m4310 using the MIT mode
         *
         * Several control modes can be derived from the MIT mode:
         * 1. Velocity mode:
         *    Rotates the motor at a constant velocity by setting kp to zero, kd to a non-zero
         *    value, and velocity to the target angular velocity
         *    e.g. motor->SetOutput(0, 3, 0, 1, 0);
         * 2. Position mode:
         *    Rotates the motor to a target position by setting position to a relative target
         *    position. Note: kd must be set to a NON-ZERO value to avoid oscillation
         *    e.g. motor->SetOutput(2*PI, 0, 0.4, 0.05, 0);
         * 3. Torque mode:
         *    Rotates the motor at a given torque by setting torque to a desired value. kp and kd
         *    should be set to zero. Note: under no load, a small torque
         *    e.g. motor->SetOutput(0, 0, 0, 0, 1);
         *
         * @param position relative target position in radian (can exceed 2*PI)
         * @param velocity target angular velocity (rad/s)
         * @param kp p gain (N/rad)
         * @param kd d gain (N*s/rad)
         * @param torque target torque (Nm)
         */
        void SetOutput(float position, float velocity, float kp, float kd, float torque);

        /**
         * Sets output parameters for m4310 using position-velocity mode
         * e.g. motor->SetOutput(2*PI, 1);
         *
         * Note: oscillations can be alleviated by tuning the acceleration/deceleration parameters
         * through the DAMIAO helper tool. The damping factor needs to be a non-zero positive number
         *
         * @param position relative target position in radian (can exceed 2*PI)
         * @param velocity maximum absolute angular velocity (rad/s)
         */
        void SetOutput(float position, float velocity);

        /**
         * Sets output parameters for m4310 using velocity mode
         *
         * e.g. motor->SetOutput(1);
         * @param velocity target angular velocity (rad/s)
         */
        void SetOutput(float velocity);

        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle
         */
        float GetTheta() const;
        /**
         * @brief get motor angular velocity, in [rad / s]
         * @return radian angular velocity
         */
        float GetOmega() const;

        /**
         * @brief get motor torque, in [Nm]
         * @return motor torque
         */
        float GetTorque() const;

        mode_t GetMode() const;

        /**
         * @brief get motor target angle, in [rad]
         * @return motor target angle
         */
        float GetRelativeTarget() const;

        /**
         * @brief Set motor target position, in [rad]
         */
        void SetRelativeTarget(float target);

        volatile bool connection_flag_ = false;

    private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
        uint16_t tx_id_actual_;  // actual CAN id

        volatile mode_t mode_;           // current motor mode
        volatile float kp_set_ = 0;      // defined kp value
        volatile float kd_set_ = 0;      // defined kd value
        volatile float vel_set_ = 0;     // defined velocity
        volatile float pos_set_ = 0;     // defined position
        volatile float torque_set_ = 0;  // defined torque

        volatile int16_t raw_pos_ = 0;        // actual position
        volatile int16_t raw_vel_ = 0;        // actual velocity
        volatile int16_t raw_torque_ = 0;     // actual torque
        volatile int16_t raw_motorTemp_ = 0;  // motor temp
        volatile int16_t raw_mosTemp_ = 0;    // MOS temp
        volatile float theta_ = 0;            // actual angular position
        volatile float omega_ = 0;            // actual angular velocity
        volatile float torque_ = 0;           // actual torque
        float relative_target_ = 0;           // target position

  // P control
  constexpr static float KP_MIN = 0;
  constexpr static float KP_MAX = 500;
  // D control
  constexpr static float KD_MIN = 0;
  constexpr static float KD_MAX = 5;
  // position
  float P_MIN = -PI;
  float P_MAX = PI;
  // velocity
  constexpr static float V_MIN = -45;
  constexpr static float V_MAX = 45;
  // torque
  constexpr static float T_MIN = -18;
  constexpr static float T_MAX = 18;
};

} /* namespace control */
