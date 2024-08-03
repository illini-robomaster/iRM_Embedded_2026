#include "arm_translate_task.h"
#include "bsp_can_bridge.h"

static control::MotorCANBase* motor9 = nullptr;
static control::ServoMotorWithLG* base_translate_motor = nullptr;
static int last_check_time_ms = 0;
static int last_motor_pos = 0;

static bool killed = false;
// static bsp::GPIO* out_5v_enable = nullptr;



bool base_translate_align_detect() {
	if(!motor9->connection_flag_){ // if motor is not enabled, then it must not be aligned
		return false;
	}

	// if in the past 500ms, the motor does not move, then it is aligned
	if(HAL_GetTick()-last_check_time_ms > 500){
		float diff =  abs(motor9->GetTheta() - last_motor_pos);
		print("diff: %f\r\n", diff);
		if(diff<0.6){
			return true;
		}
		// only update every 500ms
		last_motor_pos = motor9->GetTheta();
		last_check_time_ms = HAL_GetTick();
	}
	return false;
}


int loop_cnt = 0;
void armTranslateTask(void* arg){
	UNUSED(arg);

	// Calibration / Align
	bool aligned = false;
	print("Starting Calib\r\n");
	osDelay(1000);
	base_translate_motor->SetMaxSpeed(BASE_TRANSLATE_ALIGN_SPEED);
	while(!aligned){
		base_translate_motor->CalcOutput();
		control::MotorCANBase::TransmitOutput(&motor9, 1);
		aligned = base_translate_motor->Calibrate();

		// print data
		loop_cnt++;
		if(loop_cnt >= 100) {
			loop_cnt = 0;
                //   set_cursor(0, 0);
                //   clear_screen();
//                print("aligned: %d .. \r\n", base_translate_motor->Calibrate());
			print("HAL tick: %d \r\n", HAL_GetTick());
			// base_translate_motor->PrintData();
			motor9->PrintData();
		}
		osDelay(2);
	}
	base_translate_motor->ReAlign();
	
	base_translate_motor->TurnRelative(-1, true);
	base_translate_motor->CalcOutput();
	control::MotorCANBase::TransmitOutput(&motor9, 1);
	osDelay(200); // wait to 


	base_translate_motor->SetMaxSpeed(BASE_TRANSLATE_RUN_SPEED);

	// base_translate_motor->TurnRelative(0, true);
	// base_translate_motor->CalcOutput();
	// control::MotorCANBase::TransmitOutput(&motor9, 1);

	print("Calib Done\r\n");	

	// Start normal operation
	while(1) {
		if(killed)	{ // if killed do nothing
			osDelay(100);
			continue;
		}
		loop_cnt++;
		if(loop_cnt >= 100) {
		// set_cursor(0, 0);
		// clear_screen();
		// base_translate_motor->PrintData();

			// motor9->PrintData();
			// base_translate_motor->PrintData();

			loop_cnt = 0;
			// print("sbus ch4: %f \r\n", sbus->ch[3]/660.0/50);

		}
#ifdef SINGLE_BOARD
		base_translate_motor->TurnRelative(sbus->ch[12] / 660.0 /80);
#else
		base_translate_motor->TurnRelative(receive->arm_translate/80);
		// base_translate_motor->TurnRelative(0);

#endif
		base_translate_motor->CalcOutput();
		control::MotorCANBase::TransmitOutput(&motor9, 1);
		osDelay(2);
	}
	
}


// map entire range to one rotation
int TRANSLATE_RATIO = 4; // full translation range is this many rotation
void init_arm_translate() {
	// Init m3508 * 1
	motor9 = new control::Motor3508(can1, BASE_TRANSLATE_ID);

	// enable 5v power for the light gate sensor (on dm board only)
	// out_5v_enable = new bsp::GPIO(OUT_5V_Port, OUT_5V_Pin);
	// out_5v_enable->High();
	control::servoLG_t servoLG_data;
	servoLG_data.motor = motor9;
	servoLG_data.max_speed = BASE_TRANSLATE_RUN_SPEED;
	servoLG_data.max_acceleration = BASE_TRANSLATE_ACCELERATION;
	// TODO make sure the gear ratio is correct
	servoLG_data.transmission_ratio = M3508P19_RATIO /** TRANSLATE_RATIO*/;
    servoLG_data.omega_pid_param = new float[3]{140, 1.2, 25};
	// servoLG_data.omega_pid_param = new float[3]{0, 0, 0};
	servoLG_data.max_iout = 1000;
	servoLG_data.max_out = 13000;
	// TODO measure the calibrate offset for base translate motor
	servoLG_data.calibrate_offset = 0;
	servoLG_data.forward_soft_limit = -1; //25.5
	servoLG_data.reverse_soft_limit = -25.5;
	servoLG_data.align_detect_func = base_translate_align_detect;
	servoLG_data.align_dir_invert = false;

	base_translate_motor = new control::ServoMotorWithLG(servoLG_data);
}

void kill_arm_translate(){
	killed = true;
	RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

	motor9->SetOutput(0);
	control::MotorCANBase::TransmitOutput(&motor9, 1);
}

void revive_arm_translate(){
	killed = false;
}