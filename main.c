/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/

#include "include/main.h"
 

int main()
{
 
	int i=0;
	bool stop_walk = false;
	sensor.fall_Down_Flag_ = false;
	sensor.stop_Walk_Flag_ = false;

	balance.initialize(30);
	usleep(1000 * 1000);
	init.initial_system();
	usleep(1000 * 1000);
	IK.initial_inverse_kinematic();
	walkinggait.initialize();

	gettimeofday(&walkinggait.timer_start_, NULL);

	while(1)
	{
		datamodule.load_database();
		if(datamodule.motion_execute_flag_)
			datamodule.motion_execute();
		printf(" ");	
		sensor.load_imu();
		// sensor.load_press_left();
		// sensor.load_press_right();

		sensor.load_sensor_setting();
		sensor.sensor_package_generate();
		walkinggait.load_parameter();
		walkinggait.load_walkdata();
		walkinggait.calculate_point_trajectory();
		
        // printf("%d, %d, %d, %d, %d, %d, %d, %d\n", sensor.sensor_request_, sensor.imu_offset_reset_, sensor.force_state_, sensor.gain_set_, sensor.roll_PID_set_, sensor.pitch_PID_set_, sensor.com_PID_set_, sensor.foot_offset_set_);
		// printf("%f , %f , %f \n", sensor.rpy_[0], sensor.rpy_[1], sensor.rpy_[2]);
		// usleep(1000 * 1000);
		gettimeofday(&walkinggait.timer_end_, NULL);
		walkinggait.timer_dt_ = (double)(1000000.0 * (walkinggait.timer_end_.tv_sec - walkinggait.timer_start_.tv_sec) + (walkinggait.timer_end_.tv_usec - walkinggait.timer_start_.tv_usec));

		balance.get_sensor_value();

		if (balance.two_feet_grounded_ && sensor.fall_Down_Flag_)
		{
			sensor.stop_Walk_Flag_ = true;
		}else
		{
			sensor.stop_Walk_Flag_ = false;
		}

		if((walkinggait.timer_dt_ >= 30000.0) && !sensor.stop_Walk_Flag_)
		{
			walkinggait.walking_timer();
			
			gettimeofday(&walkinggait.timer_start_, NULL);
			// balance.balance_control();
		}

 		printf(" ");		
		if((walkinggait.locus_flag_))
		{
 
			balance.setSupportFoot();
			if(walkinggait.LIPM_flag_)
			{
				balance.balance_control();
				walkinggait.LIPM_flag_ = false;
			}
			locus.get_cpg_with_offset();

			IK.calculate_inverse_kinematic(walkinggait.motion_delay_);
			locus.do_motion();

			walkinggait.locus_flag_ = false;
		}
	}
 
		if(walkinggait.plot_once_ == true)
		{
			balance.saveData();
			IK.saveData();
			balance.resetControlValue();
			walkinggait.plot_once_ = false;
		}
	

	// clean up our memory mapping and exit
	init.Clear_Memory_Mapping();

	return( 0 );
}
