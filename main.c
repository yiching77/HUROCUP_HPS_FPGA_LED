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

	// walkinggait.aa = parameterinfo->X;

	balance.initialize(30);
	usleep(1000 * 1000);
	init.initial_system();
	usleep(1000 * 1000);
	IK.initial_inverse_kinematic();
	walkinggait.initialize();

	gettimeofday(&walkinggait.timer_start_, NULL);

	while(1)
	{
    	// printf(" ");
		// printf("aa = %d", walkinggait.aa);

		// cout << parameterinfo->X << endl;

		// printf("rpy[0]_ = %f, rpy[1]_ = %f, rpy[2]_ = %f\n", sensor.rpy_[0],sensor.rpy_[1],sensor.rpy_[2]);
		datamodule.load_database();
		if(datamodule.motion_execute_flag_)
			datamodule.motion_execute();
		sensor.load_imu();
		// sensor.load_press_left();
		// sensor.load_press_right();

		// HandIK.load_hand_data();
		//printf("hx = %lf hy = %lf hz = %lf\n",HandIK.hand_point_[0],HandIK.hand_point_[1],HandIK.hand_point_[2]);
		// HandIK.Kinetic_Main(HandIK.hand_point_[0],HandIK.hand_point_[1],HandIK.hand_point_[2]);

		// test.Kinetic_Main(inxyz[0], inxyz[1], inxyz[2]);

		// sleep(1);
		// printf(" ");
		// usleep(100 * 100);
		sensor.load_sensor_setting();
		sensor.sensor_package_generate();
		walkinggait.load_parameter();
		walkinggait.load_walkdata();
		walkinggait.calculate_point_trajectory();

		gettimeofday(&walkinggait.timer_end_, NULL);
		walkinggait.timer_dt_ = (double)(1000000.0 * (walkinggait.timer_end_.tv_sec - walkinggait.timer_start_.tv_sec) + (walkinggait.timer_end_.tv_usec - walkinggait.timer_start_.tv_usec));

		balance.get_sensor_value();

		if (balance.two_feet_grounded_ && sensor.fall_Down_Flag_)
		{
			sensor.stop_Walk_Flag_ = true;
		}
		else
		{
			sensor.stop_Walk_Flag_ = false;
		}
		if((walkinggait.timer_dt_ >= 30000.0))// && !sensor.stop_Walk_Flag_)
		{
			walkinggait.walking_timer();

			gettimeofday(&walkinggait.timer_start_, NULL);
			// balance.balance_control();
		}

 		// printf(" ");
		// sleep(1);
		// usleep(100 * 100); 
		if((walkinggait.locus_flag_))
		{
 
			balance.setSupportFoot();
			balance.endPointControl();
			if(walkinggait.LIPM_flag_)
			{
				balance.balance_control();
			}
			locus.get_cpg_with_offset();

			IK.calculate_inverse_kinematic(walkinggait.motion_delay_);
			locus.do_motion();
			walkinggait.LIPM_flag_ = false;

			walkinggait.locus_flag_ = false;
		}
		if(parameterinfo->LCFinishFlag  && parameterinfo->LCBalanceOn)
		{
			i++;
			if(i>290)
			{
				parameterinfo->LCFinishFlag = false;
				parameterinfo->LCBalanceFlag = false;
				balance.saveData();
				IK.saveData();
				i = 0;
			}
			else if(i>200)
			{
				parameterinfo->LCBalanceFlag = true;
			}
			if(i>90)
			{
				balance.setSupportFoot();
				balance.balance_control();
				locus.get_cpg_with_offset();
				IK.calculate_inverse_kinematic(30);
				locus.do_motion();
			}
		}
		else
		{
			parameterinfo->LCFinishFlag = false;
		}

		// printf(" ");
		// usleep(100);
		// sleep(1);
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
