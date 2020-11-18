/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/

#include "include/main.h"
 
Locus locus;
InverseKinematic IK;
KalmanFilter KF;
Initial init;
Walkinggait walkinggait;
Datamodule datamodule;
SensorDataProcess sensor;
BalanceControl balance;
BalanceControlUsingPDController pdbalance;

#ifdef ZMP   
	ZeroMomentPoint zmp;
#endif

int main()
{
	// int time_cnt = 0;
	// double test_time[100] = {0.0};
	// double time_sum = 0.0;
	int i=0;
	bool stop_walk = false;
	sensor.fall_Down_Flag_ = false;
	sensor.stop_Walk_Flag_ = false;

	balance.initialize(30);
	// int led_test_cnt = 0;
	usleep(1000 * 1000);
	init.initial_system();
	usleep(1000 * 1000);
	IK.initial_inverse_kinematic();

	gettimeofday(&walkinggait.timer_start_, NULL);

	while(1)//time_cnt<100)
	{
		 printf("1");
		datamodule.load_database();
		if(datamodule.motion_execute_flag_)
			datamodule.motion_execute();

		sensor.load_imu();

	#ifdef ZMP   
		sensor.load_press_left();
		sensor.load_press_right();
		zmp.zmp_filter();
	#endif

		sensor.load_sensor_setting();
		sensor.sensor_package_generate();
// printf(" ");
		walkinggait.load_parameter();
		walkinggait.load_walkdata();
		walkinggait.calculate_point_trajectory();
		gettimeofday(&walkinggait.timer_end_, NULL);
		walkinggait.timer_dt_ = (double)(1000000.0 * (walkinggait.timer_end_.tv_sec - walkinggait.timer_start_.tv_sec) + (walkinggait.timer_end_.tv_usec - walkinggait.timer_start_.tv_usec));
// printf(" ");
		// if(!parameterinfo->complan.walking_stop)
		// 	gettimeofday(&tstart, NULL);
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
			balance.balance_control();
		}

// printf(" ");		
		if((walkinggait.locus_flag_))// && !stop_walk)//(walkinggait.locus_flag_)
		{
			// if(led_test_cnt>15)
			// 	led_test_cnt = 0;
			// *(uint32_t *)init.h2p_lw_led_addr = led_test_cnt++;

			// gettimeofday(&tstart, NULL);

			locus.get_cpg_with_offset();

			IK.calculate_inverse_kinematic(walkinggait.motion_delay_);
			locus.do_motion();

			// gettimeofday(&tend, NULL);
			// test_time[time_cnt++] = (1000000 * (tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec));
			
			walkinggait.locus_flag_ = false;
		}
	}
// printf(" ");
	// int i=0;
	// for(i=0; i<100; i++)
	// {
	// 	time_sum += test_time[i];
	// }
	// time_sum /= 100.0;

	// printf("avaerage time is %f microsecond\n", time_sum);

	// gettimeofday(&tstart, NULL);
	// usleep(1000*1000);
	// gettimeofday(&tend, NULL);
	// timeuse = (1000000 * (tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec));

	// printf("1 second is %f microsecond\n", timeuse);

	// clean up our memory mapping and exit
	init.Clear_Memory_Mapping();

	return( 0 );
}
