#include "include/Feedback_Control.h"
/////////////////////////posture///////////////////////
FuzzyController fuzzy;

extern struct Points_Struct Points;
extern Locus locus;
extern SensorDataProcess sensor;
extern Walkinggait walkinggait;

BalanceControl::BalanceControl()
{
	original_ik_point_rz_ = 0.0;
	original_ik_point_lz_ = 0.0;
	last_step_ = StopStep;
	now_step_ = StopStep;
}

BalanceControl::~BalanceControl()
{	}

void BalanceControl::initialize(const int control_cycle_msec)
{
	control_cycle_sec_ = control_cycle_msec * 0.001;

	roll_imu_lpf_.initialize(control_cycle_sec_, 1.0);
	pitch_imu_lpf_.initialize(control_cycle_sec_, 1.0);
	foot_roll_adj_imu_lpf_.initialize(control_cycle_sec_, 1.0);
	foot_pitch_adj_imu_lpf_.initialize(control_cycle_sec_, 1.0);
	x_adj_cog_lpf_.initialize(control_cycle_sec_, 1.0);
	y_adj_cog_lpf_.initialize(control_cycle_sec_, 1.0);

	foot_roll_imu_ctrl_.set_control_cycle_time(control_cycle_sec_);
	foot_pitch_imu_ctrl_.set_control_cycle_time(control_cycle_sec_);
	x_adj_by_cog_ctrl_.set_control_cycle_time(control_cycle_sec_);
	y_adj_by_cog_ctrl_.set_control_cycle_time(control_cycle_sec_);

	desired_robot_to_rf_ = Eigen::VectorXd::Zero(6);
	desired_robot_to_lf_ = Eigen::VectorXd::Zero(6);
	pose_rf_adj_ = Eigen::VectorXd::Zero(6);
	pose_lf_adj_ = Eigen::VectorXd::Zero(6);
	robot_to_rf_modified_ = Eigen::VectorXd::Zero(6);
	robot_to_lf_modified_ = Eigen::VectorXd::Zero(6);

	foot_x_adjustment_abs_max_m_ = 0.2;
	foot_y_adjustment_abs_max_m_ = 0.2;
	foot_z_adjustment_abs_max_m_ = 0.2;
	foot_roll_adjustment_abs_max_rad_  = 5.0*DEGREE2RADIAN;
	foot_pitch_adjustment_abs_max_rad_ = 5.0*DEGREE2RADIAN;
	foot_yaw_adjustment_abs_max_rad_   = 5.0*DEGREE2RADIAN;

	roll_gain_ = 0.04;
	pitch_gain_ = 0.11;
}

void BalanceControl::get_sensor_value()
{
	int i;
	double rpy_radian[3] = {0};
	for(int i=0; i<3; i++)
		rpy_radian[i] = sensor.rpy_[i] * DEGREE2RADIAN;
	
	roll_imu_filtered_ = roll_imu_lpf_.get_filtered_output(rpy_radian[0]);
	pitch_imu_filtered_ = pitch_imu_lpf_.get_filtered_output(rpy_radian[1]);
	roll_over_limit_ = (fabs(roll_imu_filtered_) > 0.209 ? true : false);
	pitch_over_limit_ = (fabs(pitch_imu_filtered_) > 0.297 ? true : false);
	two_feet_grounded_ = (original_ik_point_rz_ == original_ik_point_lz_ ? true : false);

    if( (fabs( rpy_radian[0] ) > RPY_ROLL_LIMIT || ( rpy_radian[1] ) > RPY_PITCH_LIMIT) && sensor.fall_Down_Flag_ == false) //over plus limit >> forward fall down
    {
		if(sensor.fall_Down_Status_ == 'F')
		{
			gettimeofday(&timer_end_, NULL);
			sensor.fall_Down_Status_ = 'F';
		}
		else
		{
			gettimeofday(&timer_start_, NULL);
			sensor.fall_Down_Status_ = 'F';			
		}

    }
	else if(  ( rpy_radian[1] ) < -(RPY_PITCH_LIMIT)  && sensor.fall_Down_Flag_ == false) //over minus limit >> backward fall down
    {
		if(sensor.fall_Down_Status_ == 'B')
		{
			gettimeofday(&timer_end_, NULL);
			sensor.fall_Down_Status_ = 'B';
		}
		else
		{
			gettimeofday(&timer_start_, NULL);
			sensor.fall_Down_Status_ = 'B';			
		}
    }
	else if( (fabs( rpy_radian[0] ) < RPY_STAND_RANGE && fabs( rpy_radian[1] ) < RPY_STAND_RANGE) && sensor.fall_Down_Flag_ == true) 
    {
		if(sensor.fall_Down_Status_ == 'S')
		{
			gettimeofday(&timer_end_, NULL);
			sensor.fall_Down_Status_ = 'S';
		}
		else
		{
			gettimeofday(&timer_start_, NULL);
			sensor.fall_Down_Status_ = 'S';			
		}	
    }
	else
	{
		gettimeofday(&timer_start_, NULL);
		sensor.fall_Down_Status_ = sensor.fall_Down_Status_ ;
	}
	
	timer_dt_ = (double)(1000000.0 * (timer_end_.tv_sec - timer_start_.tv_sec) + (timer_end_.tv_usec - timer_start_.tv_usec));	

	if(timer_dt_ >= 2000000.0 && (sensor.fall_Down_Status_ == 'F' || sensor.fall_Down_Status_ == 'B') )//timer_dt_ unit is us 1sec = 1000000
	{
		sensor.fall_Down_Flag_ = true;
	}
	else if(timer_dt_ >= 4000000.0 && sensor.fall_Down_Status_ == 'S')
	{
		sensor.fall_Down_Flag_ = false;
	}
	else
	{
		sensor.fall_Down_Flag_ = sensor.fall_Down_Flag_;		
	}

}

void BalanceControl::p2h_get_gain()
{
	roll_gain_ = sensor.PD_Balance_Roll_Gain_;
	pitch_gain_ = sensor.PD_Balance_Pitch_Gain_;
}

void BalanceControl::balance_control()
{
	int i;
	LinearAlgebra LA;
	double rpy_radian[3] = {0};
	for(i=0; i<3; i++)
		rpy_radian[i] = sensor.rpy_[i] * DEGREE2RADIAN;

	cog_roll_offset_ = sensor.imu_desire_[0] * DEGREE2RADIAN;
	cog_pitch_offset_ = sensor.imu_desire_[1] * DEGREE2RADIAN;
	foot_roll_imu_ctrl_.set_desired(sensor.imu_desire_[0] * DEGREE2RADIAN);
	foot_pitch_imu_ctrl_.set_desired(sensor.imu_desire_[1] * DEGREE2RADIAN);
	x_adj_by_cog_ctrl_.set_desired(0);
	y_adj_by_cog_ctrl_.set_desired(0);
	
	original_ik_point_rz_ = parameterinfo->points.IK_Point_RZ;
	original_ik_point_lz_ = parameterinfo->points.IK_Point_LZ;
	
	desired_robot_to_rf_.coeffRef(0) = parameterinfo->points.IK_Point_RX;
	desired_robot_to_rf_.coeffRef(1) = parameterinfo->points.IK_Point_RY;
	desired_robot_to_rf_.coeffRef(2) = parameterinfo->points.IK_Point_RZ;
	desired_robot_to_lf_.coeffRef(0) = parameterinfo->points.IK_Point_LX;
	desired_robot_to_lf_.coeffRef(1) = parameterinfo->points.IK_Point_LY;
	desired_robot_to_lf_.coeffRef(2) = parameterinfo->points.IK_Point_LZ;

	double roll_imu_filtered_ = roll_imu_lpf_.get_filtered_output(rpy_radian[0]);
	double pitch_imu_filtered_ = pitch_imu_lpf_.get_filtered_output(rpy_radian[1]);
	// roll_over_limit_ = (fabs(roll_imu_filtered_) > 12.9 ? true : false);
	// pitch_over_limit_ = (fabs(pitch_imu_filtered_) > 17.7 ? true : false);
	// double cog_y_filtered = roll_imu_filtered_ - cog_roll_offset_;
	// double cog_x_filtered = pitch_imu_filtered_ - cog_pitch_offset_;
	// printf("------------------------------------------------------------------\n");

	// if(Points.Inverse_PointR_Z != Points.Inverse_PointL_Z)
	// {
	//	 foot_cog_x_ = 26.2 * sin(cog_x_filtered);
	//	 foot_cog_y_ = 26.2 * sin(cog_y_filtered);
	// }
	// else
	// {
	// 	foot_cog_x_ = 0;
	// 	foot_cog_y_ = 0;
	// }
	ankle_pitch_ = atan2(7.462*sin(roll_imu_filtered_), 7.462*cos(roll_imu_filtered_)+26.2);
	 
	if(sensor.gain_set_state_)
	{
		p2h_get_gain();
		foot_roll_imu_ctrl_.p2h_get_PD_gain();
		foot_pitch_imu_ctrl_.p2h_get_PD_gain();
		sensor.gain_set_state_ = false;
	}
	// printf("%f\n",sensor.PD_Balance_Roll_Gain_);
	// printf("%f\n",sensor.PD_Balance_Pitch_Gain_);
	// printf("%f\n",sensor.PD_Balance_Kp_);
	// printf("%f\n",sensor.PD_Balance_Kd_);
	//origin 0.1 & 0.1 
	//for all strategy  0.04
	foot_roll_adj_by_imu_roll_ = roll_gain_ * (double)(walkinggait.sensor_mode_ & 0x01) * foot_roll_imu_ctrl_.get_feedback(roll_imu_filtered_);
	//pitch for WL  0.12 , 0.125  , for other strategy  0.105 ~ 0.11  	
	foot_pitch_adj_by_imu_pitch_ = pitch_gain_ * (double)(walkinggait.sensor_mode_ & 0x02) * foot_pitch_imu_ctrl_.get_feedback(pitch_imu_filtered_);
	// x_adj_by_cog_ =  0.2 * (double)(walkinggait.sensor_mode_ & 0x02) * x_adj_by_cog_ctrl_.get_feedback(foot_cog_x_);
	// y_adj_by_cog_ =  0.2 * (double)(walkinggait.sensor_mode_ & 0x01) * y_adj_by_cog_ctrl_.get_feedback(foot_cog_y_);

	Eigen::MatrixXd mat_orientation_adj_by_imu = LA.getRotation4d(foot_roll_adj_by_imu_roll_, foot_pitch_adj_by_imu_pitch_, 0.0);
	Eigen::MatrixXd mat_r_xy, mat_l_xy;
	mat_r_xy.resize(4, 1);
	mat_r_xy.coeffRef(0, 0) = desired_robot_to_rf_.coeff(0) - 0.5 * (desired_robot_to_rf_.coeff(0) + desired_robot_to_lf_.coeff(0));
	mat_r_xy.coeffRef(1, 0) = desired_robot_to_rf_.coeff(1) - 0.5 * (desired_robot_to_rf_.coeff(1) + desired_robot_to_lf_.coeff(1));
	mat_r_xy.coeffRef(2, 0) = 0.0;
	mat_r_xy.coeffRef(3, 0) = 1;

	mat_l_xy.resize(4, 1);
	mat_l_xy.coeffRef(0, 0) = desired_robot_to_lf_.coeff(0) - 0.5 * (desired_robot_to_rf_.coeff(0) + desired_robot_to_lf_.coeff(0));
	mat_l_xy.coeffRef(1, 0) = desired_robot_to_lf_.coeff(1) - 0.5 * (desired_robot_to_rf_.coeff(1) + desired_robot_to_lf_.coeff(1));
	mat_l_xy.coeffRef(2, 0) = 0.0;
	mat_l_xy.coeffRef(3, 0) = 1;

	mat_r_xy = mat_orientation_adj_by_imu * mat_r_xy;
	mat_l_xy = mat_orientation_adj_by_imu * mat_l_xy;

	pose_rf_adj_.coeffRef(0) = 0;
	pose_rf_adj_.coeffRef(1) = 0;
	pose_rf_adj_.coeffRef(2) = mat_r_xy.coeff(2, 0);
	pose_rf_adj_.coeffRef(3) = foot_roll_adj_by_imu_roll_;
	pose_rf_adj_.coeffRef(4) = foot_pitch_adj_by_imu_pitch_;

	pose_lf_adj_.coeffRef(0) = 0;
	pose_lf_adj_.coeffRef(1) = 0;
	pose_lf_adj_.coeffRef(2) = mat_l_xy.coeff(2, 0);
	pose_lf_adj_.coeffRef(3) = foot_roll_adj_by_imu_roll_;
	pose_lf_adj_.coeffRef(4) = foot_pitch_adj_by_imu_pitch_;

	// cout << pose_lf_adj_ << endl;

	// pose_rf_adj_.coeffRef(0) = copysign(fmin(fabs(pose_rf_adj_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_rf_adj_.coeff(0));
	// pose_rf_adj_.coeffRef(1) = copysign(fmin(fabs(pose_rf_adj_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_rf_adj_.coeff(1));
	// pose_rf_adj_.coeffRef(2) = copysign(fmin(fabs(pose_rf_adj_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_rf_adj_.coeff(2));
	// pose_rf_adj_.coeffRef(3) = copysign(fmin(fabs(pose_rf_adj_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_rf_adj_.coeff(3));
	// pose_rf_adj_.coeffRef(4) = copysign(fmin(fabs(pose_rf_adj_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_rf_adj_.coeff(4));
	// pose_rf_adj_.coeffRef(5) = 0;

	// pose_lf_adj_.coeffRef(0) = copysign(fmin(fabs(pose_lf_adj_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_lf_adj_.coeff(0));
	// pose_lf_adj_.coeffRef(1) = copysign(fmin(fabs(pose_lf_adj_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_lf_adj_.coeff(1));
	// pose_lf_adj_.coeffRef(2) = copysign(fmin(fabs(pose_lf_adj_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_lf_adj_.coeff(2));
	// pose_lf_adj_.coeffRef(3) = copysign(fmin(fabs(pose_lf_adj_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_lf_adj_.coeff(3));
	// pose_lf_adj_.coeffRef(4) = copysign(fmin(fabs(pose_lf_adj_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_lf_adj_.coeff(4));
	// pose_lf_adj_.coeffRef(5) = 0;

	for(i=0; i<6; i++)
	{
		robot_to_rf_modified_.coeffRef(i) = desired_robot_to_rf_.coeff(i) + pose_rf_adj_.coeff(i);
		robot_to_lf_modified_.coeffRef(i) = desired_robot_to_lf_.coeff(i) + pose_lf_adj_.coeff(i);
	}

	// parameterinfo->points.IK_Point_RX = copysign((fabs(robot_to_rf_modified_.coeff(0)) + x_adj_by_cog_), robot_to_rf_modified_.coeff(0));
	// parameterinfo->points.IK_Point_RY = robot_to_rf_modified_.coeff(1) + y_adj_by_cog_;//copysign((fabs(robot_to_rf_modified_.coeff(1)) - y_adj_by_cog_), robot_to_rf_modified_.coeff(1));
	// parameterinfo->points.IK_Point_RZ = robot_to_rf_modified_.coeff(2);
	// parameterinfo->points.IK_Point_LX = copysign((fabs(robot_to_lf_modified_.coeff(0)) + x_adj_by_cog_), robot_to_lf_modified_.coeff(0));
	// parameterinfo->points.IK_Point_LY = robot_to_lf_modified_.coeff(1) + y_adj_by_cog_;//copysign((fabs(robot_to_lf_modified_.coeff(1)) - y_adj_by_cog_), robot_to_lf_modified_.coeff(1));
	// parameterinfo->points.IK_Point_LZ = robot_to_lf_modified_.coeff(2);
	parameterinfo->points.IK_Point_RX = robot_to_rf_modified_.coeff(0);
	parameterinfo->points.IK_Point_RY = robot_to_rf_modified_.coeff(1);
	parameterinfo->points.IK_Point_RZ = robot_to_rf_modified_.coeff(2);
	parameterinfo->points.IK_Point_LX = robot_to_lf_modified_.coeff(0);
	parameterinfo->points.IK_Point_LY = robot_to_lf_modified_.coeff(1);
	parameterinfo->points.IK_Point_LZ = robot_to_lf_modified_.coeff(2);

	fb_x_r.push_back(pose_rf_adj_.coeff(0));
	fb_x_l.push_back(pose_lf_adj_.coeff(0));
	fb_y_r.push_back(pose_rf_adj_.coeff(1));
	fb_y_l.push_back(pose_lf_adj_.coeff(1));
	fb_z_r.push_back(pose_rf_adj_.coeff(2));
	fb_z_l.push_back(pose_lf_adj_.coeff(2));
	// fb_x_r.push_back(parameterinfo->points.IK_Point_RX);
	// fb_x_l.push_back(parameterinfo->points.IK_Point_LX);
	// fb_y_r.push_back(parameterinfo->points.IK_Point_RY);
	// fb_y_l.push_back(parameterinfo->points.IK_Point_LY);
	// fb_z_r.push_back(parameterinfo->points.IK_Point_RZ);
	// fb_z_l.push_back(parameterinfo->points.IK_Point_LZ);
	fb_cog_x.push_back(foot_cog_x_);
	fb_cog_y.push_back(foot_cog_y_);
	now_step_ = parameterinfo->complan.walking_state;
	// if(sensor.fs_offset_reset_)//now_step_ == StopStep && last_step_ != StopStep)//
	// {
	// 	SaveData();
	// 	sensor.fs_offset_reset_ = false;
	// }
	last_step_ = now_step_;
}

void BalanceControl::control_after_ik_calculation()
{
	// if(Points.Inverse_PointR_Z != Points.Inverse_PointL_Z)
	if(parameterinfo->complan.walking_state != StopStep)
	{
		// roll
		// if(roll_over_limit_)
		{
			Points.Thta[10] -= robot_to_lf_modified_.coeff(3);
			Points.Thta[16] -= robot_to_rf_modified_.coeff(3);
			Points.Thta[14] += robot_to_lf_modified_.coeff(3);
			Points.Thta[20] += robot_to_rf_modified_.coeff(3);
		}
		// pitch
		// if(pitch_over_limit_)
		{
			Points.Thta[11] -= robot_to_lf_modified_.coeff(4);
			Points.Thta[17] -= robot_to_rf_modified_.coeff(4);
			// Points.Thta[13] -= robot_to_lf_modified_.coeff(4);
			// Points.Thta[19] -= robot_to_rf_modified_.coeff(4);
		}

		// LR hand swing
		// Points.Thta[1] = (M_PI / 2) + 2*robot_to_lf_modified_.coeff(3);
		// if(Points.Thta[1] > (M_PI / 2))
		// 	Points.Thta[1] = M_PI / 2;
		// Points.Thta[5] = (M_PI / 2) + 2*robot_to_rf_modified_.coeff(3);
		// if(Points.Thta[5] < (M_PI / 2))
		// 	Points.Thta[5] = M_PI / 2;
	}

	// switch(landing_foot_)
	// {
	// case etRightfoot:
	// 	Points.Thta[19] -= ankle_adj_by_cog_pitch_;
	// 	Points.Thta[20] -= ankle_adj_by_cog_roll_;
	// 	break;
	// case etLeftfoot:
	// 	Points.Thta[13] -= ankle_adj_by_cog_pitch_;
	// 	Points.Thta[14] -= ankle_adj_by_cog_roll_;
	// 	break;
	// case etBothfoot:
		
	// 	break;	
	// }

	// compensate
	if(Points.Inverse_PointR_Y < 0 && (original_ik_point_rz_ != original_ik_point_lz_))
	{
		double tmp = (Points.Thta[10] * 1.05) - Points.Thta[10];
		Points.Thta[10] -= tmp;
		Points.Thta[16] *= 1.025;
	}
	else if(Points.Inverse_PointR_Y > 0 && (original_ik_point_rz_ != original_ik_point_lz_))
	{
		double tmp = (Points.Thta[10] * 1.05) - Points.Thta[10];
		Points.Thta[10] -= tmp;
		Points.Thta[16] *= 1.025;
	}
}

double BalanceControl::pid_control(double error, double error_integral, double error_dot, double Kp, double Ki, double Kd)
{
	double control_value = 0.0;

	control_value = Kp * error + Ki * error_integral + Kd * error_dot;

	return (control_value * Angle_2_PI);
}

BalanceLowPassFilter::BalanceLowPassFilter()
{
	cut_off_freq_ = 1.0;
	control_cycle_sec_ = 0.008;
	prev_output_ = 0;

	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
}

BalanceLowPassFilter::~BalanceLowPassFilter()
{	}

void BalanceLowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;
	control_cycle_sec_ = control_cycle_sec;
	prev_output_ = 0;

	if(cut_off_frequency > 0)
		alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
		alpha_ = 1;
}

void BalanceLowPassFilter::set_cut_off_frequency(double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;

	if(cut_off_frequency > 0)
	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
	alpha_ = 1;
}

double BalanceLowPassFilter::get_cut_off_frequency(void)
{
	return cut_off_freq_;
}

double BalanceLowPassFilter::get_filtered_output(double present_raw_value)
{
	prev_output_ = alpha_*present_raw_value + (1.0 - alpha_)*prev_output_;
	return prev_output_;
}

BalancePDController::BalancePDController()
{
	desired_ = 0;
	curr_err_ = 0;
	prev_err_ = 0;

	//test

	//2020 0701 E305 WL  0.014
	//big 0.016  for other strategy 0.020  SP 0.019
	p_gain_ = 1;
	d_gain_ = 0.008;
}

BalancePDController::~BalancePDController()
{	}

void BalancePDController::set_desired(double desired)
{
	desired_ = desired;
}

void BalancePDController::set_control_cycle_time(double control_cycle_sec)
{
	control_cycle_sec_ = control_cycle_sec;
}

void BalancePDController::p2h_get_PD_gain()
{
	p_gain_ = sensor.PD_Balance_Kp_;
	d_gain_ = sensor.PD_Balance_Kd_;
}

double BalancePDController::get_feedback(double present_sensor_output)
{
	prev_err_ = curr_err_;
	curr_err_ = desired_ - present_sensor_output;

	return (p_gain_*curr_err_ + d_gain_*(curr_err_ - prev_err_)/control_cycle_sec_);
}

BalanceControlUsingPDController::BalanceControlUsingPDController()
{
	control_cycle_sec_ = 0.008;

	// balance enable
	roll_control_enable_ = 1.0;
	pitch_control_enable_ = 1.0;

	// desired pose
	desired_robot_to_cob_ = Eigen::MatrixXd::Identity(4, 4);
	desired_robot_to_right_foot_ = Eigen::MatrixXd::Identity(4, 4);
	desired_robot_to_left_foot_ = Eigen::MatrixXd::Identity(4, 4);

	// sensed values
	current_imu_roll_rad_per_sec_ = 0;
	current_imu_pitch_rad_per_sec_ = 0;

	// manual cob adjustment
	cob_x_manual_adjustment_m_ = 0;
	cob_y_manual_adjustment_m_ = 0;
	cob_z_manual_adjustment_m_ = 0;

	// balance algorithm result
	foot_roll_adjustment_by_imu_roll_ = 0;
	foot_pitch_adjustment_by_imu_pitch_ = 0;

	// maximum adjustment
	cob_x_adjustment_abs_max_m_ = 0.05;
	cob_y_adjustment_abs_max_m_ = 0.05;
	cob_z_adjustment_abs_max_m_ = 0.05;
	cob_roll_adjustment_abs_max_rad_  = 30.0*DEGREE2RADIAN;
	cob_pitch_adjustment_abs_max_rad_ = 30.0*DEGREE2RADIAN;
	cob_yaw_adjustment_abs_max_rad_   = 30.0*DEGREE2RADIAN;
	foot_x_adjustment_abs_max_m_ = 0.1;
	foot_y_adjustment_abs_max_m_ = 0.1;
	foot_z_adjustment_abs_max_m_ = 0.1;
	foot_roll_adjustment_abs_max_rad_  = 30.0*DEGREE2RADIAN;
	foot_pitch_adjustment_abs_max_rad_ = 30.0*DEGREE2RADIAN;
	foot_yaw_adjustment_abs_max_rad_   = 30.0*DEGREE2RADIAN;

	mat_robot_to_cob_modified_        = Eigen::MatrixXd::Identity(4,4);
	mat_robot_to_right_foot_modified_ = Eigen::MatrixXd::Identity(4,4);
	mat_robot_to_left_foot_modified_  = Eigen::MatrixXd::Identity(4,4);
	pose_cob_adjustment_         = Eigen::VectorXd::Zero(6);
	pose_right_foot_adjustment_  = Eigen::VectorXd::Zero(6);;
	pose_left_foot_adjustment_   = Eigen::VectorXd::Zero(6);;
}

BalanceControlUsingPDController::~BalanceControlUsingPDController()
{	}

void BalanceControlUsingPDController::initialize(const int control_cycle_msec)
{
	control_cycle_sec_ = control_cycle_msec * 0.001;

	pose_cob_adjustment_.fill(0);
	pose_right_foot_adjustment_.fill(0);
 	pose_left_foot_adjustment_.fill(0);

	roll_imu_lpf_.initialize(control_cycle_sec_, 1.0);
	pitch_imu_lpf_.initialize(control_cycle_sec_, 1.0);
}

void BalanceControlUsingPDController::set_roll_control_enable(bool enable)
{
	if(enable)
		roll_control_enable_ = 1.0;
	else
		roll_control_enable_ = 0.0;
}

void BalanceControlUsingPDController::set_pitch_control_enable(bool enable)
{
	if(enable)
		pitch_control_enable_ = 1.0;
	else
		pitch_control_enable_ = 0.0;
}

void BalanceControlUsingPDController::process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified)
{
	LinearAlgebra LA;

	pose_cob_adjustment_.fill(0);
	pose_right_foot_adjustment_.fill(0);
	pose_left_foot_adjustment_.fill(0);

	double roll_imu_filtered = roll_imu_lpf_.get_filtered_output(current_imu_roll_rad_per_sec_);
	double pitch_imu_filtered = pitch_imu_lpf_.get_filtered_output(current_imu_pitch_rad_per_sec_);

	foot_roll_adjustment_by_imu_roll_ = -0.1*roll_control_enable_*foot_roll_imu_ctrl_.get_feedback(roll_imu_filtered);
	foot_pitch_adjustment_by_imu_pitch_ = -0.1*pitch_control_enable_*foot_pitch_imu_ctrl_.get_feedback(pitch_imu_filtered);

	Eigen::MatrixXd mat_orientation_adjustment_by_imu = LA.getRotation4d(foot_roll_adjustment_by_imu_roll_, foot_pitch_adjustment_by_imu_pitch_, 0.0);
	Eigen::MatrixXd mat_r_xy, mat_l_xy;
	mat_r_xy.resize(4, 1);
	mat_r_xy.coeffRef(0, 0) = desired_robot_to_right_foot_.coeff(0, 3) - 0.5*(desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
	mat_r_xy.coeffRef(1, 0) = desired_robot_to_right_foot_.coeff(1, 3) - 0.5*(desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
	mat_r_xy.coeffRef(2, 0) = 0.0;
	mat_r_xy.coeffRef(3, 0) = 1;

	mat_l_xy.resize(4, 1);
	mat_l_xy.coeffRef(0, 0) = desired_robot_to_left_foot_.coeff(0, 3) - 0.5*(desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
	mat_l_xy.coeffRef(1, 0) = desired_robot_to_left_foot_.coeff(1, 3) - 0.5*(desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
	mat_l_xy.coeffRef(2, 0) = 0.0;
	mat_l_xy.coeffRef(3, 0) = 1;

	mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
	mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;

	// sum of sensory balance result
	pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
	pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
	pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

	pose_right_foot_adjustment_.coeffRef(0) = 0;
	pose_right_foot_adjustment_.coeffRef(1) = 0;
	pose_right_foot_adjustment_.coeffRef(2) = mat_r_xy.coeff(2, 0);
	pose_right_foot_adjustment_.coeffRef(3) = foot_roll_adjustment_by_imu_roll_;
	pose_right_foot_adjustment_.coeffRef(4) = foot_pitch_adjustment_by_imu_pitch_;

	pose_left_foot_adjustment_.coeffRef(0) = 0;
	pose_left_foot_adjustment_.coeffRef(1) = 0;
	pose_left_foot_adjustment_.coeffRef(2) = mat_l_xy.coeff(2, 0);
	pose_left_foot_adjustment_.coeffRef(3) = foot_roll_adjustment_by_imu_roll_;
	pose_left_foot_adjustment_.coeffRef(4) = foot_pitch_adjustment_by_imu_pitch_;

	// check limitation
	pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
	pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
	pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
	pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
	pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
	pose_cob_adjustment_.coeffRef(5) = 0;

	pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
	pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
	pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
	pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
	pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
	pose_right_foot_adjustment_.coeffRef(5) = 0;

	pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
	pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
	pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
	pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
	pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
	pose_left_foot_adjustment_.coeffRef(5) = 0;

	Eigen::MatrixXd cob_rotation_adj = LA.getRotationZ(pose_cob_adjustment_.coeff(5)) * LA.getRotationY(pose_cob_adjustment_.coeff(4)) * LA.getRotationX(pose_cob_adjustment_.coeff(3));
	Eigen::MatrixXd rf_rotation_adj = LA.getRotationZ(pose_right_foot_adjustment_.coeff(5)) * LA.getRotationY(pose_right_foot_adjustment_.coeff(4)) * LA.getRotationX(pose_right_foot_adjustment_.coeff(3));
	Eigen::MatrixXd lf_rotation_adj = LA.getRotationZ(pose_left_foot_adjustment_.coeff(5)) * LA.getRotationY(pose_left_foot_adjustment_.coeff(4)) * LA.getRotationX(pose_left_foot_adjustment_.coeff(3));
	mat_robot_to_cob_modified_.block<3,3>(0,0) = cob_rotation_adj * desired_robot_to_cob_.block<3,3>(0,0);
	mat_robot_to_right_foot_modified_.block<3,3>(0,0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3,3>(0,0);;
	mat_robot_to_left_foot_modified_.block<3,3>(0,0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3,3>(0,0);;

	mat_robot_to_cob_modified_.coeffRef(0,3) = desired_robot_to_cob_.coeff(0,3) + pose_cob_adjustment_.coeff(0);
	mat_robot_to_cob_modified_.coeffRef(1,3) = desired_robot_to_cob_.coeff(1,3) + pose_cob_adjustment_.coeff(1);
	mat_robot_to_cob_modified_.coeffRef(2,3) = desired_robot_to_cob_.coeff(2,3) + pose_cob_adjustment_.coeff(2);

	mat_robot_to_right_foot_modified_.coeffRef(0,3) = desired_robot_to_right_foot_.coeff(0,3) + pose_right_foot_adjustment_.coeff(0);
	mat_robot_to_right_foot_modified_.coeffRef(1,3) = desired_robot_to_right_foot_.coeff(1,3) + pose_right_foot_adjustment_.coeff(1);
	mat_robot_to_right_foot_modified_.coeffRef(2,3) = desired_robot_to_right_foot_.coeff(2,3) + pose_right_foot_adjustment_.coeff(2);

	mat_robot_to_left_foot_modified_.coeffRef(0,3) = desired_robot_to_left_foot_.coeff(0,3) + pose_left_foot_adjustment_.coeff(0);
	mat_robot_to_left_foot_modified_.coeffRef(1,3) = desired_robot_to_left_foot_.coeff(1,3) + pose_left_foot_adjustment_.coeff(1);
	mat_robot_to_left_foot_modified_.coeffRef(2,3) = desired_robot_to_left_foot_.coeff(2,3) + pose_left_foot_adjustment_.coeff(2);

	*robot_to_cob_modified        = mat_robot_to_cob_modified_;
	*robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
	*robot_to_left_foot_modified  = mat_robot_to_left_foot_modified_;
}

void BalanceControlUsingPDController::set_desired_pose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot)
{
	desired_robot_to_cob_        = robot_to_cob;
	desired_robot_to_right_foot_ = robot_to_right_foot;
	desired_robot_to_left_foot_  = robot_to_left_foot;
}

void BalanceControlUsingPDController::set_desired_cob_imu(double imu_roll, double imu_pitch)
{
	foot_roll_imu_ctrl_.desired_  = imu_roll;
	foot_pitch_imu_ctrl_.desired_ = imu_pitch;
}

void BalanceControlUsingPDController::set_current_imu_sensor_output(double imu_roll, double imu_pitch)
{
	current_imu_roll_rad_per_sec_  = imu_roll;
	current_imu_pitch_rad_per_sec_ = imu_pitch;
}

void BalanceControlUsingPDController::set_maximum_adjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                                          double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                          double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                          double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
	cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
	cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
	cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
	cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
	cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
	cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;
	foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
	foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
	foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
	foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
	foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
	foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

void BalanceControlUsingPDController::set_cob_manual_adjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

double BalanceControlUsingPDController::get_cob_manual_adjustment_x()
{
  return cob_x_manual_adjustment_m_;
}

double BalanceControlUsingPDController::get_cob_manual_adjustment_y()
{
  return cob_y_manual_adjustment_m_;
}

double BalanceControlUsingPDController::get_cob_manual_adjustment_z()
{
  return cob_z_manual_adjustment_m_;
}

LinearAlgebra::LinearAlgebra()
{	}

LinearAlgebra::~LinearAlgebra()
{	}

Eigen::Matrix3d LinearAlgebra::getRotationX(double angle)
{
	Eigen::Matrix3d rotation(3,3);

	rotation <<
		1.0, 0.0, 0.0,
		0.0, cos(angle), -sin(angle),
		0.0, sin(angle), cos(angle);

	return rotation;
}

Eigen::Matrix3d LinearAlgebra::getRotationY(double angle)
{
	Eigen::Matrix3d rotation(3,3);

	rotation <<
		cos(angle), 0.0, sin(angle),
		0.0, 1.0, 0.0,
		-sin(angle), 0.0, cos(angle);

	return rotation;
}

Eigen::Matrix3d LinearAlgebra::getRotationZ(double angle)
{
	Eigen::Matrix3d rotation(3,3);

	rotation <<
		cos(angle), -sin(angle), 0.0,
		sin(angle), cos(angle), 0.0,
		0.0, 0.0, 1.0;

	return rotation;
}

Eigen::Matrix4d LinearAlgebra::getRotation4d(double roll, double pitch, double yaw )
{
	double sr = sin(roll), cr = cos(roll);
	double sp = sin(pitch), cp = cos(pitch);
	double sy = sin(yaw), cy = cos(yaw);

	Eigen::Matrix4d mat_roll;
	Eigen::Matrix4d mat_pitch;
	Eigen::Matrix4d mat_yaw;

	mat_roll <<
		1, 0, 0, 0,
		0, cr, -sr, 0,
		0, sr, cr, 0,
		0, 0, 0, 1;

	mat_pitch <<
		cp, 0, sp, 0,
		0, 1, 0, 0,
		-sp, 0, cp, 0,
		0, 0, 0, 1;

	mat_yaw <<
		cy, -sy, 0, 0,
		sy, cy, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::Matrix4d mat_rpy = (mat_yaw*mat_pitch)*mat_roll;

	return mat_rpy;
}

string BalanceControl::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void BalanceControl::SaveData()
{
    string savedText = "R_move_X\tL_move_X\t"
                       "R_move_Y\tL_move_Y\t"
                       "R_move_Z\tL_move_Z\t"
                       "COG_X\tCOG_Y\n";
    char path[200] = "/data";
    strcat(path, "/Feedback_Control_Record.xls");

    fstream fp;
    fp.open(path, ios::out);

    fp<<savedText;

    for(int i = 0; i < fb_x_r.size(); i++)
    {
        savedText = DtoS(fb_x_r[i]) + "\t"
                + DtoS(fb_x_l[i]) + "\t"
                + DtoS(fb_y_r[i]) + "\t"
                + DtoS(fb_y_l[i]) + "\t"
                + DtoS(fb_z_r[i]) + "\t"
                + DtoS(fb_z_l[i]) + "\t"
                + DtoS(fb_cog_x[i]) + "\t"
                + DtoS(fb_cog_y[i]) + "\n";
        fp<<savedText;
    }
    // fb_x_r.clear();
    // fb_x_l.clear();
    // fb_y_r.clear();
    // fb_y_l.clear();
    // fb_z_r.clear();
    // fb_z_l.clear();
    // fb_cog_x.clear();
    // fb_cog_y.clear();
    fp.close();
}

ZeroMomentPoint::ZeroMomentPoint()
{
	printf("yee");
}


ZeroMomentPoint::~ZeroMomentPoint()
{

}


void ZeroMomentPoint::zmp_offset_reset()
{


	/*if(Zmp_offset_load_flag_)
	{
		for(int kg_count = 0 ; kg_count < 5 ; kg_count++)
		{
			for(int press_number_count = 0 ; press_number_count<8 ; press_number_count++)
			{
				Zmp_kg_offset_[kg_count][press_number_count] = Zmp_kg_offset_load_[kg_count][press_number_count];
			} 
		}
	}
	else
	{*/
		// for(int press_number_count = 0 ; press_number_count < 4 ; press_number_count++)
		// {
		// 	for(int kg_count = 0 ; kg_count<5 ; kg_count++)
		// 	{
		// 		Zmp_kg_offset_[press_number_count][kg_count] = Zmp_kg_offset_default_[press_number_count][kg_count];
		// 	} 
		// }
	//}
	
}

//
//    |--------------------|	|--------------------|
//    |P0     (front)    P1|	|P4     (front)    P5|
//    |                    |	|                    |
//    |                    |	|                    |
//    |                    |	|                    |
//    |                    |	|                    |
//    |             (right)|	|             (right)|
//    |                    |	|                    |
//    |       left         |	|       right        |
//    |                    |	|                    |
//    |                    |	|                    |
//    |P2                P3|	|P6                P7|
//    |--------------------|	|--------------------|
			


void ZeroMomentPoint::zmp_filter()
{
	ZMP_L_X_ 	= 0;	//left foot zmp point X
	ZMP_L_Y_ 	= 0;	//left foot zmp point Y
	ZMP_R_X_ 	= 0;	//right foot zmp point X
	ZMP_R_Y_	= 0;	//right foot zmp point Y
	ZMP_X_			= 0;	//robot zmp point X
	ZMP_Y_			= 0;	//robot zmp point Y
	int filter_count = 0;


	//zmp_offset_reset();
	int Zmp_kg_offset_[8][5] = 	{	
									{21,30,53,79,130},
									{0,28,58,84,139},
									{0,29,55,79,114},
									{0,8,29,56,116},
									{0,29,58,85,139},
									{0,27,55,78,133},
									{0,27,55,82,133},
									{0,28,56,84,135}
								};


	int		Zmp_kg_table[5] = {0,1,2,3,5} ;

	// if(Zmp_offset_reset_flag_)
	// {
	// 	zmp_offset_reset();
	// }else
	// {
	// 	//nothing
	// }

	//left foot press value translate to kg
	
	for(int press_count = 0;press_count < 4;press_count++)
	{
		if(sensor.press_left_[press_count]<0)
		{
			sensor.press_left_[press_count] = 0;
		}
		ZMP_S_L_[press_count] = 0;
		for(int kg_count = 0 ; kg_count < 4;kg_count++)
		{
			if ((sensor.press_left_[press_count] >= Zmp_kg_offset_[press_count][kg_count]) && (sensor.press_left_[press_count] <= Zmp_kg_offset_[press_count][kg_count+1]))
			{
				if((Zmp_kg_offset_[press_count][kg_count+1] - Zmp_kg_offset_[press_count][kg_count])!=0)
				{
					ZMP_S_L_[press_count] = (double)Zmp_kg_table[kg_count] + (double)((Zmp_kg_table[kg_count+1]-Zmp_kg_table[kg_count])*((double)(sensor.press_left_[press_count] - Zmp_kg_offset_[press_count][kg_count])/(double)(Zmp_kg_offset_[press_count][kg_count+1] - Zmp_kg_offset_[press_count][kg_count])));
					break;
				}
			}
			
		}
	}


	//right foot press value translate to kg

	for(int press_count = 0;press_count < 4;press_count++)
	{
		if(sensor.press_right_[press_count]<0)
		{
			sensor.press_right_[press_count] = 0;
		}
		ZMP_S_R_[press_count] = 0;
		for(int kg_count = 0 ; kg_count < 4;kg_count++)
		{
			if ((sensor.press_right_[press_count] >= Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count]) && (sensor.press_right_[press_count] <= Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count+1]))
			{
				if((Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count+1] - Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count])!=0)
				{
					ZMP_S_R_[press_count] = (double)Zmp_kg_table[kg_count] + (double)((Zmp_kg_table[kg_count+1]-Zmp_kg_table[kg_count])*((double)(sensor.press_right_[press_count] - Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count])/(double)(Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count+1] - Zmp_kg_offset_[press_count+RIGHT_PRESS_SHIFT][kg_count])));
					break;
				}
			}
			
		}
	}	


//					  left	press							 	  & right  press
	sensor_digital_	= ZMP_S_L_[0]+ZMP_S_L_[1]+ZMP_S_L_[2]+ZMP_S_L_[3] + ZMP_S_R_[0]+ZMP_S_R_[1]+ZMP_S_R_[2]+ZMP_S_R_[3];
	sensor_digital_left_	= ZMP_S_L_[0]+ZMP_S_L_[1]+ZMP_S_L_[2]+ZMP_S_L_[3];
	sensor_digital_right_	= ZMP_S_R_[0]+ZMP_S_R_[1]+ZMP_S_R_[2]+ZMP_S_R_[3];

	//ROBOT two feet ZMP 
	if(sensor_digital_ > 0.2)
	{
		ZMP_Y_ = (double)((((ZMP_S_L_[0]+ZMP_S_L_[3]-ZMP_S_R_[1]-ZMP_S_R_[2])*DOUBLE_FEET_WEIGHT_FAR_Y) + ((ZMP_S_L_[1]+ZMP_S_L_[2]-ZMP_S_R_[0]-ZMP_S_R_[3]) * DOUBLE_FEET_WEIGHT_NEAR_Y) )/(double)sensor_digital_);
		ZMP_X_ = (double)((ZMP_S_L_[0]+ZMP_S_L_[1]+ZMP_S_R_[0]+ZMP_S_R_[1]-ZMP_S_L_[2]-ZMP_S_L_[3]-ZMP_S_R_[2]-ZMP_S_R_[3]) * DOUBLE_FEET_WEIGHT_X )/(double)sensor_digital_;
	}
	else
	{
		ZMP_Y_ = 0 ;
		ZMP_X_ = 0 ;
	}
	// fp_digital.push_back(sensor_digital_);	
	// fp_ZMP_X.push_back(ZMP_X_);
	// fp_ZMP_Y.push_back(ZMP_Y_);


	//ROBOT left foot ZMP
	if(sensor_digital_left_ > 0.5)
	{
		ZMP_L_Y_ = ((ZMP_S_L_[0]+ZMP_S_L_[3]-ZMP_S_L_[1]-ZMP_S_L_[2]) * SINGLE_FOOT_WEIGHT_EQUAL_Y)/sensor_digital_left_;
		ZMP_L_X_ = ((ZMP_S_L_[0]+ZMP_S_L_[1]-ZMP_S_L_[2]-ZMP_S_L_[3]) * SINGLE_FOOT_WEIGHT_X)/sensor_digital_left_;
		//printf("%f",sensor_digital_left_);		
	}
	else
	{
		ZMP_L_Y_ = 0;
		ZMP_L_X_ = 0;		
	}
	// press_left_0.push_back((double)sensor.press_left_[0]);
	// press_left_1.push_back((double)sensor.press_left_[1]);
	// press_left_2.push_back((double)sensor.press_left_[2]);
	// press_left_3.push_back((double)sensor.press_left_[3]);
	// fp_ZMP_S_L_0.push_back(ZMP_S_L_[0]);
	// fp_ZMP_S_L_1.push_back(ZMP_S_L_[1]);
	// fp_ZMP_S_L_2.push_back(ZMP_S_L_[2]);
	// fp_ZMP_S_L_3.push_back(ZMP_S_L_[3]);	
	// fp_digital_L.push_back(sensor_digital_left_);	
	// fp_ZMP_L_X.push_back(ZMP_L_X_);
	// fp_ZMP_L_Y.push_back(ZMP_L_Y_);

	//ROBOT right foot ZMP
	if(sensor_digital_right_ > 0.1)
	{
		ZMP_R_Y_ = ((ZMP_S_R_[0]+ZMP_S_R_[3]-ZMP_S_R_[1]-ZMP_S_R_[2]) * SINGLE_FOOT_WEIGHT_EQUAL_Y)/sensor_digital_right_;
		ZMP_R_X_ = ((ZMP_S_R_[0]+ZMP_S_R_[1]-ZMP_S_R_[2]-ZMP_S_R_[3]) * SINGLE_FOOT_WEIGHT_X)/sensor_digital_right_;	
		//printf("sensor_digital_right_ = %f",sensor_digital_right_);			
	}
	else
	{ 
		ZMP_R_Y_ = 0;
		ZMP_R_X_ = 0;		
	}
	// press_right_0.push_back((double)sensor.press_right_[0]);
	// press_right_1.push_back((double)sensor.press_right_[1]);
	// press_right_2.push_back((double)sensor.press_right_[2]);
	// press_right_3.push_back((double)sensor.press_right_[3]);
	// fp_ZMP_S_R_0.push_back(ZMP_S_R_[0]);
	// fp_ZMP_S_R_1.push_back(ZMP_S_R_[1]);
	// fp_ZMP_S_R_2.push_back(ZMP_S_R_[2]);
	// fp_ZMP_S_R_3.push_back(ZMP_S_R_[3]);	
	// fp_digital_R.push_back(sensor_digital_right_);	
	// fp_ZMP_R_X.push_back(ZMP_R_X_);
	// fp_ZMP_R_Y.push_back(ZMP_R_Y_);

}

void ZeroMomentPoint::VectorSave()
{
	////feet

	fp_digital.push_back(sensor_digital_);	
	fp_ZMP_X.push_back(ZMP_X_);
	fp_ZMP_Y.push_back(ZMP_Y_);

	////left

	press_left_0.push_back((double)sensor.press_left_[0]);
	press_left_1.push_back((double)sensor.press_left_[1]);
	press_left_2.push_back((double)sensor.press_left_[2]);
	press_left_3.push_back((double)sensor.press_left_[3]);
	fp_ZMP_S_L_0.push_back(ZMP_S_L_[0]);
	fp_ZMP_S_L_1.push_back(ZMP_S_L_[1]);
	fp_ZMP_S_L_2.push_back(ZMP_S_L_[2]);
	fp_ZMP_S_L_3.push_back(ZMP_S_L_[3]);	
	fp_digital_L.push_back(sensor_digital_left_);	
	fp_ZMP_L_X.push_back(ZMP_L_X_);
	fp_ZMP_L_Y.push_back(ZMP_L_Y_);

	////right

	press_right_0.push_back((double)sensor.press_right_[0]);
	press_right_1.push_back((double)sensor.press_right_[1]);
	press_right_2.push_back((double)sensor.press_right_[2]);
	press_right_3.push_back((double)sensor.press_right_[3]);
	fp_ZMP_S_R_0.push_back(ZMP_S_R_[0]);
	fp_ZMP_S_R_1.push_back(ZMP_S_R_[1]);
	fp_ZMP_S_R_2.push_back(ZMP_S_R_[2]);
	fp_ZMP_S_R_3.push_back(ZMP_S_R_[3]);	
	fp_digital_R.push_back(sensor_digital_right_);	
	fp_ZMP_R_X.push_back(ZMP_R_X_);
	fp_ZMP_R_Y.push_back(ZMP_R_Y_);
	
}

//for left save
void ZeroMomentPoint::SaveDataL()
{
    string savedText = "P0\tP1\tP2\tP3\tZMP_L0\tZMP_L1\tZMP_L2\tZMP_L3\tdigi_L\tZMP_L_Y\tZMP_L_X\n";
    char path[200] = "/data";
    strcat(path, "/ZMP_L.xls");

    fstream fp;
    fp.open(path, ios::out);

    fp<<savedText; 

    for(int i = 0; i < fp_ZMP_L_Y.size(); i++)
    {
        savedText =
				 DtoS(press_left_0[i]) + "\t"
				+DtoS(press_left_1[i]) + "\t"
				+DtoS(press_left_2[i]) + "\t"
				+DtoS(press_left_3[i]) + "\t"
				+DtoS(fp_ZMP_S_L_0[i]) + "\t"
                +DtoS(fp_ZMP_S_L_1[i]) + "\t"
                +DtoS(fp_ZMP_S_L_2[i]) + "\t"
                +DtoS(fp_ZMP_S_L_3[i]) + "\t"
                +DtoS(fp_digital_L[i]) + "\t"
                +DtoS(fp_ZMP_L_Y[i]) + "\t"
                +DtoS(fp_ZMP_L_X[i]) + "\n";
        fp<<savedText;
     }
	// fp_ZMP_S_L_0.clear();
	// fp_ZMP_S_L_1.clear();
	// fp_ZMP_S_L_2.clear();
	// fp_ZMP_S_L_3.clear();
	// fp_digital_L.clear();
    // fp_ZMP_L_Y.clear();
    // fp_ZMP_L_X.clear();

    fp.close();
}

void ZeroMomentPoint::SaveDataR()
{
    string savedText = "Pr0\tPr1\tPr2\tPr3\tZMP_R0\tZMP_R1\tZMP_R2\tZMP_R3\tdigi_R\tZMP_R_Y\tZMP_R_X\n";
    char path[200] = "/data";
    strcat(path, "/ZMP_R.xls");

    fstream fp;
    fp.open(path, ios::out);

    fp<<savedText; 

    for(int i = 0; i < fp_ZMP_R_Y.size(); i++)
    {
        savedText =
				 DtoS(press_right_0[i]) + "\t"
				+DtoS(press_right_1[i]) + "\t"
				+DtoS(press_right_2[i]) + "\t"
				+DtoS(press_right_3[i]) + "\t"
				+DtoS(fp_ZMP_S_R_0[i]) + "\t"
                +DtoS(fp_ZMP_S_R_1[i]) + "\t"
                +DtoS(fp_ZMP_S_R_2[i]) + "\t"
                +DtoS(fp_ZMP_S_R_3[i]) + "\t"
                +DtoS(fp_digital_R[i]) + "\t"
                +DtoS(fp_ZMP_R_Y[i]) + "\t"
                +DtoS(fp_ZMP_R_X[i]) + "\n";
        fp<<savedText;
     }
	
	// fp_ZMP_S_R_0.clear();
	// fp_ZMP_S_R_0.clear();
	// fp_ZMP_S_R_0.clear();
	// fp_ZMP_S_R_0.clear();				
	// fp_ZMP_S_R_0.clear();
	// fp_ZMP_S_R_1.clear();
	// fp_ZMP_S_R_2.clear();
	// fp_ZMP_S_R_3.clear();
	// fp_digital_R.clear();
    // fp_ZMP_R_Y.clear();
    // fp_ZMP_R_X.clear();


    fp.close();
}

void ZeroMomentPoint::SaveDataLR()
{
    string savedText = "ZL0\tZL1\tZL2\tZL3\tZR0\tZR1\tZR2\tZR3\tdigi_LR\tZMP_Y\tZMP_X\n";
    char path[200] = "/data";
    strcat(path, "/ZMP_LR.xls");

    fstream fp;
    fp.open(path, ios::out);

    fp<<savedText; 

    for(int i = 0; i < fp_ZMP_R_Y.size(); i++)
    {
        savedText =
				 DtoS(fp_ZMP_S_L_0[i]) + "\t"
				+DtoS(fp_ZMP_S_L_1[i]) + "\t"
				+DtoS(fp_ZMP_S_L_2[i]) + "\t"
				+DtoS(fp_ZMP_S_L_3[i]) + "\t"
				+DtoS(fp_ZMP_S_R_0[i]) + "\t"
                +DtoS(fp_ZMP_S_R_1[i]) + "\t"
                +DtoS(fp_ZMP_S_R_2[i]) + "\t"
                +DtoS(fp_ZMP_S_R_3[i]) + "\t"
                +DtoS(fp_digital[i]) + "\t"
                +DtoS(fp_ZMP_Y[i]) + "\t"
                +DtoS(fp_ZMP_X[i]) + "\n";
	
        // savedText =DtoS(press_right_0.size()) + "\t"
		// 		+DtoS(press_right_1.size()) + "\t"
		// 		+DtoS(press_right_2.size()) + "\t"
		// 		+DtoS(press_right_3.size()) + "\t"
		// 		+DtoS(fp_ZMP_S_R_0.size()) + "\t"
        //         +DtoS(fp_ZMP_S_R_1.size()) + "\t"
        //         +DtoS(fp_ZMP_S_R_2.size()) + "\t"
        //         +DtoS(fp_ZMP_S_R_3.size()) + "\t"
        //         +DtoS(fp_digital_R.size()) + "\t"
        //         +DtoS(fp_ZMP_R_Y.size()) + "\t"
        //         +DtoS(fp_ZMP_R_X.size()) + "\n";
        fp<<savedText;
     }
    fp.close();
}

string ZeroMomentPoint::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}