#include "include/Walkinggait.h"

WalkingCycle walkingcycle;
WalkingTrajectory walkingtrajectory;

extern Initial init;

Walkinggait::Walkinggait()
{
    update_parameter_flag_ = false;
    update_walkdata_flag_ = false;
    continuous_stop_flag_ = false;
    get_parameter_flag_ = false;
    get_walkdata_flag_ = false;
    locus_flag_ = false;
}

Walkinggait::~Walkinggait()
{

}

void Walkinggait::walking_timer()
{
    if(!parameterinfo->complan.walking_stop)
    {
        walkingcycle.walkingkindfunction(parameterinfo->walking_mode);
        walkingtrajectory.walkingprocess(parameterinfo->walking_mode);

        parameterinfo->CPGalready = true;
        locus_flag_ = true;
    }
    gettimeofday(&timer_start_, NULL);
}

void Walkinggait::load_parameter()
{
    int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_parameter_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_parameter_addr)
			{
				state = 1;
				continue;
			}
			else
			{
				break;
			}
		}
		else if(state == 1)
		{
			if(count <= 5)
			{
				parameter_[count] = *(uint32_t *)init.p2h_parameter_addr;
				count++;
				*(uint32_t *)init.h2p_read_parameter_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_parameter_pulse_addr = 0;
				continue;
			}
			else
			{
				update_parameter_flag_ = true;
				state = 0;
				break;
			}
		}
	}
    update_parameter();
}

void Walkinggait::update_parameter()
{
    if(update_parameter_flag_)
    {
        int parameter_cnt;
        int arr_index = 0;
        short tmp = 0;
        double tmp_arr[12] = {0.0};

        for(parameter_cnt=0; parameter_cnt<6; parameter_cnt++)
        {
            tmp = ((parameter_[parameter_cnt] & 0xFFFF0000) >> 16);
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1)) / 100;
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF)) / 100;

            tmp = ((parameter_[parameter_cnt] & 0x0000FFFF));
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1)) / 100;
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF)) / 100;
        }

        arr_index = 0;
        parameter_cnt = 1;
        parameterinfo->parameters.X_Swing_Range = tmp_arr[arr_index++];
        parameterinfo->parameters.Y_Swing_Range = tmp_arr[arr_index++];
        parameterinfo->parameters.Z_Swing_Range = tmp_arr[arr_index++];
        parameterinfo->parameters.Period_T = parameter_[parameter_cnt++] & 0x0000FFFF;
        parameterinfo->parameters.Period_T2 = (parameter_[parameter_cnt] & 0xFFFF0000) >> 16;
        parameterinfo->parameters.Sample_Time = (parameter_[parameter_cnt] & 0x0000FF00) >> 8;
        parameterinfo->parameters.OSC_LockRange = ((double)(parameter_[parameter_cnt++] & 0x000000FF)) / 100;

        arr_index = 6;
        parameter_cnt = 5;
        parameterinfo->parameters.BASE_Default_Z = tmp_arr[arr_index++];
        parameterinfo->parameters.X_Swing_COM = tmp_arr[arr_index++];
        parameterinfo->parameters.Y_Swing_Shift = tmp_arr[arr_index++];
        parameterinfo->parameters.BASE_LIFT_Z = tmp_arr[arr_index++];
        parameterinfo->walking_mode = (parameter_[parameter_cnt] & 0xFF000000) >> 24;


        if(parameterinfo->parameters.Sample_Time == 0)
        {
            motion_delay_ = 30;
        }
        else
        {
            motion_delay_ = parameterinfo->parameters.Period_T / parameterinfo->parameters.Sample_Time;
        }       
        get_parameter_flag_ = true;
    }
}

void Walkinggait::load_walkdata()
{
    int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_walkdata_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_walkdata_addr)
			{
				state = 1;
				continue;
			}
			else
			{
				break;
			}
		}
		else if(state == 1)
		{
			if(count <= 2)
			{
				walkdata_[count] = *(uint32_t *)init.p2h_walkdata_addr;
				count++;
				*(uint32_t *)init.h2p_read_walkdata_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_walkdata_pulse_addr = 0;
				continue;
			}
			else
			{
				update_walkdata_flag_ = true;
				state = 0;
				break;
			}
		}
	}
    update_walkdata();
}

void Walkinggait::update_walkdata()
{
    if(update_walkdata_flag_)
    {
        int walkdata_cnt;
        int arr_index = 0;
        short tmp = 0;
        double tmp_arr[12] = {0.0};

        for(walkdata_cnt=0; walkdata_cnt<2; walkdata_cnt++)
        {
            tmp = ((walkdata_[walkdata_cnt] & 0xFFFF0000) >> 16);
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1));
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF));

            tmp = ((walkdata_[walkdata_cnt] & 0x0000FFFF));
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1));
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF));
        }

        arr_index = 0;
        walkdata_cnt = 2;
        parameterinfo->X = tmp_arr[arr_index++] / 1000.0;
        parameterinfo->Y = tmp_arr[arr_index++] / 1000.0;
        parameterinfo->Z = tmp_arr[arr_index++] / 1000.0;
        parameterinfo->THTA = tmp_arr[arr_index] / 180.0 * PI;
        walking_cmd_ = (walkdata_[walkdata_cnt] >> 24) & 0xFF;
        sensor_mode_ = (walkdata_[walkdata_cnt] >> 16) & 0xFF;

        get_walkdata_flag_ = true;
    }
}

void Walkinggait::calculate_point_trajectory()
{
    if(get_parameter_flag_ && get_walkdata_flag_)
    {
        if(walking_cmd_ != etChangeValue)
        {
            // check walking_cmd if it is start , stop or change value
            if(parameterinfo->complan.walking_state == StopStep)
            {
                parameterinfo->complan.walking_stop = false;
                parameterinfo->complan.walking_state = StartStep;
                parameterinfo->WalkFlag = true;
                parameterinfo->counter = 0;
                parameterinfo->Repeat = true;
            }
            else
            {
                parameterinfo->WalkFlag = false;
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->Repeat = false;
            }
        }
        get_parameter_flag_ = false;
    }
    get_walkdata_flag_ = false;
}

WalkinggaitByLIPM::WalkinggaitByLIPM()
{

}
WalkinggaitByLIPM::~WalkinggaitByLIPM()
{

}

void WalkinggaitByLIPM::process()
{
    // WTc = sqrt(Parameters.COM_Height/Wg);

    // A = length * now_step;
    // B = width_size * pow((-1), (now_step+1));

    // d1 = length/2;
    // d2 = length/2;

    // if(now_step == 0)   //init
    // {
    //     A = 0;
    //     d1 = 0;
    //     lpx = wLIPM_foot_px_init(length, t, TT, T_DSP, now_step);
    //     rpx = 0;
    //     lpz = wLIPM_foot_pz(height, t, TT, T_DSP);
    //     rpz = 0;
    //     if((lpx > rpx) && (lpz == 0) && (lpz == 0))
    //     {
    //         //
    //     }
    //     else if(lpz != 0)
    //     {
    //         //
    //     }
    // }
    // else if(now_step == wn) //fin
    // {
    //     d2 = 0;

    //     if((now_step % 2) == 1)
    //     {
    //         lpx = length * now_step;
    //         rpx = wLIPM_foot_px_fin(length, t, TT, T_DSP, now_step);
    //         lpz = 0;
    //         rpz = wLIPM_foot_pz(height, t, TT, T_DSP);
    //         if((rpx > lpx) && (rpz == 0))
    //         {
    //             length_boun_min = length_boun_max;
    //             length_boun_max = rpx;
    //         }
    //         else if(rpz != 0)
    //         {
    //             length_boun_min = lpx;
    //             length_boun_max = lpx;
    //         }
    //     }
    //     else if((now_step % 2) == 0)
    //     {
    //         lpx = wLIPM_foot_px_fin(length, t, TT, T_DSP, now_step);
    //         rpx = length * now_step;
    //         lpz = wLIPM_foot_pz(height, t, TT, T_DSP);
    //         rpz = 0;
    //         if((lpx > rpx) && (lpz == 0))
    //         {
    //             length_boun_min = length_boun_max;
    //             length_boun_max = lpx;
    //         }
    //         else if(rpz != 0)
    //         {
    //             length_boun_min = rpx;
    //             length_boun_max = rpx;
    //         }
    //     }
    // }
    // else    //repeat
    // {
    //     d1 = length / 2;
    //     d2 = length / 2;

    //     wzmpx_data = set_zmpx_data_repeat(length, t, TT, T_DSP, now_step);
    //     if((now_step % 2) == 1)
    //     {
    //         lpx = length * now_step;
    //         rpx = wLIPM_foot_px_repeat(length, t, TT, T_DSP, now_step);
    //         lpz = 0;
    //         rpz = wLIPM_foot_pz(height, t, TT, T_DSP);
    //         if((rpx >　lpx) && (rpz == 0))
    //         {
    //             length_boun_min = length_boun_max;
    //             length_boun_max = rpx;
    //         }
    //         else if(rpz != 0)
    //         {
    //             length_boun_min = lpx;
    //             length_boun_max = lpx;
    //         }
    //     }
    //     else if((now_step % 2) == 0)
    //     {
    //         lpx = wLIPM_foot_px_repeat(length, t, TT, T_DSP, now_step);
    //         rpx = length * now_step;
    //         lpz = wLIPM_foot_pz(height, t, TT, T_DSP);
    //         rpz = 0;
    //         if((lpx >　rpx) && (lpz == 0))
    //         {
    //             length_boun_min = length_boun_max;
    //             length_boun_max = lpx;
    //         }
    //         else if(rpz != 0)
    //         {
    //             length_boun_min = rpx;
    //             length_boun_max = rpx;
    //         }
    //     }
    // }

    // l1 = A - d1;
    // l2 = A-aa1-(((aa1+aa2)*t1)/(t2-t1));
    // l3 = A+aa2-(((d2-aa2)*t2)/(TT-t2));

    // k1 = (d1-aa1)/t1;
    // k2 = ((aa1+aa2))/(t2-t1);
    // k3 = (d2-aa2)/(TT-t2);
    
    // y1t = B*(TT-WTc*sinh(TT/WTc))/t1;
    // y2t = B*(1-cosh((TT-t1)/WTc))-B*(TT-t1*cosh((TT-t1)/WTc)-WTc*sinh((TT-t1)/WTC))/t1;
    // y3t = B*t2*(1-cosh((TT-t2)/WTc))/t1-B*(TT-t2*cosh((TT-t2)/WTc)-WTc*sinh((TT-t2)/WTc))/t1;

    // x1t = WLIPM_com_X1(L1,K1,TT,WTc);
    // x2t = WLIPM_com_X2(L2,L1,K2,K1,TT,WTc,t1);
    // x3t = WLIPM_com_X3(L3,L2,K3,K2,TT,WTc,t2);
    // dx0 = DSP_X_velocity_0(A-d1,A+d2,TT,WTc,X1T,X2T,X3T) + (pow(WTc,2)*q/pow(C,2))*(cosh(TT/WTc)-1)/(WTc*sinh(TT/WTc)); 

    // if((t>=0)&&(t<=t1))
    // {

    // }
}

double WalkinggaitByLIPM::wosc_move_x(double lock_range, double period_t, double step_x, int time_t)
{
    double period_t_ms = period_t * 0.001;
    double t_divby2 = period_t_ms / 2;
    double t_divby4 = period_t_ms / 4;
    int time_t_temp;
    if((time_t % (int)period_t) == 0)
        time_t_temp = time_t;
    else
        time_t_temp = time_t % (int)period_t;
    
    double time_t_ms = (double)(time_t_temp/1000);
    double omega_x = 2 * PI / period_t_ms / (1-lock_range) * 1;
    
    if(time_t_ms >= 0 && time_t_ms <= t_divby4/2)
    {
        return(step_x/sin(PI/4)) * sin(omega_x * (time_t_ms));
    }
    else if(time_t_ms > t_divby4/2 && time_t_ms <= t_divby4/2+t_divby4)
    {
        return step_x;
    }
    else if(time_t_ms > t_divby4/2+t_divby4 && time_t_ms <= t_divby4/2+t_divby2)
    {
        return (step_x/sin(PI/4)) * sin(omega_x * (time_t_ms));
    }
    else if(time_t_ms > t_divby4/2+t_divby2 && time_t_ms <= t_divby4/2+t_divby2+t_divby4)
    {
        return -step_x;
    }
    else if(time_t_ms > t_divby4/2+t_divby2+t_divby4)
    {
        return (step_x/sin(PI/4)) * sin(omega_x * (time_t_ms));
    }
    else
        return 0;
    
}
double WalkinggaitByLIPM::wosc_move_y(double lock_range, double period_t, double step_y, int time_t)
{
    double period_t_ms = period_t * 0.001;
    double t_divby2 = period_t_ms / 2;
    double t_divby4 = period_t_ms / 4;
    double time_t_ms = (double)(time_t/1000);
    double omega_y = PI/period_t_ms/(1-lock_range);

    return step_y * sin(omega_y * time_t_ms);
}
double WalkinggaitByLIPM::wosc_move_z(double lock_range, double period_t, double step_z, double rho_z, int time_t)
{
    double period_t_ms = period_t * 0.001;
    double t_divby2 = period_t_ms / 2;
    double t_divby4 = period_t_ms / 4;
    int time_t_temp;
    if((time_t % (int)period_t) == 0)
        time_t_temp = time_t;
    else
        time_t_temp = time_t % (int)period_t;
    double time_t_ms = (double)(time_t_temp/1000);
    double omega_z = PI / period_t_ms / (1-lock_range);

    if(time_t_ms >= 0 && time_t_ms < t_divby2+t_divby4)
    {
        return 0;
    }
    else if(time_t_ms >= t_divby2+t_divby4 && time_t_ms < period_t_ms-(lock_range *t_divby4))
    {
        return step_z * sin(4 * omega_z * (time_t_ms - lock_range * 3 * t_divby4) + rho_z);
    }
    else
        return 0;
    
}
double WalkinggaitByLIPM::wLIPM_com_vx0(double x0, double xt, double px, double z, double t, double T, double C, double q)
{
    return (xt-x0*cosh(t/T)+(px+(pow(T,2)*q/pow(C,2)))*(cosh(t/T)-1))/(T*sin(t/T));
}
double WalkinggaitByLIPM::wLIPM_com_x1(double l1, double k1, double t, double Tc)
{
    return l1*(1-cosh(t/Tc))+k1*(t-Tc*sinh(t/Tc));
}
double WalkinggaitByLIPM::wLIPM_com_x2(double l2, double l1, double k2, double k1, double t, double Tc, double t1)
{
    return (l2-l1)*(1-cosh((t-t1)/Tc)) + (k2-k1)*(t-t1*cosh((t-t1)/Tc)-Tc*sinh((t-t1)/Tc));
}
double WalkinggaitByLIPM::wLIPM_com_x3(double l3, double l2, double k3, double k2, double t, double Tc, double t2)
{
    return (l3-l2)*(1-cosh((t-t2)/Tc)) + (k3-k2)*(t-t2*cosh((t-t2)/Tc)-Tc*sinh((t-t2)/Tc));
}
double WalkinggaitByLIPM::dsp_x_velocity_0(double x0, double xt, double t, double Tc, double x1t, double x2t, double x3t)
{
    return (xt-x0*cosh(t/Tc)-x1t-x2t-x3t)/(Tc*sinh(t/Tc));
}
double WalkinggaitByLIPM::dsp_x_velocity_1(double l1, double k1, double t, double Tc)
{
    return l1*(0-sinh(t/Tc)/Tc)+k1*(1-cosh(t/Tc));
}
double WalkinggaitByLIPM::dsp_x_velocity_2(double l2, double l1, double k2, double k1, double t, double Tc, double t1)
{
    return (l2-l1)*(0-sinh((t-t1)/Tc)/Tc) +(k2-k1)*(1-t1*sinh((t-t1)/Tc)/Tc-cosh((t-t1)/Tc));
}
double WalkinggaitByLIPM::dsp_x_velocity_3(double l3, double l2, double k3, double k2, double t, double Tc, double t2)
{
    return (l3-l2)*(0-sinh((t-t2)/Tc)/Tc) +(k3-k2)*(1-t2*sinh((t-t2)/Tc)/Tc-cosh((t-t2)/Tc));
}
double WalkinggaitByLIPM::wLIPM_com_px(double x0, double dx0, double px, double z, double t, double T, double C, double q)
{
    return x0*cosh(t/T)+T*dx0*sinh(t/T)-(px+(pow(T,2)*q/pow(C,2)))*(cosh(t/T)-1);
}
double WalkinggaitByLIPM::wLIPM_com_vx(double x0, double dx0, double px, double z, double t, double T, double C, double q)
{
    return x0*sinh(t/T)/T+dx0*cosh(t/T)-(px/T+(T*q/pow(C,2)))*sinh(t/T);
}
double WalkinggaitByLIPM::wLIPM_com_vy0(double y0, double py, double z, double t, double T)
{
    return ((y0-y0*cosh(t/T))+py*(cosh(t/T)-1))/(T*sinh(t/T));
}
double WalkinggaitByLIPM::wLIPM_com_py(double y0, double dy0, double py, double z, double t, double T)
{
    return y0*cosh(t/T)+T*dy0*sinh(t/T)-py*(cosh(t/T)-1);
}
double WalkinggaitByLIPM::wLIPM_com_vy(double y0, double dy0, double py, double z, double t, double T)
{
    return y0*sinh(t/T)/T+dy0*cosh(t/T)-(py/T)*sinh(t/T);
}
double WalkinggaitByLIPM::wLIPM_com_pz(const double com_rho_z, const double t, const double T, const double t_dsp, const int now_step)
{
    double omega = 2*PI/T;
    return 0.5*com_rho_z*(1-cos(omega*t));
}
double WalkinggaitByLIPM::wLIPM_foot_px_init(const double wlength, const double t, const double T, const double t_dsp, const int now_step)
{
    double new_T = T*(1-t_dsp);
    double new_t = t-T*t_dsp/2;
    double omega = 2*PI/new_T;
    
    if(t > T*t_dsp/2 && t <= T*(1-t_dsp/2))
    {
        return wlength*(omega*new_t-sin(omega*new_t+0))/(2*PI);
    }
    else if(t <= T*t_dsp/2 && t > 0)
    {
        return wlength*now_step;
    }
    else if(t > T*(1-t_dsp/2) && t <= T)
    {
        return wlength*(now_step+1);
    }
    else 
    {
        return -1;
    }
}
double WalkinggaitByLIPM::wLIPM_foot_px_fin(const double wlength, const double t, const double T, const double t_dsp, const int now_step)
{
    double new_T = T*(1-t_dsp);
    double new_t = t-T*t_dsp/2;
    double omega = 2*PI/new_T;
    
    if(t > T*t_dsp/2 && t <= T*(1-t_dsp/2))
    {
        return wlength*(omega*new_t-sin(omega*new_t+0))/(2*PI)+wlength*(now_step-1);
    }
    else if(t <= T*t_dsp/2 && t > 0)
    {
        return wlength*(now_step-1);
    }
    else if(t > T*(1-t_dsp/2) && t <= T)
    {
        return wlength*now_step;
    }
    else 
    {
        return -1;
    }
}
double WalkinggaitByLIPM::wLIPM_foot_px_repeat(const double wlength, const double t, const double T, const double t_dsp, const int now_step)
{
    double new_T = T*(1-t_dsp);
    double new_t = t-T*t_dsp/2;
    double omega = 2*PI/new_T;
    
    if(t > T*t_dsp/2 && t <= T*(1-t_dsp/2))
    {
        return 2*wlength*(omega*new_t-sin(omega*new_t+0))/(2*PI)+wlength*(now_step-1);
    }
    else if(t <= T*t_dsp/2 && t > 0)
    {
        return wlength*(now_step-1);
    }
    else if(t > T*(1-t_dsp/2) && t <= T)
    {
        return wlength*(now_step+1);
    }
    else 
    {
        return -1;
    }
}
double WalkinggaitByLIPM::wLIPM_foot_pz(const double wheight, const double t, const double T, const double t_dsp)
{
    double new_T = T*(1-t_dsp);
    double new_t = t-T*t_dsp/2;
    double omega = 2*PI/new_T;
    
    if (t > T*t_dsp/2 && t <= T*(1-t_dsp/2))
    {
        return 0.5*wheight*(1-cos(omega*new_t));
    }
    else
    {
        return 0;
    }
}
double WalkinggaitByLIPM::sinh(double x)
{
    return (double)(exp(x)-exp(-x))/2;
}
double WalkinggaitByLIPM::cosh(double x)
{
    return (double)(exp(x)+exp(-x))/2;
}