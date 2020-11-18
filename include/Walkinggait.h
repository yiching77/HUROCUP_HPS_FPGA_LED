/*
 * walkinggait.h
 *
 *  Created on: 2019/09/21
 *      Author: Yu-Chih, Wang
 */

#ifndef WALKINGGAIT_H_
#define WALKINGGAIT_H_

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "Initial.h"
#include "WalkingCycle.h"
#include "WalkingTrajectory.h"
// #include <std_msgs/String.h>

#define WALKING_INTERVAL 30000.0   //30 ms

typedef enum
{
    etStart = 0x01,
    etStop = 0x02,
    etChangeValue = 0x04
}etWalkingCmd;

class Walkinggait
{
public:
    Walkinggait();
    ~Walkinggait();

    void load_parameter();
    void update_parameter();
    void load_walkdata();
    void update_walkdata(); 
    void calculate_point_trajectory();
    void walking_timer();

    double x_, y_, z_, thta_;
    char walking_cmd_, sensor_mode_;
    
    struct timeval timer_start_, timer_end_;
    double timer_dt_;
    bool get_parameter_flag_;
    bool get_walkdata_flag_;
    bool locus_flag_;
    int motion_delay_;

private:
    bool update_parameter_flag_;
    bool update_walkdata_flag_;
    bool continuous_stop_flag_;
    int parameter_[6];
    int walkdata_[3];
};

class WalkinggaitByLIPM
{
public:
    WalkinggaitByLIPM();
    ~WalkinggaitByLIPM();

    void process();

    double wosc_move_x(double lock_range, double period_t, double step_x, int time_t);
    double wosc_move_y(double lock_range, double period_t, double step_y, int time_t);
    double wosc_move_z(double lock_range, double period_t, double step_z, double rho_z, int time_t);

    double wLIPM_com_vx0(double x0, double xt, double px, double z, double t, double T, double C, double q);
    double wLIPM_com_x1(double l1, double k1, double t, double Tc);
    double wLIPM_com_x2(double l2, double l1, double k2, double k1, double t, double Tc, double t1);
    double wLIPM_com_x3(double l3, double l2, double k3, double k2, double t, double Tc, double t2);
    double dsp_x_velocity_0(double x0, double xt, double t, double Tc, double x1t, double x2t, double x3t);
    double dsp_x_velocity_1(double l1, double k1, double t, double Tc);
    double dsp_x_velocity_2(double l2, double l1, double k2, double k1, double t, double Tc, double t1);
    double dsp_x_velocity_3(double l3, double l2, double k3, double k2, double t, double Tc, double t2);
    double wLIPM_com_px(double x0, double dx0, double px, double z, double t, double T, double C, double q);
    double wLIPM_com_vx(double x0, double dx0, double px, double z, double t, double T, double C, double q);
    double wLIPM_com_vy0(double y0, double py, double z, double t, double T);
    double wLIPM_com_py(double y0, double dy0, double py, double z, double t, double T);
    double wLIPM_com_vy(double y0, double dy0, double py, double z, double t, double T);
    double wLIPM_foot_px_init(const double wlength, const double t, const double T, const double t_dsp, const int now_step);
    double wLIPM_foot_px_fin(const double wlength, const double t, const double T, const double t_dsp, const int now_step);
    double wLIPM_foot_px_repeat(const double wlength, const double t, const double T, const double t_dsp, const int now_step);
    double wLIPM_foot_pz(const double wheight, const double t, const double T, const double t_dsp);
    double wLIPM_com_pz(const double com_rho_z, const double t, const double T, const double t_dsp, const int now_step);

    double sinh(double x);
    double cosh(double x);

private:

};

#endif /* WALKINGGAIT_H_ */