//
// Created by MK on 18.07.2020.
//
#include "SinglePID.h"
#include <assert.h>

/* PID STRUCTURES - DEFINITIONS*/
typedef struct
{
    double integral_sum;
    double last_error;
}PID_Data_t;


struct sPID_t{
    double kp, ki, kd;
    double upper_bound, lower_bound;
    PID_Data_t data;
};

/* USED PID DECLARATION*/
static PID_t pid = {0};

/* PID STRUCTURES - DEFINITIONS*/
static double integrate(const double *error_ptr, const double *dt_ptr);
static double derive(const double *error_ptr, const double *dt_ptr);

/* PID SETUP FUNCTIONS - DEFINITIONS (GLOBAL)*/
void setBoundriesPID(double upper_bound, double lower_bound)
{
    pid.upper_bound = upper_bound;
    pid.lower_bound = lower_bound;
}

void settingPID(double p, double i, double d)
{
    pid.kp = p;
    pid.ki = i;
    pid.kd = d;
}

/* PID CALCULATION FUNCTIONS - DEFINITIONS (GLOBAL)*/
double calculateIdealPID(const double *set_point_ptr, const double *p_v_ptr, double dt)
{
    assert(dt != 0);
    assert(!(pid.upper_bound == 0 && pid.lower_bound ==0)); //clamping anti-windup won't work without boundries

    double error = *set_point_ptr - *p_v_ptr;
    double temp_integral = integrate(&error, &dt);

    double c_v = pid.kp * (error + pid.ki * temp_integral + pid.kd * derive(&error, &dt));

    if (c_v > pid.upper_bound || c_v < pid.lower_bound)
    {
        c_v -= pid.kp * pid.ki * temp_integral; //switch off integral part
    }
    else
    {
        pid.data.integral_sum += temp_integral;
    }

    pid.data.last_error = error;

    return c_v;
}

double calculatePararellPID(const double *set_point_ptr, const double *p_v_ptr, double dt)
{
    assert(dt != 0);
    assert(!(pid.upper_bound == 0 && pid.lower_bound ==0)); //clamping anti-windup won't work without boundries

    double error = *set_point_ptr - *p_v_ptr;
    double temp_integral = integrate(&error, &dt);

    double c_v = pid.kp * error + pid.ki * temp_integral + pid.kd * derive(&error, &dt);
    if (c_v > pid.upper_bound || c_v < pid.lower_bound)
    {
        c_v -= pid.ki * temp_integral; //switch off integral part
    }
    else
    {
        pid.data.integral_sum += temp_integral;
    }

    pid.data.last_error = error;

    return c_v;
}

/* STATIC FUNCTIONS - DEFINITIONS*/

static double integrate(const double *error_ptr, const double *dt_ptr)
{
    return pid.data.integral_sum + *error_ptr * *dt_ptr;
}

static double derive(const double *error_ptr, const double *dt_ptr)
{
    return (*error_ptr - pid.data.last_error) / *dt_ptr;
}

