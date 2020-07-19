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
/**
  * @brief Sets boundries which activate clamping anti-windup
  * @param upper_bound (double) - If Calculated CV will be higher than upper_bound - clamping will be activated
  * @param lower_bound (double) - If Calculated CV will be lower than lower_bound - clamping will be activated
  * @retval None
  */
void setBoundriesPID(double upper_bound, double lower_bound)
{
    pid.upper_bound = upper_bound;
    pid.lower_bound = lower_bound;
}

/**
  * @brief Sets Tuning of PID Controller
  * @param p (double) - sets value of Proportional Term Multiplier(kp)
  * @param i (double) - sets value of Integral Term Multiplier(ki)
  * @param d (double) - sets value of Derivative Term Multiplier(kd)
  * @retval None
  */
void tunePID(double p, double i, double d)
{
    pid.kp = p;
    pid.ki = i;
    pid.kd = d;
}

/* PID CALCULATION FUNCTIONS - DEFINITIONS (GLOBAL)*/
/**
  * @brief implements calculation of control value for ideal PID
  * Note: Pointers used to avoid making unecessary copies, which may be heavy for e.g. microcontrollers
  * @param setpoint_ptr (double) - pointer to Setpoint Value
  * @param pv_ptr (double) - pointer to value of Process Value (PV)
  * @param dt (double) - time from last calculation. If unknown - better keep at 1.0
  * @retval cv - Control Value
  */
double calculateIdealPID(const double *setpoint_ptr, const double *pv_ptr, double dt)
{
    assert(dt != 0);
    assert(!(pid.upper_bound == 0 && pid.lower_bound ==0)); //clamping anti-windup won't work without boundries

    double error = *setpoint_ptr - *pv_ptr;
    double temp_integral = integrate(&error, &dt);

    double cv = pid.kp * (error + pid.ki * temp_integral + pid.kd * derive(&error, &dt));

    if (cv > pid.upper_bound || cv < pid.lower_bound)
    {
        cv -= pid.kp * pid.ki * temp_integral; //switch off integral part
    }
    else
    {
        pid.data.integral_sum += temp_integral;
    }

    pid.data.last_error = error;

    return cv;
}

/**
  * @brief implements calculation of control value for parallel PID
  * Note: Pointers used to avoid making unecessary copies, which may be heavy for e.g. microcontrollers
  * @param setpoint_ptr (double) - pointer to Setpoint Value
  * @param pv_ptr (double) - pointer to value of Process Value (PV)
  * @param dt (double) - time from last calculation. If unknown - better keep at 1.0
  * @retval cv - Control Value
  */
double calculateParallelPID(const double *setpoint_ptr, const double *pv_ptr, double dt)
{
    assert(dt != 0);
    assert(!(pid.upper_bound == 0 && pid.lower_bound ==0)); //clamping anti-windup won't work without boundries

    double error = *setpoint_ptr - *pv_ptr;
    double temp_integral = integrate(&error, &dt);

    double cv = pid.kp * error + pid.ki * temp_integral + pid.kd * derive(&error, &dt);
    if (cv > pid.upper_bound || cv < pid.lower_bound)
    {
        cv -= pid.ki * temp_integral; //switch off integral part
    }
    else
    {
        pid.data.integral_sum += temp_integral;
    }

    pid.data.last_error = error;

    return cv;
}

/* STATIC FUNCTIONS - DEFINITIONS*/

/**
  * @brief Performs 'Integrating' Operation
  * @param error_ptr (double) - pointer to error value (difference between setpoint and PV)
  * @param dt_ptr (double) - pointer to time from last calculation
  * @retval calculated integral sum
  */
static double integrate(const double *error_ptr, const double *dt_ptr)
{
    return pid.data.integral_sum + *error_ptr * *dt_ptr;
}

/**
  * @brief Performs 'Derivative' Operation
  * @param error_ptr (double) - pointer to error value (difference between setpoint and PV)
  * @param dt_ptr (double) - pointer to time from last calculation
  * @retval calculated differential
  */
static double derive(const double *error_ptr, const double *dt_ptr)
{
    return (*error_ptr - pid.data.last_error) / *dt_ptr;
}

