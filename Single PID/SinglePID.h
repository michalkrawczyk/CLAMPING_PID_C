//
// Created by MK on 18.07.2020.
//

#ifndef PID_C_SINGLEPID_H
#define PID_C_SINGLEPID_H

struct sPID_t;
typedef struct sPID_t PID_t;

/*Setup Functions*/
void tunePID(double p, double i, double d);                         //set tuning for PID controller
void setBoundriesPID(double upper_bound, double lower_bound);       //set boundries which activate clamping anti-windup

/*PID Functions - Choose which model want to use
 * Note: Those Functions returns control value (CV)*/
double calculateIdealPID(const double *setpoint_ptr, const double *pv_ptr, double dt);      //Ideal PID
double calculateParallelPID(const double *setpoint_ptr, const double *pv_ptr, double dt);   //Parallel PID

#endif //PID_C_SINGLEPID_H
