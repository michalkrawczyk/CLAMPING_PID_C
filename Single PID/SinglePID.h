//
// Created by MK on 18.07.2020.
//

#ifndef PID_C_SINGLEPID_H
#define PID_C_SINGLEPID_H

struct sPID_t;
typedef struct sPID_t PID_t;

void settingPID(double p, double i, double d);
void setBoundriesPID(double upper_bound, double lower_bound);

double calculateIdealPID(const double *set_point_ptr, const double *p_v_ptr, double dt);
double calculatePararellPID(const double *set_point_ptr, const double *p_v_ptr, double dt);

#endif //PID_C_SINGLEPID_H
