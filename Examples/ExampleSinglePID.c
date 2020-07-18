////
//// Created by MK on 18.07.2020.
////
//
#include <stdio.h>
#include "SinglePID.h"
//#include <unistd.h>

int main() {
    settingPID(0.1, 0.5, 0.01);
    setBoundriesPID(20, -20);

    double set_point = 1;
    double p_v = 10;
    double d_t = 1.0;

    for (int i = 0; i < 200; i++)
    {
        //double c_v = calculatePararellPID(&set_point, &p_v, d_t);
        double c_v = calculateIdealPID(&set_point, &p_v, d_t);
        printf("p_v:% 4.3f \t c_v:% 4.3f\n", p_v, c_v);
        p_v += c_v;
//        sleep(d_t); //Convertion from double to int - ok in this example
    }
    return 0;
}