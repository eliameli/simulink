#include <math.h>

void c_turning_function_by_Euler(const double v[], const double w[], double dv_dt[])
{
    dv_dt[0] = w[1]*sin(v[2]) + w[2]*cos(v[2]);
    dv_dt[1] = (w[1]*cos(v[2]) - w[2]*sin(v[2]))/cos(v[0]);
    dv_dt[2] = w[0] - tan(v[0])*(w[1]*cos(v[2]) - w[2]*sin(v[2]));
}