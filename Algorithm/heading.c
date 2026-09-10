#include "heading.h"
#include <math.h>
float Heading_Update(float error, float gyro, float dt, float kp, float ki, float kg, float limit,
                     float *integral)
{
    if (fabsf(error) < 0.005f)
        error = 0;
    *integral = fmaxf(-0.5f, fminf(0.5f, *integral + error * dt));
    return fmaxf(-limit, fminf(limit, kp * error + ki * *integral - kg * gyro));
}
