#ifndef HEADING_H
#define HEADING_H
float Heading_Update(float error_rad, float gyro_rad_s, float dt, float kp, float ki, float kg,
                     float limit, float *integral);
#endif
