#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

extern void pid_init(pid_struct_t *pid,
                     fp32 PID[3], fp32 max_out, fp32 max_iout);

extern fp32 pid_calc(pid_struct_t *pid, fp32 ref, fp32 set);
extern fp32 gimbal_Yaw_PID_calc(pid_struct_t *pid, fp32 fdb, fp32 set);
extern fp32 gimbal_Pitch_PID_cal(pid_struct_t *pid, fp32 fdb, fp32 set);
#endif