/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       pid.h
  * @brief      PID controller interface and data structure.
  * @note       Keeps the legacy API while adding optional derivative filtering
  *             and integral-separation support for competition tuning.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef PID_H
#define PID_H

#include "struct_typedef.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 derivative_lpf_alpha;
    fp32 integral_separation_err;

    fp32 Dbuf[3];
    fp32 error[3];
} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern void PID_improve_init(pid_type_def *pid, fp32 integral_separation_err, fp32 derivative_lpf_alpha);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);

#endif
