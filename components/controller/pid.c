/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       pid.c
  * @brief      PID controller implementation.
  * @note       Preserves the original API and adds optional derivative
  *             low-pass filtering plus integral-separation for tighter
  *             closed-loop behavior on Robocon chassis control.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "pid.h"

#include <stddef.h>
#include <math.h>

#define LimitMax(input, max)       \
    {                              \
        if ((input) > (max))       \
        {                          \
            (input) = (max);       \
        }                          \
        else if ((input) < -(max)) \
        {                          \
            (input) = -(max);      \
        }                          \
    }

void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }

    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->derivative_lpf_alpha = 1.0f;
    pid->integral_separation_err = -1.0f;
    pid->Dbuf[0] = 0.0f;
    pid->Dbuf[1] = 0.0f;
    pid->Dbuf[2] = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->out = 0.0f;
    pid->set = 0.0f;
    pid->fdb = 0.0f;
}

void PID_improve_init(pid_type_def *pid, fp32 integral_separation_err, fp32 derivative_lpf_alpha)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral_separation_err = (integral_separation_err > 0.0f) ? integral_separation_err : -1.0f;

    if (derivative_lpf_alpha < 0.0f)
    {
        pid->derivative_lpf_alpha = 0.0f;
    }
    else if (derivative_lpf_alpha > 1.0f)
    {
        pid->derivative_lpf_alpha = 1.0f;
    }
    else
    {
        pid->derivative_lpf_alpha = derivative_lpf_alpha;
    }
}

fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        fp32 raw_d_error = 0.0f;
        fp32 filtered_d_error = 0.0f;
        fp32 candidate_iout = 0.0f;
        fp32 tentative_out = 0.0f;
        uint8_t allow_integral = 1U;

        pid->Pout = pid->Kp * pid->error[0];

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        raw_d_error = pid->error[0] - pid->error[1];
        if (pid->derivative_lpf_alpha >= 1.0f)
        {
            filtered_d_error = raw_d_error;
        }
        else
        {
            filtered_d_error = pid->Dbuf[1] +
                               pid->derivative_lpf_alpha * (raw_d_error - pid->Dbuf[1]);
        }

        pid->Dbuf[0] = filtered_d_error;
        pid->Dout = pid->Kd * pid->Dbuf[0];

        if (pid->integral_separation_err > 0.0f &&
            fabsf(pid->error[0]) > pid->integral_separation_err)
        {
            allow_integral = 0U;
        }

        candidate_iout = pid->Iout;
        if (allow_integral)
        {
            candidate_iout += pid->Ki * pid->error[0];
            LimitMax(candidate_iout, pid->max_iout);
            tentative_out = pid->Pout + candidate_iout + pid->Dout;

            if (!((tentative_out > pid->max_out && pid->error[0] > 0.0f) ||
                  (tentative_out < -pid->max_out && pid->error[0] < 0.0f)))
            {
                pid->Iout = candidate_iout;
            }
        }

        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->Dbuf[0] = 0.0f;
    pid->Dbuf[1] = 0.0f;
    pid->Dbuf[2] = 0.0f;
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->fdb = 0.0f;
    pid->set = 0.0f;
}
