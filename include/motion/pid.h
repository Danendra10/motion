#ifndef PID_H_
#define PID_H_

#ifdef _cplusplus
extern "C"
{
#endif

    typedef struct PID_tag
    {
        float Kp;
        float Ki;
        float Kd;
    } PID_t;

    void PIDInit(PID_t *pid, float Kp, float Ki, float Kd)
    {
        pid->Kp = Kp;
        pid->Ki = Ki;
        pid->Kd = Kd;
    }
    float PIDCalculate(PID_t *pid, float error, float dt)
    {
        static float integral = 0;
        static float last_error = 0;
        float derivative = (error - last_error) / dt;
        integral += error * dt;
        last_error = error;
        return pid->Kp * error + pid->Ki * integral + pid->Kd * derivative;
    }

    float GetProportional(PID_t *pid, float error)
    {
        return pid->Kp * error;
    }

    float GetIntegral(PID_t *pid, float error, float dt)
    {
        static float integral = 0;
        integral += error * dt;
        return pid->Ki * integral;
    }

    float GetDerivative(PID_t *pid, float error, float dt)
    {
        static float last_error = 0;
        float derivative = (error - last_error) / dt;
        last_error = error;
        return pid->Kd * derivative;
    }

#ifdef _cplusplus
}
#endif

#endif // PID_H_