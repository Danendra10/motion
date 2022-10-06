#include "motion/motion.h"
#include "motion/pid.h"
#include <ros/ros.h>
#include <angles/angles.h>
#include "utils/pid.h"
#include "utils/utils.h"

/* Acceleration control */
void ManualMotion(motion_data_t *data, motion_return_t *ret)
{
    pos_robot[0] += data->vel_x;
    pos_robot[1] += data->vel_y;
    pos_robot[2] += data->vel_th;
    ObstacleCheck(90, 80, 80, 1);
    static float v_buffer[2];

    float delta_v[2];
    delta_v[0] = data->vel_x - v_buffer[0];
    delta_v[1] = data->vel_y - v_buffer[1];

    float r = sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1]);
    float theta = atan2(delta_v[1], delta_v[0]);

    if (r > data->acceleration)
        r = data->acceleration;

    v_buffer[0] += r * cos(theta);
    v_buffer[1] += r * sin(theta);

    // printf("Velocity datas: %f %f %f\n", v_buffer[0], v_buffer[1], data->vel_th);

    ret->vx = v_buffer[0];
    ret->vy = v_buffer[1];
    ret->vth = data->vel_th;
}

/* Position Control */
void PositionAngularMotion(motion_data_t *data, motion_return_t *ret, robot_data_t *robot_data)
{
    PID_t pid_posisi;
    PID_t pid_sudut;

    PIDInit(&pid_posisi, 0.1, 0.1, 0.1);
    PIDInit(&pid_sudut, 0.1, 0.1, 0.1);

    /* Error Posisi X */
    error[0] = data->target_x - robot_data->robot_x[robot_data->robot_num];

    /* Error Posisi Y */
    error[1] = data->target_y - robot_data->robot_y[robot_data->robot_num];

    /* Error theta */
    error[2] = data->target_th - robot_data->robot_th[robot_data->robot_num];

    /* Error Posisi */
    error[3] = sqrt(error[0] * error[0] + error[1] * error[1]);

    while (error[3] < -180)
        error[3] += 360;
    while (error[3] > 180)
        error[3] -= 360;

    output[0] = PIDCalculate(&pid_posisi, error[3], 0.1);
    output[1] = PIDCalculate(&pid_sudut, error[2], 0.1);

    /* Output vel X */
    output[2] = output[0] * cos(atan2(error[0], error[1]));
    output[3] = output[0] * sin(atan2(error[0], error[1]));

    /**
     * The vx, vy, and vth will be sent to decision maker
     * And will be published to comm motor
     */
    ret->vx = output[2];
    ret->vy = output[3];
    ret->vth = output[1];
}

void ResetVelocity(motion_data_t *data, motion_return_t *ret)
{
    ret->vx = 0;
    ret->vy = 0;
    ret->vth = 0;
}