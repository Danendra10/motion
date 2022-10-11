#include "utils/utils.h"
#include "utils/pid.h"
#include "motion/motion.h"

/* Acceleration control */
void ManualMotion(motion_data_t *data, motion_return_t *ret)
{
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

void ManualMotionVector(uint16_t vx, uint16_t vy, int16_t angles_target, motion_return_t *ret)
{
    static PID_t angles_pid;
    motion_data_t motion;

    PIDInit(&angles_pid, 1, 0, 0);

    float angle_error = angles_target - pos_robot[2];

    while (angle_error < -180)
        angle_error += 360;
    while (angle_error > 180)
        angle_error -= 360;

    float angle_output = PIDCalculate(&angles_pid, angle_error, 25);

    float vector_output[2];

    vector_output[0] = vx * sin(pos_robot[2] * DEG2RAD) - vy * cos(pos_robot[2] * DEG2RAD);
    vector_output[1] = vx * cos(pos_robot[2] * DEG2RAD) + vy * sin(pos_robot[2] * DEG2RAD);

    motion.vel_x = vector_output[0];
    motion.vel_y = vector_output[1];
    motion.vel_th = angle_output;
    motion.acceleration = 3;
    ManualMotion(&motion, ret);
}

void ManualMotionControl(motion_data_t *data, motion_return_t *ret)
{
}

void ManualMotionPosition(int8_t _vx, int8_t _vy, int8_t _vth, motion_return_t *ret)
{

    motion_data_t motion_vector;
    float vector_out[2];
    vector_out[0] = _vx * sin(pos_robot[2] * DEG2RAD) - _vy * cos(pos_robot[2] * DEG2RAD);
    vector_out[1] = _vx * cos(pos_robot[2] * DEG2RAD) + _vy * sin(pos_robot[2] * DEG2RAD);

    motion_vector.vel_x = vector_out[0];
    motion_vector.vel_y = vector_out[1];
    motion_vector.vel_th = _vth;
    // printf("Velocity datas: %f %f %f\n", motion_vector.vel_x, motion_vector.vel_y, motion_vector.vel_th);

    ManualMotion(&motion_vector, ret);
}

/* Position Control */
bool PositionAngularMotion(motion_data_t *data, motion_return_t *ret)
{
    PID_t position_pid;
    PID_t angles_pid;

    PIDInit(&position_pid, 0.65, 0, 0.4);
    PIDInit(&angles_pid, 0.55, 0, 0);

    /* Error Position X */
    error[0] = data->target_x - pos_robot[0];

    /* Error Position Y */
    error[1] = data->target_y - pos_robot[1];

    /* Error Position */
    error[3] = sqrt(error[0] * error[0] + error[1] * error[1]);

    /* Error theta */
    error[2] = data->target_th - pos_robot[2];
    while (error[2] < -180)
        error[2] += 360;
    while (error[2] > 180)
        error[2] -= 360;

    /* Output Position */
    output[3] = PIDCalculate(&position_pid, error[3], data->vel_position);
    /* Output Theta */
    output[2] = PIDCalculate(&angles_pid, error[2], data->vel_th);

    // printf("Output: %f %f\n", output[0], output[1]);

    /* Output vel X */
    output[0] = output[3] * cos(atan2(error[1], error[0]));
    output[1] = output[3] * sin(atan2(error[1], error[0]));

    ManualMotionPosition(output[0], output[1], output[2], ret);

    printf("vx: %d, vy: %d, vth: %d\n", ret->vx, ret->vy, ret->vth);

    if (error[3] < 20 && fabs(error[2] < 9))
    {
        printf("Position Reached");
        return true;
    }
    else
        return false;
}

bool MotionAroundPoint(motion_data_t *data, motion_return_t *ret)
{
    PID_t position_pid;
    PID_t angles_pid;
    PID_t arc_pid;

    PIDInit(&position_pid, 0.45, 0, 0);
    PIDInit(&angles_pid, 0.35, 0, 0);
    PIDInit(&arc_pid, 0.35, 0, 0);

    /* Error X Position */
    error[0] = data->target_x - pos_robot[0];
    /* Error Y Position */
    error[1] = data->target_y - pos_robot[1];
    /* Error Position */
    error[3] = sqrt(error[0] * error[0] + error[1] * error[1]);
    /* Error theta */
    error[2] = data->target_th - pos_robot[2];
    while (error[2] < -180)
        error[2] += 360;
    while (error[2] > 180)
        error[2] -= 360;

    float error_arc_angle = data->target_th - RobotAngletoPoint(data->target_x, data->target_y);
    while (error_arc_angle < -180)
        error_arc_angle += 360;
    while (error_arc_angle > 180)
        error_arc_angle -= 360;

    float circumfence = 2 * M_PI * Pythagoras(data->target_x, data->target_y, pos_robot[0], pos_robot[1]);
    /* Divided by 360 */
    float arc_error = circumfence * error_arc_angle * 0.00277777777;

    /* Output Position */
    output[0] = PIDCalculate(&position_pid, error[3], data->vel_position);
    /* Output Theta */
    output[1] = PIDCalculate(&angles_pid, error[2], data->vel_th);
    /* Output Arc */
    output[2] = PIDCalculate(&arc_pid, arc_error, data->vel_position);

    output_buffer[0] = output[0] * cos(atan2(error[1], error[0])) + output[2] * cos(atan2(error[1], error[0]) + M_PI_2);
    output_buffer[1] = output[0] * sin(atan2(error[1], error[0])) + output[2] * sin(atan2(error[1], error[0]) + M_PI_2);

    ManualMotionPosition(output_buffer[0], output_buffer[1], output[1], ret);

    if (fabs(error[3]) < 20 && fabs(error[2]) < 9 && fabs(arc_error) < 10)
    {
        printf("Position Reached");
        return true;
    }
    else
        return false;
}

bool MotionAroundBall(motion_data_t *data, motion_return_t *ret)
{
    data->target_x = ball_on_field[0];
    data->target_y = ball_on_field[1];
    if (ball_status)    
        return MotionAroundPoint(data, ret);

    return false;
}

void ResetVelocity(motion_data_t *data, motion_return_t *ret)
{
    ret->vx = 0;
    ret->vy = 0;
    ret->vth = 0;
}
