#include "motion/motion.h"
#include "motion/pid.h"
#include <ros/ros.h>
#include <angles/angles.h>

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

//------------Basic Calculation------------//
float pythagoras(float x1, float y1, float x2, float y2) { return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)); }

float RobotAngletoPoint(int16_t x, int16_t y, robot_data_t *robot_data) { return atan2(y - robot_data->robot_y[robot_data->robot_num], x - robot_data->robot_x[robot_data->robot_num]); }

//-----------Obstacle Avoidance-----------//
ObstacleDetection ObstacleCheck(float theta, float theta_thresh, float dist, uint8_t _ignore_friend, robot_data_t *robot_data)
{
    ObstacleDetection obs_data;

    static uint8_t obs_counter;
    static uint8_t obs_start;
    static uint8_t obs_final;

    obs_data.status = 0;
    obs_data.distance = dist;

    // Set every 60 iteration is not an obstacle for init
    for (uint8_t i = 0; i < 60; i++)
        obs_data.detection[i].status = 0;

    uint16_t init_index = (theta - theta_thresh) * 0.166666667;
    uint16_t final_index = (theta + theta_thresh) * 0.166666667;

    while (init_index < 0)
        init_index += 60;
    while (init_index > 59)
        init_index -= 60;
    while (final_index < 0)
        final_index += 60;
    while (final_index > 59)
        final_index -= 60;

    std::vector<float> obs_dist;
    std::vector<uint8_t> obs_index;

    if (init_index > final_index)
    {
        obs_counter = 0;
        for (uint8_t i = init_index; i <= final_index; i++)
        {
            /* Check if the obstacle is in the range of our dist thresh */
            if (robot_data->obs_on_field[i] < dist && i != final_index)
            {
                obs_data.status = 1;
                obs_data.angle = i * 6;
                obs_data.distance = robot_data->obs_on_field[i];
                obs_data.pos_x = robot_data->robot_x[robot_data->robot_num] + robot_data->obs_on_field[i] * cos(angles::from_degrees(i * 6));
                obs_data.pos_y = robot_data->robot_y[robot_data->robot_num] + robot_data->obs_on_field[i] * sin(angles::from_degrees(i * 6));

                obs_data.detection[i].status = 1;
                obs_data.detection[i].pos_x = obs_data.pos_x;
                obs_data.detection[i].pos_y = obs_data.pos_y;

                obs_counter++;

                if (obs_counter == 1)
                    obs_start = i;
            }
            else
            {
                if (obs_counter >= 2)
                {
                    obs_final = i - 1;
                    obs_data.status = 1;
                    obs_data.angle = (obs_start + obs_final) * 6;

                    obs_data.pos_x = 0;
                    obs_data.pos_y = 0;

                    for (uint8_t j = obs_start; j <= obs_final; j++)
                    {
                        obs_data.pos_x += obs_data.detection[j].pos_x;
                        obs_data.pos_y += obs_data.detection[j].pos_y;
                    }

                    obs_data.pos_x /= (obs_final - obs_start);
                    obs_data.pos_y /= (obs_final - obs_start);

                    obs_data.pos_x += 25 * cosf(obs_data.angle * M_PI * 0.005555556);
                    obs_data.pos_y += 25 * sinf(obs_data.angle * M_PI * 0.005555556);

                    float buffer_obs_dist;

                    buffer_obs_dist = pythagoras(robot_data->robot_x[robot_data->robot_num], robot_data->robot_y[robot_data->robot_num], obs_data.pos_x, obs_data.pos_y);

                    obs_dist.push_back(buffer_obs_dist);
                    obs_index.push_back((int)(obs_start + obs_final) * 0.5);
                }
                obs_counter = 0;
            }
        }
    }
    else if (init_index > final_index)
    {
        obs_counter = 0;
        for (uint8_t i = init_index; i <= final_index; i++)
        {
            /* Check if the obstacle is in the range of our dist thresh */
            if (robot_data->obs_on_field[i] < dist && i != final_index)
            {
                obs_data.status = 1;
                obs_data.angle = i * 6;
                obs_data.distance = robot_data->obs_on_field[i];
                obs_data.pos_x = robot_data->robot_x[robot_data->robot_num] + robot_data->obs_on_field[i] * cos(angles::from_degrees(i * 6));
                obs_data.pos_y = robot_data->robot_y[robot_data->robot_num] + robot_data->obs_on_field[i] * sin(angles::from_degrees(i * 6));

                obs_data.detection[i].status = 1;
                obs_data.detection[i].pos_x = obs_data.pos_x;
                obs_data.detection[i].pos_y = obs_data.pos_y;

                obs_counter++;

                if (obs_counter == 1)
                    obs_start = i;
            }
            else
            {
                if (obs_counter >= 2)
                {
                    obs_final = i - 1;
                    obs_data.status = 1;
                    obs_data.angle = (obs_start + obs_final) * 6;

                    obs_data.pos_x = 0;
                    obs_data.pos_y = 0;

                    for (uint8_t j = obs_start; j <= obs_final; j++)
                    {
                        obs_data.pos_x += obs_data.detection[j].pos_x;
                        obs_data.pos_y += obs_data.detection[j].pos_y;
                    }

                    obs_data.pos_x /= (obs_final - obs_start);
                    obs_data.pos_y /= (obs_final - obs_start);

                    obs_data.pos_x += 25 * cosf(obs_data.angle * M_PI * 0.005555556);
                    obs_data.pos_y += 25 * sinf(obs_data.angle * M_PI * 0.005555556);

                    float buffer_obs_dist;

                    buffer_obs_dist = pythagoras(robot_data->robot_x[robot_data->robot_num], robot_data->robot_y[robot_data->robot_num], obs_data.pos_x, obs_data.pos_y);

                    obs_dist.push_back(buffer_obs_dist);
                    obs_index.push_back((int)(obs_start + obs_final) * 0.5);
                }
                obs_counter = 0;
            }
            if (robot_data->obs_on_field[i] < dist && i == final_index)
                obs_final = i;
        }
        for (uint8_t i = init_index; i <= 59; i++)
        {
            /* Check if the obstacle is in the range of our dist thresh */
            if (robot_data->obs_on_field[i] < dist && i != 59)
            {
                obs_data.status = 1;
                obs_data.angle = i * 6;
                obs_data.distance = robot_data->obs_on_field[i];
                obs_data.pos_x = robot_data->robot_x[robot_data->robot_num] + robot_data->obs_on_field[i] * cos(angles::from_degrees(i * 6));
                obs_data.pos_y = robot_data->robot_y[robot_data->robot_num] + robot_data->obs_on_field[i] * sin(angles::from_degrees(i * 6));

                obs_data.detection[i].status = 1;
                obs_data.detection[i].pos_x = obs_data.pos_x;
                obs_data.detection[i].pos_y = obs_data.pos_y;

                obs_counter++;

                if (obs_counter == 1)
                    obs_start = i;
            }
            else
            {
                if (obs_counter >= 2)
                {
                    obs_final = i - 1;
                    obs_data.status = 1;
                    obs_data.angle = (obs_start + obs_final) * 6;

                    obs_data.pos_x = 0;
                    obs_data.pos_y = 0;

                    for (uint8_t j = obs_start; j <= obs_final; j++)
                    {
                        obs_data.pos_x += obs_data.detection[j].pos_x;
                        obs_data.pos_y += obs_data.detection[j].pos_y;
                    }

                    obs_data.pos_x /= (obs_final - obs_start);
                    obs_data.pos_y /= (obs_final - obs_start);

                    obs_data.pos_x += 25 * cosf(obs_data.angle * M_PI * 0.005555556);
                    obs_data.pos_y += 25 * sinf(obs_data.angle * M_PI * 0.005555556);

                    float buffer_obs_dist;

                    buffer_obs_dist = pythagoras(robot_data->robot_x[robot_data->robot_num], robot_data->robot_y[robot_data->robot_num], obs_data.pos_x, obs_data.pos_y);

                    obs_dist.push_back(buffer_obs_dist);
                    obs_index.push_back((int)(obs_start + obs_final) * 0.5);
                }
                obs_counter = 0;
            }
            if (robot_data->obs_on_field[i] < dist && i == 59)
                obs_final = i;
        }
    }
    /* Full Scan */
    else if (init_index == final_index)
    {
        
    }

    return obs_data;
}