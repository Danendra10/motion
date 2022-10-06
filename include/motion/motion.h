#ifndef MOTION_H_
#define MOTION_H_


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "goalkeeper/goalkeeper.h"
// #include <vector>
// #include <stdint.h>

    typedef struct motion_data_tag
    {
        short int vel_x;
        short int vel_y;
        short int vel_th;
        short int acceleration;

        uint16_t target_x;
        uint16_t target_y;
        int8_t target_th;
    } motion_data_t;

    typedef struct motion_return_tag
    {
        short int vx;
        short int vy;
        short int vth;
    } motion_return_t;

    typedef struct robot_data_tag
    {
        int16_t robot_x[6];
        int16_t robot_y[6];
        int16_t robot_th[6];
        uint8_t robot_num;
        float ball_x;
        float ball_y;
        int8_t obs_on_field[60];
        uint8_t game_status;
    } robot_data_t;

    typedef struct
    {
        uint8_t status;
        float angle;
        float distance;
        float pos_x;
        float pos_y;

        struct
        {
            uint8_t status;
            float pos_x;
            float pos_y;
        } detection[60];
    } ObstacleDetection;

    /**
     * Variabels:
     * error[0] = error x
     * error[1] = error y
     * error[2] = error sudut
     * error[3] = error posisi
     */
    float error[4];
    float output[4];

    /* set the motion data */
    void ManualMotion(motion_data_t *data, motion_return_t *ret);

    /* set the motion data with pid position control */
    void PositionAngularMotion(motion_data_t *data, motion_return_t *ret, robot_data_t *robot_data);

    /* Reset Motion */
    void ResetVelocity(motion_data_t *data, motion_return_t *ret);

    float pythagoras(float x1, float y1, float x2, float y2);

    float RobotAngletoPoint(int16_t x, int16_t y, robot_data_t *robot_data);

    //---Obstacle Avoidance
    //=====================
    ObstacleDetection ObstacleCheck(float theta, float theta_thresh, float dist, uint8_t _ignore_friend, robot_data_t *robot_data);

#endif