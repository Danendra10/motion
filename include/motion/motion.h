#ifndef MOTION_H_
#define MOTION_H_


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "goalkeeper/goalkeeper.h"
// #include <vector>
// #include <stdint.h>

    extern int16_t pos_robot[3];
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


    /**
     * Variabels:
     * error[0] = error x
     * error[1] = error y
     * error[2] = error sudut
     * error[3] = error posisi
     */
    float error[4];
    float output[4];

    /* set the motion data with velocity control
        * @param data: pointer to motion data:
        * @param data.vel_x: velocity x
        * @param data.vel_y: velocity y
        * @param data.vel_th: velocity theta
        * @param data.acceleration: acceleration
        * @param data.target_x: target x
        * @param data.target_y: target y
        * @param data.target_th: target theta
        * @param ret: pointer to motion return:
        * @param ret.vx: velocity x
        * @param ret.vy: velocity y
        * @param ret.vth: velocity theta
    */
    void ManualMotion(motion_data_t *data, motion_return_t *ret);

    /* set the motion data with pid position control */
    void PositionAngularMotion(motion_data_t *data, motion_return_t *ret);

    /* Reset Motion */
    void ResetVelocity(motion_data_t *data, motion_return_t *ret);

    float pythagoras(float x1, float y1, float x2, float y2);

    float RobotAngletoPoint(int16_t x, int16_t y, robot_data_t *robot_data);

#endif