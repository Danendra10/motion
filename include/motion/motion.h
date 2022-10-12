#ifndef MOTION_H_
#define MOTION_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

extern float pos_robot[3];
extern float ball_on_field[4];
extern uint8_t ball_status;

/**
 * @var output[0] = vx
 * @var output[1] = vy
 */
float output_buffer[4];

typedef struct motion_data_tag
{
    float vel_x;
    float vel_y;
    float vel_th;
    short int vel_position;
    short int acceleration;

    int16_t target_x;
    int16_t target_y;
    int8_t target_th;
    uint8_t distance_from_target;
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


motion_data_t motion_data;
motion_return_t motion_return;

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

void ManualMotionPosition(int8_t _vx, int8_t _vy, int8_t _vth, motion_return_t *ret);

void ManualMotionVector(uint16_t vx, uint16_t vy, int16_t angles_target, motion_return_t *ret);

/* set the motion data with pid position control */
bool PositionAngularMotion(motion_data_t *data, motion_return_t *ret);

/**
 * @param data.target_y  y_point
 * @param data.target_x  x_point
 * @param data.distance_from_target  distance from target x and y
 */
bool MotionAroundPoint(motion_data_t *data, motion_return_t *ret);

/**
 * @param data.target_y  y_ball
 * @param data.target_x  x_ball
 * @param data.distance_from_target  distance from ball
 * @param data.target_th  angle from ball
 *
 * @tparam data.vel_position  velocity position
 * @tparam data.vel_th  velocity theta
 */
bool MotionAroundBall(motion_data_t *data, motion_return_t *ret);

/* Reset Motion */
void ResetVelocity(motion_data_t *data, motion_return_t *ret);

float pythagoras(float x1, float y1, float x2, float y2);

float RobotAngletoPoint(int16_t x, int16_t y, robot_data_t *robot_data);

#endif