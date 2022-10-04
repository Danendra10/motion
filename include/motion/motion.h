#ifndef MOTION_H_
#define MOTION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>

    typedef struct motion_data_tag
    {
        short int vel_x;
        short int vel_y;
        short int vel_th;
    } motion_data_t;

    void JalanManual(motion_data_t *data);
#ifdef __cplusplus
}
#endif
#endif