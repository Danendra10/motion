#include "motion/motion.h"

void JalanManual(motion_data_t *data)
{
    printf("Jalan Manual: %d %d %d\n", data->vel_x, data->vel_y, data->vel_th);
}