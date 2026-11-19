#ifndef BSP_BMI088_MOUNTING_H
#define BSP_BMI088_MOUNTING_H

#include "alg_matrix.h"

namespace BMI088_Mounting
{
// false：正常摆放，三轴原样使用；true：倒装，保留 X 轴并翻转 Y/Z。
constexpr bool Face_Down = false;

inline Class_Matrix_f32<3, 1> Sensor_To_Body(Class_Matrix_f32<3, 1> vector)
{
    if (Face_Down)
    {
        vector[1][0] = -vector[1][0];
        vector[2][0] = -vector[2][0];
    }
    return vector;
}

inline Class_Matrix_f32<3, 1> Body_To_Sensor(const Class_Matrix_f32<3, 1> &vector)
{
    return Sensor_To_Body(vector);
}
}

#endif
