#include "bsp_bmi088_mounting.h"
#include "alg_filter_vqf.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

static constexpr float Rad_To_Deg = 57.29577951f;

static Class_Matrix_f32<3, 1> Vector(float x, float y, float z)
{
    const float values[3] = {x, y, z};
    return Class_Matrix_f32<3, 1>(values);
}

static void Check_Near(float actual, float expected, float tolerance, const char *name)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance)
    {
        std::fprintf(stderr, "%s: expected %.6f, got %.6f\n", name, expected, actual);
        std::exit(EXIT_FAILURE);
    }
}

static void Check_Face_Up_Startup()
{
    for (int boot = 0; boot < 12; ++boot)
    {
        Class_Filter_VQF filter;
        filter.Init(Struct_VQF_Parameter{}, 0.0005f, 0.004f);
        const float direction = (15.0f + 30.0f * boot) / Rad_To_Deg;
        const auto accel = Vector(0.00981f * std::cos(direction),
                                  0.00981f * std::sin(direction), 9.809995f);
        for (int sample = 0; sample < 4000; ++sample)
        {
            filter.Update_Gyro(BMI088_Mounting::Sensor_To_Body(Vector(0, 0, 0)));
            if (sample % 8 == 0)
            {
                filter.Update_Accel(BMI088_Mounting::Sensor_To_Body(accel));
                const auto euler = filter.Get_Quaternion_6D().Get_Euler_Angle();
                Check_Near(euler[0][0] * Rad_To_Deg, 0.0f, 0.1f, "startup yaw");
                Check_Near(euler[1][0] * Rad_To_Deg, 0.0f, 0.1f, "startup pitch");
                Check_Near(euler[2][0] * Rad_To_Deg, 0.0f, 0.1f, "startup roll");
            }
        }
        filter.Reset();
        Check_Near(filter.Get_Quaternion_6D().Get_Euler_Angle()[0][0],
                   0.0f, 1.0e-6f, "reset yaw");
    }
}

static void Check_Bias_And_Yaw_Direction()
{
    const auto sensor_bias = Vector(0.001f, -0.002f, 0.003f);
    const auto body_bias = BMI088_Mounting::Sensor_To_Body(sensor_bias);
    const auto restored_bias = BMI088_Mounting::Body_To_Sensor(body_bias);
    Check_Near(body_bias[0][0], 0.001f, 1.0e-6f, "bias X");
    Check_Near(body_bias[1][0], -0.002f, 1.0e-6f, "bias Y");
    Check_Near(body_bias[2][0], 0.003f, 1.0e-6f, "bias Z");
    for (int axis = 0; axis < 3; ++axis)
    {
        Check_Near(restored_bias[axis][0], sensor_bias[axis][0], 1.0e-6f,
                   "Flash bias round trip");
    }

    for (int direction : {-1, 0, 1})
    {
        Struct_VQF_Parameter parameter;
        parameter.Motion_Bias_Estimation_Enable = false;
        parameter.Rest_Bias_Estimation_Enable = false;
        Class_Filter_VQF filter;
        filter.Init(parameter, 0.0005f, 0.004f);
        filter.Set_Bias_Estimate(body_bias);
        const auto gyro = sensor_bias + Vector(0, 0, direction * 90.0f / Rad_To_Deg);
        for (int sample = 0; sample < 2000; ++sample)
        {
            filter.Update_Gyro(BMI088_Mounting::Sensor_To_Body(gyro));
            if (sample % 8 == 0)
            {
                filter.Update_Accel(BMI088_Mounting::Sensor_To_Body(Vector(0, 0, 9.81f)));
            }
        }
        const auto euler = filter.Get_Quaternion_6D().Get_Euler_Angle();
        Check_Near(euler[0][0] * Rad_To_Deg, direction * 90.0f, 0.05f, "yaw direction");
        Check_Near(euler[1][0], 0.0f, 1.0e-5f, "yaw motion pitch");
        Check_Near(euler[2][0], 0.0f, 1.0e-5f, "yaw motion roll");
    }
}

int main()
{
    Check_Face_Up_Startup();
    Check_Bias_And_Yaw_Direction();
    std::puts("IMU mounting selftest passed: startup, reset, bias, signed yaw.");
}
