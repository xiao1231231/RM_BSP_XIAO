#include "gimbal_remote.h"
#include "gimbal_config.h"

#include <algorithm>
#include <cmath>

float Gimbal_Remote_Map_Yaw_Rate_Radps(const Struct_DT7_Data &remote)
{
    if (remote.Valid == 0U || remote.Switch_Left != DT7_SWITCH_MIDDLE)
    {
        return 0.0f;
    }

    const int channel = remote.Channel[2];
    const int magnitude = std::abs(channel);
    if (magnitude <= Gimbal_Config::Remote_Deadband_Channel)
    {
        return 0.0f;
    }

    const float normalized = static_cast<float>(std::min(magnitude, 660) -
                                                 Gimbal_Config::Remote_Deadband_Channel) /
                             (660.0f - Gimbal_Config::Remote_Deadband_Channel);
    const float direction = channel > 0 ? Gimbal_Config::Remote_Yaw_Direction
                                        : -Gimbal_Config::Remote_Yaw_Direction;
    return direction * normalized * Gimbal_Config::Remote_Max_Yaw_Rate_Deg_Per_S *
           (3.1415926f / 180.0f);
}
