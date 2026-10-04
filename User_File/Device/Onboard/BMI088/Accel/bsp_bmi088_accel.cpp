/**
 * @file    bsp_bmi088_accel.cpp
 * @brief   BMI088 加速度计实现（数据读取 + 温度读取 + TIM10 恒温控制）
 *
 * @note    来源：H7_BSP（USTC-RoboWalker / yssickjgd）。
 *          移植改动清单见 bsp_bmi088_accel.h 顶部注释。
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel.h"

#include "alg_basic.h"
#include <math.h>
#include <stddef.h>

/* Private variables ---------------------------------------------------------*/

/** 标准重力加速度（上海地区的常用取值） */
const float GRAVITY_ACCELERATION = 9.8015f;

/** 温度合理区间 + 单帧最大跳变：越界的判野值，防止一个坏帧污染零偏估计 */
static constexpr float BMI088_TEMPERATURE_MIN = -40.0f;
static constexpr float BMI088_TEMPERATURE_MAX = 85.0f;
static constexpr float BMI088_TEMPERATURE_MAX_STEP = 5.0f;
static uint32_t BMI088_Temperature_Outlier_Counter = 0U;
static uint32_t BMI088_Accel_Invalid_Counter = 0U;

/* Function prototypes -------------------------------------------------------*/

uint32_t Class_BMI088_Accel::Get_Temperature_Outlier_Counter() const
{
    return (BMI088_Temperature_Outlier_Counter);
}

uint32_t Class_BMI088_Accel::Get_Accel_Invalid_Counter() const
{
    return (BMI088_Accel_Invalid_Counter);
}

bool Class_BMI088_Accel::Get_Temperature_Valid_Flag() const
{
    return Get_Temperature_State().Data_Valid;
}

uint32_t Class_BMI088_Accel::Get_Temperature_Age_Us() const
{
    return Get_Temperature_State().Age_Us;
}

/**
 * @brief 取一份温度快照：值 + 上次有效时刻 + 年龄 + 能不能信
 *
 * @note  ★ "年龄"是关键：温度是 128ms 才读一次的慢变量，如果 SPI 挂了、
 *        温度一直不更新，只看数值是看不出来的（它还是最后一次的旧值）。
 *        Data_Valid = 值有效 且 年龄没超时，这样上层才敢用它。
 */
Struct_BMI088_Accel_Temperature_State Class_BMI088_Accel::Get_Temperature_State() const
{
    Struct_BMI088_Accel_Temperature_State state = {};
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    state.Now_Timestamp_Us = Sys_Get_Micros();
    state.Last_Valid_Timestamp_Us = Temperature_Last_Valid_Timestamp;
    state.Temperature = Now_Temperature;
    const bool raw_valid = Temperature_Valid_Flag;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (raw_valid)
    {
        const uint64_t age_us = state.Now_Timestamp_Us >= state.Last_Valid_Timestamp_Us
                                    ? state.Now_Timestamp_Us - state.Last_Valid_Timestamp_Us
                                    : UINT64_MAX;
        state.Age_Us = age_us <= UINT32_MAX ? static_cast<uint32_t>(age_us)
                                            : UINT32_MAX;
        state.Data_Valid = age_us <= TEMPERATURE_STALE_TIMEOUT_US;
    }
    return state;
}

/**
 * @brief 初始化加速度计
 *
 * @note  每一步最多重试 5 次，失败返回 false —— 上层必须检查。
 */
bool Class_BMI088_Accel::Init()
{
    const uint8_t max_attempts = 5;
    Valid_Flag = false;

    SPI_Manage_Object = &SPI1_Manage_Object;

    CS_GPIO_Port = CS1_ACCEL_GPIO_Port;
    CS_Pin = CS1_ACCEL_Pin;
    Activate_Pin_State = GPIO_PIN_RESET;

    /* ── 恒温初始化（照 basic_framework 的 C 板参数，见 .h 里的换算说明）── */
    PID_Temperature.Init(HEATER_KP, HEATER_KI, 0.0f,
                         0.0f,
                         HEATER_I_OUT_MAX,
                         HEATER_OUT_MAX,
                         HEATER_D_T);
    /* 启动加热 PWM，Compare 从 0 开始（上电不加热，等 PID 接管）。
     * 启动失败就把恒温关掉 —— IMU 没有恒温也能跑，只是温漂大一点。 */
    if (HAL_TIM_PWM_Start(BMI088_HEAT_TIM, BMI088_HEAT_CHANNEL) != HAL_OK)
    {
        Heater_Enable = false;
        __HAL_TIM_SET_COMPARE(BMI088_HEAT_TIM, BMI088_HEAT_CHANNEL, 0U);
    }
    else
    {
        Heater_Enable = true;
        __HAL_TIM_SET_COMPARE(BMI088_HEAT_TIM, BMI088_HEAT_CHANNEL, 0U);
    }

    uint8_t res;

    // 检测通信是否正常（加速度计的芯片 ID 是 0x1E）
    Register.ACC_CHIP_ID_RO = 0x00;
    for (uint8_t attempt = 0; attempt < max_attempts && Register.ACC_CHIP_ID_RO != 0x1e; attempt++)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_CHIP_ID_RO));
        Sys_Delay_S(0.1f);
    }
    if (Register.ACC_CHIP_ID_RO != 0x1e)
    {
        return false;
    }

    // 软重启
    res = 0xb6;
    Write_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_SOFTRESET_WO), &res);
    Sys_Delay_S(0.1f);

    // 重启后再确认一次通信
    Register.ACC_CHIP_ID_RO = 0x00;
    for (uint8_t attempt = 0; attempt < max_attempts && Register.ACC_CHIP_ID_RO != 0x1e; attempt++)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_CHIP_ID_RO));
        Sys_Delay_S(0.1f);
    }
    if (Register.ACC_CHIP_ID_RO != 0x1e)
    {
        return false;
    }

    // 逐条写配置 + 读回校验（哨兵值取反，保证"值为 0 的配置"也真的被写过）
    for (uint8_t i = 0; i < BMI088_ACCEL_INIT_INSTRUCTION_NUM; i++)
    {
        uint8_t *readback = ((uint8_t *) (&Register)) + BMI088_ACCEL_REGISTER_CONFIG[i][0];
        *readback = ~BMI088_ACCEL_REGISTER_CONFIG[i][1];
        for (uint8_t attempt = 0; attempt < max_attempts && *readback != BMI088_ACCEL_REGISTER_CONFIG[i][1]; attempt++)
        {
            Write_Single_Register(BMI088_ACCEL_REGISTER_CONFIG[i][0], &BMI088_ACCEL_REGISTER_CONFIG[i][1]);
            Sys_Delay_S(0.1f);

            Read_Single_Register(BMI088_ACCEL_REGISTER_CONFIG[i][0]);
            Sys_Delay_S(0.1f);
        }
        if (*readback != BMI088_ACCEL_REGISTER_CONFIG[i][1])
        {
            return false;
        }
    }

    // 预读一次数据：让第一帧就是有效值，而不是开机瞬间的 0
    Read_Multi_Register(offsetof(Struct_BMI088_Accel_Register, ACC_X_RO), 6);
    Sys_Delay_S(0.1f);
    return true;
}

/**
 * @brief SPI 接收回调：解析刚收回来的一笔（加速度数据 / 温度 / 普通寄存器）
 */
void Class_BMI088_Accel::SPI_RxCpltCallback()
{
    uint8_t spi_init_address = SPI_Manage_Object->Tx_Buffer[0] & ~BMI088_ACCEL_READ_MASK;

    /* 加速度计的读时序比陀螺多一个保留字节（见头文件 BMI088_ACCEL_SPI_RX_RESERVED）：
     * 有效数据从 Rx_Buffer[2] 开始，所以这里 +1 */
    memcpy((uint8_t *) (&Register) + spi_init_address, &SPI_Manage_Object->Rx_Buffer[1 + BMI088_ACCEL_SPI_RX_RESERVED], SPI_Manage_Object->Rx_Buffer_Length);

    if (spi_init_address == offsetof(Struct_BMI088_Accel_Register, ACC_X_RO))
    {
        // 加速度数据：int16 → m/s²（量程 24g 时 1 LSB = 1.5mg）
        Vector_Raw_Accel[0][0] = (float) (Register.ACC_X_RO) / 32768.0f * (1 << (BMI088_ACCEL_RANGE + 1)) * 1.5f * GRAVITY_ACCELERATION;
        Vector_Raw_Accel[1][0] = (float) (Register.ACC_Y_RO) / 32768.0f * (1 << (BMI088_ACCEL_RANGE + 1)) * 1.5f * GRAVITY_ACCELERATION;
        Vector_Raw_Accel[2][0] = (float) (Register.ACC_Z_RO) / 32768.0f * (1 << (BMI088_ACCEL_RANGE + 1)) * 1.5f * GRAVITY_ACCELERATION;

        if (Basic_Math_Is_Invalid_Float(Vector_Raw_Accel[0][0]) || Basic_Math_Is_Invalid_Float(Vector_Raw_Accel[1][0]) || Basic_Math_Is_Invalid_Float(Vector_Raw_Accel[2][0]))
        {
            Valid_Flag = false;
            BMI088_Accel_Invalid_Counter++;
        }
        else
        {
            Valid_Flag = true;
        }
    }
    else if (spi_init_address == offsetof(Struct_BMI088_Accel_Register, TEMP_MSB_RO))
    {
        // 温度：12 位，0.125°C/LSB，23°C 偏移
        int16_t raw_temperature = static_cast<int16_t>(Register.TEMP_MSB_RO << 3 | Register.TEMP_LSB_RO >> 5);
        if ((raw_temperature & 0x0400) != 0)
        {
            raw_temperature -= 1 << 11;      // 12 位补码 → 有符号
        }
        const float temperature = 23.0f + static_cast<float>(raw_temperature) * 0.125f;

        /* 野值判定分两类：
         *  · 物理越界（-40~85°C 之外）—— 一定是坏数据
         *  · 单帧跳变超过 5°C —— 温度是慢变量，突变必是坏帧
         * 判野值后不是简单丢弃，而是进入"重新采信"流程（见下），
         * 避免一次干扰就让温度永久不可信。 */
        const bool physical_outlier = temperature < BMI088_TEMPERATURE_MIN || temperature > BMI088_TEMPERATURE_MAX;
        const bool step_outlier = Temperature_Valid_Flag &&
                                  fabsf(temperature - Now_Temperature) > BMI088_TEMPERATURE_MAX_STEP;
        if (physical_outlier || step_outlier)
        {
            BMI088_Temperature_Outlier_Counter++;
            Temperature_Valid_Flag = false;
            Temperature_Rebase_Count = 0U;
        }
        else if (Temperature_Valid_Flag)
        {
            Now_Temperature = temperature;
            Temperature_Last_Valid_Timestamp = Sys_Get_Micros();
        }
        else
        {
            /* 重新采信：连续 3 帧都在同一个值附近（互相差异不超过 5°C），
             * 才认为"温度真的变了"而不是"坏了" */
            if (Temperature_Rebase_Count == 0U ||
                fabsf(temperature - Temperature_Rebase_Candidate) > BMI088_TEMPERATURE_MAX_STEP)
            {
                Temperature_Rebase_Candidate = temperature;
                Temperature_Rebase_Count = 1U;
            }
            else
            {
                Temperature_Rebase_Candidate = temperature;
                Temperature_Rebase_Count++;
            }

            if (Temperature_Rebase_Count >= TEMPERATURE_REBASE_SAMPLE_COUNT)
            {
                Now_Temperature = Temperature_Rebase_Candidate;
                Temperature_Last_Valid_Timestamp = Sys_Get_Micros();
                Temperature_Rebase_Count = 0U;
                __DMB();
                Temperature_Valid_Flag = true;
            }
        }
    }
}

/** @brief 请求一笔加速度数据（8 字节收发：2 发 + 6 收） */
uint8_t Class_BMI088_Accel::SPI_Request_Accel()
{
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(offsetof(Struct_BMI088_Accel_Register, ACC_X_RO) | BMI088_ACCEL_READ_MASK);

    return SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                                     Activate_Pin_State, tx_data, sizeof(tx_data), 6);
}

/** @brief 请求一笔温度数据（4 字节收发：2 发 + 2 收） */
uint8_t Class_BMI088_Accel::SPI_Request_Temperature()
{
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(offsetof(Struct_BMI088_Accel_Register, TEMP_MSB_RO) | BMI088_ACCEL_READ_MASK);

    return SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                                     Activate_Pin_State, tx_data, sizeof(tx_data), 2);
}

/**
 * @brief IMU 恒温控制（500Hz 调用，见 .h 说明）
 *
 * @note  温度数据无效/过期时立即停热 —— 温度不可信时宁可不加热，
 *        也不能让 PID 拿着旧温度一直输出大功率。
 */
void Class_BMI088_Accel::Heater_Control(const float &__Now_Temperature)
{
    if (!Heater_Enable)
    {
        return;
    }

    const Struct_BMI088_Accel_Temperature_State state = Get_Temperature_State();
    if (!state.Data_Valid || Basic_Math_Is_Invalid_Float(state.Temperature))
    {
        __HAL_TIM_SET_COMPARE(BMI088_HEAT_TIM, BMI088_HEAT_CHANNEL, 0U);
        Heater_PWM_Compare = 0U;
        return;
    }

    PID_Temperature.Set_Target(HEATER_TARGET_TEMPERATURE);
    PID_Temperature.Set_Now(__Now_Temperature);
    PID_Temperature.TIM_Calculate_PeriodElapsedCallback();

    /* PID 输出即 PWM Compare（0~9999），负值截为 0（只能加热不能制冷） */
    const float out = Basic_Math_Constrain(PID_Temperature.Get_Out(), 0.0f, HEATER_OUT_MAX);
    Heater_PWM_Compare = (uint32_t)out;
    __HAL_TIM_SET_COMPARE(BMI088_HEAT_TIM, BMI088_HEAT_CHANNEL, Heater_PWM_Compare);
}

/**
 * @brief 读单个寄存器
 *
 * @note  ★ 异步：数据在 SPI 接收回调里填进 Register。
 *        加速度计读时序要发 2 字节（地址 + 保留字节），所以 Tx=2、Rx=1。
 */
void Class_BMI088_Accel::Read_Single_Register(const uint8_t &Register_Address) const
{
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(Register_Address | BMI088_ACCEL_READ_MASK);

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, sizeof(tx_data), 1);
}

/** @brief 读多个寄存器（异步） */
void Class_BMI088_Accel::Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const
{
    constexpr uint32_t tx_length = 2U;
    if (Rx_Length > (SPI_BUFFER_SIZE - tx_length))
    {
        return;
    }
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(Register_Address | BMI088_ACCEL_READ_MASK);

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, sizeof(tx_data), static_cast<uint16_t>(Rx_Length));
}

/** @brief 写单个寄存器 */
void Class_BMI088_Accel::Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const
{
    const uint8_t tx_data[2] = {Register_Address, Tx_Data_Buffer[0]};

    SPI_Transmit_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                      Activate_Pin_State, tx_data, sizeof(tx_data));
}

/** @brief 写多个寄存器 */
void Class_BMI088_Accel::Write_Multi_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer, const uint32_t &Tx_Length) const
{
    if (Tx_Length > (SPI_BUFFER_SIZE - 1U))
    {
        return;
    }
    uint8_t tx_data[SPI_BUFFER_SIZE] = {};
    tx_data[0] = Register_Address;
    memcpy(&tx_data[1], Tx_Data_Buffer, Tx_Length);

    SPI_Transmit_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin, Activate_Pin_State,
                      tx_data, static_cast<uint16_t>(Tx_Length + 1U));
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
