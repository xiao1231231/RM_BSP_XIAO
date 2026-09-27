#ifndef SYSTEM_CALLBACK_H
#define SYSTEM_CALLBACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 集中分发 HAL 回调
 * @note  本文件【只做转发】，不含任何业务逻辑。
 *        所有 HAL 的 __weak 回调都在这里被覆盖，然后转给对应的 BSP 层。
 */
void System_Callback_Init(void);

/**
 * @brief SPI1 收发完成回调（由 bsp_spi 层通过函数指针调用）
 * @note  按片选分发给对应器件（目前只有 BMI088）。
 *        在 SPI_Init() 时作为回调注册进去，所以要能被 sys_attitude 取到地址。
 */
void SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer,
                   uint16_t Tx_Length, uint16_t Rx_Length);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CALLBACK_H */