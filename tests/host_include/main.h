#ifndef DJI_MOTOR_HOST_MAIN_H
#define DJI_MOTOR_HOST_MAIN_H

#include <stdint.h>

/* 主机自检仅替换 CMSIS/HAL 边界，不模拟真实中断时序。 */
struct CAN_HandleTypeDef { void *Instance; };
struct Host_DWT { uint32_t CYCCNT; };
extern Host_DWT host_dwt;
#define DWT (&host_dwt)

inline uint32_t __get_PRIMASK() { return 0U; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
inline void __DMB() {}

#endif
