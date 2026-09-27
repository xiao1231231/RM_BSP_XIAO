/**
 * @file    alg_matrix_cmsis_compat.c
 * @brief   两个三角函数的实现（为什么只剩这些，见 .h）
 */

#include "alg_matrix_cmsis_compat.h"

/* CMSIS-DSP 的这两个函数是【查表实现】，要求输入落在 [0, 2π) 内。
 * 本工程调用处传的都是归一化过的角度，直接转发标准库即可 ——
 * 行为更宽松（支持任意角度），不会出错。 */

float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}

float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}
