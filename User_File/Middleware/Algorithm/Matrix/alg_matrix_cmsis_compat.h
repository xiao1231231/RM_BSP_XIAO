/**
 * @file    alg_matrix_cmsis_compat.h
 * @brief   只提供 VQF 算法包需要的 CMSIS-DSP 替代品
 *
 * @note  ★ 为什么需要这个文件
 *        VQF 的 alg_basic.h 原本 #include "arm_math.h"（整个 CMSIS-DSP），
 *        但整包（alg_basic / alg_matrix / alg_quaternion / alg_filter_vqf）
 *        统计下来只用到这几样东西：
 *            arm_sin_f32 / arm_cos_f32   三角函数（alg_matrix.cpp、alg_quaternion.cpp）
 *            float32_t / PI              类型和常量
 *            memcpy                      alg_matrix.h 大量使用
 *        为这几样引入几百 KB 的 CMSIS-DSP 不划算，所以这里按 CMSIS-DSP 的
 *        【原样接口】实现前两个，再把 alg_basic.h 的 arm_math.h 换成这个头文件。
 *
 * @note  ★ 名字里的 matrix 是历史原因 —— 早先这里还实现过一套矩阵运算，
 *        那部分随算法换代已经删除。现在它只提供三角函数和类型别名。
 */

#ifndef ALG_MATRIX_CMSIS_COMPAT_H
#define ALG_MATRIX_CMSIS_COMPAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
/* ★ 必须保留：alg_matrix.h 用 memcpy，而它原来是从 arm_math.h 间接拿到声明的 */
#include <string.h>

/** 角度换算常量（原来来自 arm_math.h） */
#ifndef PI
#define PI 3.14159265358979f
#endif

/** CMSIS-DSP 的浮点类型别名 */
typedef float float32_t;

/* ── 三角函数（CMSIS-DSP 同名接口，输入单位：弧度）──────────────── */

/** @brief cos */
float32_t arm_cos_f32(float32_t x);

/** @brief sin */
float32_t arm_sin_f32(float32_t x);

#ifdef __cplusplus
}
#endif

#endif /* ALG_MATRIX_CMSIS_COMPAT_H */
