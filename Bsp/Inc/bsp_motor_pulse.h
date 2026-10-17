#ifndef BSP_MOTOR_PULSE_H
#define BSP_MOTOR_PULSE_H

#include "types.h"
#include "cfg_motor.h"

/* ============================================================================
 * 四路步进脉冲输出（PUL）＋ 四路方向（DIR）
 *
 * 本层只知道"某一路通道"，不知道"左前轮/右后轮"——那是 App 层的语义。
 * 通道号直接取 Config 里的车体编号，两边永远一致。
 * ============================================================================ */

typedef enum
{
    MOTOR_CH_L1 = CFG_MOTOR_FL,   /* TIM1_CH1  PE9   左前 */
    MOTOR_CH_L2 = CFG_MOTOR_RL,   /* TIM1_CH2  PE11  左后 */
    MOTOR_CH_R1 = CFG_MOTOR_FR,   /* TIM2_CH1  PA0   右前 */
    MOTOR_CH_R2 = CFG_MOTOR_RR    /* TIM2_CH2  PA1   右后 */
} MotorChannel;

typedef enum
{
    MOTOR_DIR_CW  = 0,            /* DIR 引脚低电平 */
    MOTOR_DIR_CCW = 1             /* DIR 引脚高电平 */
} MotorDir;

/* ---------------------------------------------------------------------------
 * 初始化：打开 4 路 PWM 通道 + 高级定时器 TIM1 的主输出(MOE)，
 *         然后保持两路定时器停止，等 Bsp_MotorPulse_StartAll() 启动。
 *         （时钟、GPIO、AF 复用、时基、PWM 占空比全部由 CubeMX 生成代码完成）
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_Init(void);

/* 设置某一路通道的方向 */
void Bsp_MotorPulse_SetDir(MotorChannel ch, MotorDir dir);

/* ---------------------------------------------------------------------------
 * 设置某一路通道的脉冲频率(Hz)；hz == 0 表示"停这一侧时基"。
 *
 * ⚠️ 同侧两路共用一路定时器时基：设 L1 也会把 L2 的频率改掉（反之亦然），
 *    这是硬件决定的，不是 bug。想左右频率不同 → 左右各给一次即可。
 *
 * ⚠️⚠️ 调用顺序有硬约束：**必须先 Bsp_MotorPulse_StartAll() 再 SetFreq()**。
 *    因为 SetFreq(0) 会 HAL_TIM_Base_Stop() 掉整路时基；如果先 SetFreq 再
 *    StartAll，会把本该停转的那一侧用旧分频值重新启动。
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_SetFreq(MotorChannel ch, u32 hz);

/* 启动/停止两路时基（所有通道一起） */
void Bsp_MotorPulse_StartAll(void);
void Bsp_MotorPulse_StopAll(void);

#endif /* BSP_MOTOR_PULSE_H */
