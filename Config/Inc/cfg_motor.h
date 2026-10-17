#ifndef CFG_MOTOR_H
#define CFG_MOTOR_H

/* ============================================================
 * 电机与机械参数
 *
 *    定时器时钟必须与 Core/Src/main.c 的 SystemClock_Config() 一致！
 *    当前：HSE 8MHz, PLLM=8, PLLN=336, PLLP=2 → SYSCLK 168MHz
 *          APB1 = HCLK/4 = 42MHz → TIM2 时钟 = 42x2 = 84MHz
 *          APB2 = HCLK/2 = 84MHz → TIM1 时钟 = 84x2 = 168MHz
 *    改 .ioc 的时钟树后，这里必须同步，否则转速静默出错（不报警）。
 * ============================================================ */
#define CFG_TIM1_CLK_HZ     168000000UL     /* APB2 定时器 */
#define CFG_TIM2_CLK_HZ      84000000UL     /* APB1 定时器 */

/* PWM 载波：固定 ARR，靠改 PSC 调频 */
#define CFG_PUL_ARR         100

/* 脉冲频率限幅 */
#define CFG_PUL_MIN_HZ      30
#define CFG_PUL_MAX_HZ      100000

/* 电机编号（车体语义，不是通道号） */
#define CFG_MOTOR_FL        0
#define CFG_MOTOR_RL        1
#define CFG_MOTOR_FR        2
#define CFG_MOTOR_RR        3

#endif /* CFG_MOTOR_H */