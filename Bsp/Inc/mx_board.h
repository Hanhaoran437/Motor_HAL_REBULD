#ifndef BSP_MX_BOARD_H
#define BSP_MX_BOARD_H

/* ============================================================================
 * 防腐层（Anti-Corruption Layer）
 *
 * 全工程**唯一**允许 #include CubeMX 生成头文件的地方
 * （main.h / tim.h / usart.h / gpio.h）。
 *
 * 其它任何一层想用 htim1、huart1、GPIO_PIN_x、DIR_xx_Pin，
 * 都必须经过这里翻译成 BOARD_xxx 名字。
 * 这样将来在 .ioc 里改外设名/换引脚/换串口，只改这一个文件。
 *
 * ⚠️ 本文件属于 Bsp 层：不允许出现任何上层（Service/App）概念。
 * ============================================================================ */

#include "main.h"
#include "tim.h"
#include "usart.h"

/* ---------------------------------------------------------------------------
 * 定时器：左右两侧各一路独立时基（差速转向需要左右频率不同）
 *   左侧 FL/RL -> TIM1
 *   右侧 FR/RR -> TIM2
 * ------------------------------------------------------------------------ */
#define BOARD_HTIM_LEFT           (&htim1)
#define BOARD_HTIM_RIGHT          (&htim2)

/* ---------------------------------------------------------------------------
 * 串口：与上位机通信的那一路，USART1，PB6=TX / PB7=RX。
 * GPIO 复用由 .ioc / Core/Src/usart.c 配置。
 * ------------------------------------------------------------------------ */
#define BOARD_HUART_HOST          (&huart1)
#define BOARD_UART_HOST_INSTANCE  USART1

/* ---------------------------------------------------------------------------
 * PUL 脉冲输出引脚（AF 复用）
 *
 * ⚠️ CubeMX 不为 AF 复用引脚生成 User Label 宏，下面 8 行是手工补的引脚表。
 *    来源：Core/Src/tim.c 的 HAL_TIM_MspPostInit()
 *      tim.c:203  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;  (GPIOE, AF1_TIM1)
 *      tim.c:225  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;   (GPIOA, AF1_TIM2)
 *
 * ⚠️ 在 CubeMX 里改过引脚分配后，这里必须同步修改；CubeMX 不会提醒你，
 *    编译器也不会报错（PUL 是硬件定时器输出，写错了只是"某个轮子不转"）。
 *
 * 更彻底的做法：以后在 CubeMX 里给 PE9/PE11/PA0/PA1 设 User Label
 * (PUL_LF / PUL_LR / PUL_RF / PUL_RR)，生成的宏就可以直接替换下面 8 行。
 * ------------------------------------------------------------------------ */
#define PUL_LF_PIN   GPIO_PIN_9        /* PE9   TIM1_CH1  左前 */
#define PUL_LF_PORT  GPIOE
#define PUL_LR_PIN   GPIO_PIN_11       /* PE11  TIM1_CH2  左后 */
#define PUL_LR_PORT  GPIOE
#define PUL_RF_PIN   GPIO_PIN_0        /* PA0   TIM2_CH1  右前 */
#define PUL_RF_PORT  GPIOA
#define PUL_RR_PIN   GPIO_PIN_1        /* PA1   TIM2_CH2  右后 */
#define PUL_RR_PORT  GPIOA

/* ---------------------------------------------------------------------------
 * DIR 方向引脚：CubeMX 已生成（Core/Inc/main.h:60-67），这里只做改名
 * ------------------------------------------------------------------------ */
#define DIR_LF_PIN   DIR_LF_Pin
#define DIR_LF_PORT  DIR_LF_GPIO_Port
#define DIR_LR_PIN   DIR_LR_Pin
#define DIR_LR_PORT  DIR_LR_GPIO_Port
#define DIR_RF_PIN   DIR_RF_Pin
#define DIR_RF_PORT  DIR_RF_GPIO_Port
#define DIR_RR_PIN   DIR_RR_Pin
#define DIR_RR_PORT  DIR_RR_GPIO_Port

#endif /* BSP_MX_BOARD_H */
