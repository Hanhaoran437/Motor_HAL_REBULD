#include "osal.h"
#include "stm32f4xx_hal.h"

/* ============================================================================
 * OSAL 实现 — 当前是裸机 + SysTick 版本，只有两行。
 * 换 RTOS 时只需要改这个文件。
 * ============================================================================ */

void Osal_DelayMs(u32 ms)
{
    HAL_Delay(ms);
}

u32 Osal_GetTickMs(void)
{
    return HAL_GetTick();
}
