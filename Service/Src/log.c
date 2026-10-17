#include "log.h"
#include "bsp_uart.h"

/* ============================================================================
 * printf 重定向（替代旧工程 usart.c 里的 ARMCC fputc 方案）
 *
 *    Core/Src/syscalls.c:35 里只有一句弱声明：
 *        extern int __io_putchar(int ch) __attribute__((weak));
 *    全工程没有任何地方定义它（只有 syscalls.c:87 的 _write() 里调用）。
 *    弱符号找不到强定义会解析成地址 0 —— printf 能链接通过，
 *    但一调用就跳 0 → HardFault。
 *
 *    ⚠️ 因此本函数必须是**强定义且全局符号**（不能加 static），
 *       否则 newlib-nano 的 printf 又会走回空指针。
 *
 *    发送走 Bsp 的 Bsp_Uart_SendByte()：
 *      Service 层不允许 include usart.h / 不许直接碰 HAL 句柄，
 *      只能依赖 Bsp 暴露的接口（这正是 Service → Bsp 的合法方向）。
 * ============================================================================ */

int __io_putchar(int ch)
{
    Bsp_Uart_SendByte((u8)ch);
    return ch;
}
