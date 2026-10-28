#ifndef BSP_UART_H
#define BSP_UART_H

#include "types.h"

/* ============================================================================
 * 串口（USART1 调试 + USART3 遥控）— 只负责"把收到的字节交给上层"，
 * 不解析协议、不组帧（那是 Service 层 protocol.c 的事）。
 *
 * ⚠️ 绝不能再自己定义 USART1_IRQHandler！
 *    Core/Src/stm32f4xx_it.c 里已经有一个（它调用 HAL_UART_IRQHandler(&huart1)），
 *    重复定义会导致串口整个收不到数据（链接不报错，行为直接错）。
 * ============================================================================ */

/* 收到 1 字节时回调；由上层注册（App/Src/app_init.c 注册 Protocol_FeedByte） */
typedef void (*Bsp_Uart_RxByteHandler)(u8 byte);

void Bsp_Uart_RxStart(void);                                   /* 启动中断接收 */
void Bsp_Uart_SetRxByteHandler(Bsp_Uart_RxByteHandler handler); /* 注册回调 */

/* Independent USART3 receiver. Same byte-only callback, no protocol coupling. */
void Bsp_Uart_RcRxStart(void);
void Bsp_Uart_RcSetRxByteHandler(Bsp_Uart_RxByteHandler handler);
u32  Bsp_Uart_RcGetErrorCount(void);

/* --- 发送 ---------------------------------------------------------------
 * 上层（Service 的 log.c 里 __io_putchar / 以后协议回包）只调这两个函数，
 * 不要去直接抓 HAL 句柄 —— Service 层不允许 include usart.h。
 * 都是阻塞发送（和旧 uart_port.c 的 __io_putchar 行为一致）。
 * ---------------------------------------------------------------------- */
void Bsp_Uart_SendByte(u8 byte);
void Bsp_Uart_SendBuf(const u8 *buf, u16 len);

#endif /* BSP_UART_H */
