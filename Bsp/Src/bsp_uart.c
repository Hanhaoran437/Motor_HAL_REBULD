#include "bsp_uart.h"
#include "mx_board.h"

/* ============================================================================
 * 串口底层：HAL 逐字节中断接收 + "把字节转交给上层注册的回调"
 *
 *    旧代码在 USART1_IRQHandler 里直接读 DR（SPL 时代）；
 *    现在用 HAL_UART_Receive_IT() 每次收 1 字节，收完调 HAL_UART_RxCpltCallback。
 *
 *    旧代码把"帧状态机"也写在这个回调里 → 会导致 Bsp 依赖 Service（方向反了）。
 *    现在回调只做一件事：把字节丢给注册进来的处理函数；没人注册就丢掉。
 * ============================================================================ */

static u8 s_rx_byte;                                       /* HAL 逐字节接收暂存 */
static Bsp_Uart_RxByteHandler s_rx_handler = 0;             /* 上层注册的字节处理器 */

void Bsp_Uart_RxStart(void)
{
    HAL_UART_Receive_IT(BOARD_HUART_HOST, &s_rx_byte, 1);
}

void Bsp_Uart_SetRxByteHandler(Bsp_Uart_RxByteHandler handler)
{
    s_rx_handler = handler;
}

void Bsp_Uart_SendByte(u8 byte)
{
    HAL_UART_Transmit(BOARD_HUART_HOST, &byte, 1, HAL_MAX_DELAY);
}

void Bsp_Uart_SendBuf(const u8 *buf, u16 len)
{
    if (buf == 0 || len == 0)
    {
        return;
    }
    HAL_UART_Transmit(BOARD_HUART_HOST, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

/* HAL 的接收完成回调（弱符号，全工程只能有这一处强定义） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BOARD_UART_HOST_INSTANCE)
    {
        if (s_rx_handler != 0)
        {
            s_rx_handler(s_rx_byte);
        }

        /* 重新武装，准备收下一个字节 */
        HAL_UART_Receive_IT(BOARD_HUART_HOST, &s_rx_byte, 1);
    }
}
