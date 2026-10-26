#include "protocol.h"

/* ============================================================================
 * 帧状态机（与旧工程 USART1_IRQHandler 里的完全一致）
 *
 *    bit15    : 一帧接收完成
 *    bit14    : 已经收到过 0x0D
 *    bit13~0  : 已收字节数
 *
 * 旧代码用的是全局变量 USART_RX_BUF / USART_RX_STA（uart_port.c:35-36），
 * 任何地方都能改；现在收进本文件 static，只能用这两个 API 访问。
 * ============================================================================ */

/* 中断写入、主循环读取：禁止优化器缓存状态或移动缓冲区访问。
 * 完整帧取走前 ISR 不再写入缓冲区，主循环复制完成后才清状态。
 */
static volatile u8  s_rx_buf[PROTOCOL_FRAME_MAX];
static volatile u16 s_rx_sta = 0;

void Protocol_FeedByte(u8 byte)
{
    if ((s_rx_sta & 0x8000) != 0)           /* 上一帧还没被取走：丢弃新字节 */
    {
        return;
    }

    if ((s_rx_sta & 0x4000) != 0)           /* 已经收到 0x0D，等 0x0A */
    {
        if (byte != 0x0A)
        {
            s_rx_sta = 0;                    /* 不是 0x0A，接收错误，重新开始 */
        }
        else
        {
            s_rx_sta |= 0x8000;              /* 0x0D 0x0A 齐了，一帧完成 */
        }
    }
    else                                     /* 还没收到 0x0D */
    {
        if (byte == 0x0D)
        {
            s_rx_sta |= 0x4000;
        }
        else
        {
            s_rx_buf[s_rx_sta & 0x3FFF] = byte;
            s_rx_sta++;
            if (s_rx_sta > (PROTOCOL_FRAME_MAX - 1))
            {
                s_rx_sta = 0;                /* 溢出保护：重新开始 */
            }
        }
    }
}

u16 Protocol_TakeFrame(char *out, u16 outSize, u16 *len)
{
    u16 n;
    u16 i;

    if ((s_rx_sta & 0x8000) == 0)            /* 没有完整帧 */
    {
        return 0;
    }

    n = (u16)(s_rx_sta & 0x3FFF);            /* 帧长（不含结束符） */

    if (out == 0 || outSize == 0)
    {
        s_rx_sta = 0;
        return 0;
    }

    if (n > (u16)(outSize - 1))              /* 缓冲区不够：截断 */
    {
        n = (u16)(outSize - 1);
    }

    for (i = 0; i < n; i++)
    {
        out[i] = (char)s_rx_buf[i];
    }
    out[n] = '\0';

    if (len != 0)
    {
        *len = n;
    }

    s_rx_sta = 0;                            /* 清标志，准备收下一帧 */
    return 1;
}

void Protocol_Reset(void)
{
    s_rx_sta = 0;
}
