#include "app.h"
#include "motion.h"
#include "protocol.h"
#include "log.h"
#include "bsp_uart.h"

/* ============================================================================
 * App 层 — 集成点（原 Core/Src/main.c 的 USER CODE 段）
 *
 * ⚠️ 这是全工程的**唯一集成点**：Bsp 的字节回调在这里接到 Service 的协议
 *    解析器上；上电后要跑什么动作由 Core/Src/main.c 的 USER CODE 段决定。
 *    多人协作时，这个文件只能一个人拥有。
 *
 * 本文件不允许 include 任何 MX 外设头文件（tim / usart / gpio 那三个），
 * 也不允许直接调 HAL 的延时函数 —— 一律走 Bsp / Service / Osal 的接口。
 * ============================================================================ */

static u8 s_loop_started = 0;

void App_Init(void)
{
    static const u8 boot_message[] =
        "\r\nBOOT: USART1 115200 8N1, TX=PB6 RX=PB7\r\n"
        "Motor demo running; wait for UART ready.\r\n";

    s_loop_started = 0;
    Protocol_Reset();
    setvbuf(stdout, NULL, _IONBF, 0);  /* 调试输出立即发送，不等 stdio 缓冲。 */

    /* 1) 先注册"收到字节交给谁"，再开中断接收（顺序不能反） */
    Bsp_Uart_SetRxByteHandler(Protocol_FeedByte);
    Bsp_Uart_RxStart();

    /* 2) 四路电机初始化（打开 PWM 通道，保持时基停止） */
    Motion_Init();

    /* 直接发送启动提示，独立验证串口 TX，不依赖 printf 重定向。 */
    Bsp_Uart_SendBuf(boot_message, (u16)(sizeof(boot_message) - 1));
}

void App_Loop(void)
{
    char frame[PROTOCOL_FRAME_MAX];
    u16  len = 0;

    if (s_loop_started == 0)
    {
        s_loop_started = 1;
        LOG("UART ready: send text ending with CRLF (0D 0A).\r\n");
    }

    /* 串口收帧：Service 组好一帧，这里取走并回显 */
    if (Protocol_TakeFrame(frame, (u16)sizeof(frame), &len) != 0)
    {
        LOG("RX: %s\r\n", frame);

        /* TODO: 解析上位机命令 → 调 Motion_Drive / Motion_Steer / Motion_Brake
         *       例如 "F2500" 前进、 "L1000" 左转、 "S" 停车。 */
    }
}
