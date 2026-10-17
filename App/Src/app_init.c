#include "app.h"
#include "motion.h"
#include "mission.h"
#include "protocol.h"
#include "log.h"
#include "bsp_uart.h"

/* ============================================================================
 * App 层 — 集成点（原 Core/Src/main.c 的 USER CODE 段）
 *
 * ⚠️ 这是全工程的**唯一集成点**：Bsp 的字节回调在这里接到 Service 的协议
 *    解析器上；上电后跑什么流程也在这里决定。
 *    多人协作时，这个文件只能一个人拥有。
 *
 * 本文件不允许 include 任何 MX 外设头文件（tim / usart / gpio 那三个），
 * 也不允许直接调 HAL 的延时函数 —— 一律走 Bsp / Service / Osal 的接口。
 * ============================================================================ */

void App_Init(void)
{
    /* 1) 先注册"收到字节交给谁"，再开中断接收（顺序不能反） */
    Bsp_Uart_SetRxByteHandler(Protocol_FeedByte);
    Bsp_Uart_RxStart();

    /* 2) 四路电机初始化（打开 PWM 通道，保持时基停止） */
    Motion_Init();

    /* 3) 启动上电流程 */
    Mission_Init();
    Mission_Start(0);
}

void App_Loop(void)
{
    char frame[PROTOCOL_FRAME_MAX];
    u16  len = 0;

    /* 流程推进（非阻塞：演示动作之间的等待在这里消耗） */
    Mission_Tick();

    /* 串口收帧：Service 组好一帧，这里取走并回显 */
    if (Protocol_TakeFrame(frame, (u16)sizeof(frame), &len) != 0)
    {
        LOG("RX: %s\r\n", frame);

        /* TODO: 解析上位机命令 → 调 Motion_* / Mission_*
         *       例如 "F2500" 前进、 "L1000" 左转、 "S" 停车、
         *            "M3" 开始走 3 株（表型检测流程）。 */
    }
}
