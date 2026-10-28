#include "app.h"
#include "motion.h"
#include "protocol.h"
#include "log.h"
#include "bsp_uart.h"
#include "ibus.h"
#include "rc_control.h"
#include "cfg_rc.h"
#include "osal.h"

/* ============================================================================
 * App 层 — 集成点（原 Core/Src/main.c 的 USER CODE 段）
 *
 * ⚠️ 这是全工程的**唯一集成点**：Bsp 的字节回调在这里接到 Service 的协议
 *    解析器上；上电保持停车，主循环推进遥控状态机。
 *    多人协作时，这个文件只能一个人拥有。
 *
 * 本文件不允许 include 任何 MX 外设头文件（tim / usart / gpio 那三个），
 * 也不允许直接调 HAL 的延时函数 —— 一律走 Bsp / Service / Osal 的接口。
 * ============================================================================ */

static u8 s_loop_started = 0;
static u32 s_last_rc_log_ms;
static RcState s_last_rc_state;

void App_Init(void)
{
    static const u8 boot_message[] =
        "\r\nBOOT: USART1 115200 8N1, TX=PB6 RX=PB7\r\n"
        "RC: USART3 115200 8N1, RX=PB11 (i-BUS). Motors stopped.\r\n";

    s_loop_started = 0;
    Protocol_Reset();
    Ibus_Init();
    setvbuf(stdout, NULL, _IONBF, 0);  /* 调试输出立即发送，不等 stdio 缓冲。 */

    Motion_Init();
    RcControl_Init(Osal_GetTickMs());
    s_last_rc_log_ms = Osal_GetTickMs();
    s_last_rc_state = RC_WAIT_SIGNAL;

    /* 1) 先注册"收到字节交给谁"，再开中断接收（顺序不能反） */
    Bsp_Uart_SetRxByteHandler(Protocol_FeedByte);
    Bsp_Uart_RxStart();
    Bsp_Uart_RcSetRxByteHandler(Ibus_FeedByte);
    Bsp_Uart_RcRxStart();

    /* 直接发送启动提示，独立验证串口 TX，不依赖 printf 重定向。 */
    Bsp_Uart_SendBuf(boot_message, (u16)(sizeof(boot_message) - 1));
    LOG("RC map: throttle=CH%u steering=CH%u enable=CH%u. SWA LOW then HIGH; center sticks 500ms.\r\n",
        (unsigned)CFG_RC_THROTTLE_CH, (unsigned)CFG_RC_STEERING_CH, (unsigned)CFG_RC_ENABLE_CH);
}

void App_Loop(void)
{
    char frame[PROTOCOL_FRAME_MAX];
    u16  len = 0;
    u32 now;
    RcControlStatus rc;
    IbusFrame rx;
    IbusStats stats;
    s32 left;
    s32 right;

    /* Motor authority belongs to this nonblocking state machine only. */
    RcControl_Tick(Osal_GetTickMs());

    if (s_loop_started == 0)
    {
        s_loop_started = 1;
        LOG("UART ready: send text ending with CRLF (0D 0A).\r\n");
    }

    /* 串口收帧：Service 组好一帧，这里取走并回显 */
    if (Protocol_TakeFrame(frame, (u16)sizeof(frame), &len) != 0)
    {
        LOG("RX: %s\r\n", frame);

        /* Text is echoed only. It cannot bypass the RC enable/failsafe. */
    }

    RcControl_Tick(Osal_GetTickMs());
    now = Osal_GetTickMs();
    RcControl_GetStatus(&rc);
    if (rc.state != s_last_rc_state || (u32)(now - s_last_rc_log_ms) >= CFG_RC_LOG_PERIOD_MS)
    {
        s_last_rc_log_ms = now;
        s_last_rc_state = rc.state;
        (void)Ibus_GetLatest(&rx);
        Ibus_GetStats(&stats);
        Motion_GetSpeed(&left, &right);
        LOG("RC %s ch1-6=%u,%u,%u,%u,%u,%u T=%ld S=%ld L=%ld R=%ld age=%lu frames=%lu bad=%lu uart_err=%lu need_low=%u\r\n",
            RcControl_StateName(rc.state),
            (unsigned)rx.channel[0], (unsigned)rx.channel[1], (unsigned)rx.channel[2],
            (unsigned)rx.channel[3], (unsigned)rx.channel[4], (unsigned)rx.channel[5],
            (long)rc.throttle, (long)rc.steering, (long)left, (long)right,
            (unsigned long)rc.frame_age_ms, (unsigned long)stats.valid_frames,
            (unsigned long)stats.bad_checksums, (unsigned long)Bsp_Uart_RcGetErrorCount(),
            (unsigned)rc.need_enable_low);
    }
}
