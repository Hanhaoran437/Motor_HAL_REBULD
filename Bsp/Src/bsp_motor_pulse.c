#include "bsp_motor_pulse.h"
#include "mx_board.h"

/* ============================================================================
 * 四路步进脉冲输出（PUL）＋ 四路方向（DIR）
 *
 * 频率公式：  f = tim_clk / ((PSC + 1) * ARR)
 *   固定 ARR（CFG_PUL_ARR），靠改 PSC 调频。
 * ============================================================================ */

/* 分清"哪一路通道属于哪一侧定时器" */
static TIM_HandleTypeDef *Bsp_ChTimer(MotorChannel ch)
{
    return (ch == MOTOR_CH_L1 || ch == MOTOR_CH_L2) ? BOARD_HTIM_LEFT
                                                    : BOARD_HTIM_RIGHT;
}

static u32 Bsp_ChTimerClk(MotorChannel ch)
{
    return (ch == MOTOR_CH_L1 || ch == MOTOR_CH_L2) ? CFG_TIM1_CLK_HZ
                                                    : CFG_TIM2_CLK_HZ;
}

static GPIO_TypeDef *Bsp_ChDirPort(MotorChannel ch)
{
    switch (ch)
    {
        case MOTOR_CH_L1: return DIR_LF_PORT;
        case MOTOR_CH_L2: return DIR_LR_PORT;
        case MOTOR_CH_R1: return DIR_RF_PORT;
        default:          return DIR_RR_PORT;
    }
}

static uint16_t Bsp_ChDirPin(MotorChannel ch)
{
    switch (ch)
    {
        case MOTOR_CH_L1: return DIR_LF_PIN;
        case MOTOR_CH_L2: return DIR_LR_PIN;
        case MOTOR_CH_R1: return DIR_RF_PIN;
        default:          return DIR_RR_PIN;
    }
}

/* ---------------------------------------------------------------------------
 * 改一路定时器的分频系数，使其输出 freq(Hz) 的脉冲
 * ------------------------------------------------------------------------ */
static void Bsp_SetTimerFreq(TIM_HandleTypeDef *htim, u32 tim_clk, u32 freq)
{
    u32 psc;

    if (freq == 0)                          /* 频率为 0 即为停止 */
    {
        HAL_TIM_Base_Stop(htim);
        return;
    }

    if (freq < CFG_PUL_MIN_HZ) freq = CFG_PUL_MIN_HZ;   /* 频率下限保护 */
    if (freq > CFG_PUL_MAX_HZ) freq = CFG_PUL_MAX_HZ;   /* 频率上限保护 */

    psc = (tim_clk / (freq * CFG_PUL_ARR)) - 1;
    __HAL_TIM_SET_PRESCALER(htim, psc);
    htim->Instance->EGR = TIM_EGR_UG;       /* 更新事件，让 PSC 立即生效 */
}

void Bsp_MotorPulse_Init(void)
{
    HAL_TIM_PWM_Start(BOARD_HTIM_LEFT,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(BOARD_HTIM_LEFT,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(BOARD_HTIM_RIGHT, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(BOARD_HTIM_RIGHT, TIM_CHANNEL_2);

    HAL_TIM_Base_Stop(BOARD_HTIM_LEFT);
    HAL_TIM_Base_Stop(BOARD_HTIM_RIGHT);
}

void Bsp_MotorPulse_SetDir(MotorChannel ch, MotorDir dir)
{
    HAL_GPIO_WritePin(Bsp_ChDirPort(ch), Bsp_ChDirPin(ch),
                      (dir == MOTOR_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_MotorPulse_SetFreq(MotorChannel ch, u32 hz)
{
    Bsp_SetTimerFreq(Bsp_ChTimer(ch), Bsp_ChTimerClk(ch), hz);
}

void Bsp_MotorPulse_StartAll(void)
{
    (void)HAL_TIM_Base_Start(BOARD_HTIM_LEFT);
    (void)HAL_TIM_Base_Start(BOARD_HTIM_RIGHT);
}

void Bsp_MotorPulse_StopAll(void)
{
    (void)HAL_TIM_Base_Stop(BOARD_HTIM_LEFT);
    (void)HAL_TIM_Base_Stop(BOARD_HTIM_RIGHT);
}
