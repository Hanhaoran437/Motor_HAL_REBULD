#include "motion.h"
#include "bsp_motor_pulse.h"
#include "osal.h"
#include "cfg_motion.h"

/* ============================================================================
 * App 层 — 运动控制
 *
 * 分家规则（这是整个重构最核心的一刀）：
 *   - 这一层知道"左前轮/右后轮/左转/右转"   → 留在本文件（App）
 *   - 下层只知道"某一路通道 / PSC 分频 / EGR" → 在 Bsp/bsp_motor_pulse.c
 *
 * 本文件只允许通过 Bsp 的接口碰硬件：不许 include "tim.h"，不许 HAL_GPIO_WritePin。
 *
 * 速度模型：
 *   s_speed_left / s_speed_right 记录"当前已下达"的左右速度（Hz，有符号）。
 *   所有平滑动作都从这里当起点，所以调用者不需要传"从多少开始"。
 * ============================================================================ */

/* 车体语义 → Bsp 通道（这层映射本身就是 App 层的职责） */
#define MOTION_CH_FL   MOTOR_CH_L1
#define MOTION_CH_RL   MOTOR_CH_L2
#define MOTION_CH_FR   MOTOR_CH_R1
#define MOTION_CH_RR   MOTOR_CH_R2

/* 左/右侧轮在车体上镜像安装，同一 DIR 电平下两侧转向相反 */
#define MOTION_DIR_LEFT_FWD    MOTOR_DIR_CCW
#define MOTION_DIR_LEFT_BWD    MOTOR_DIR_CW
#define MOTION_DIR_RIGHT_FWD   MOTOR_DIR_CW
#define MOTION_DIR_RIGHT_BWD   MOTOR_DIR_CCW

/* ---------------------------------------------------------------------------
 * S 曲线过渡表（101 点，0 ~ 1000 表示 0.0% ~ 100.0%）
 *   本质为 cosine S 曲线: y(t) = (1-cos(pi*t))/2
 *   加速度在两端为 0、中间最大，起点/终点速度无阶跃跳变，
 *   避免 37cm 小车斜坡加速时更容易失步的问题。
 * ------------------------------------------------------------------------ */
static const u16 s_scurve_table[CFG_SCURVE_POINTS] = {
      0,   0,   1,   2,   4,   6,   9,  12,  16,  20,
     24,  30,  35,  41,  48,  54,  62,  70,  78,  86,
     95, 105, 115, 125, 136, 146, 158, 169, 181, 194,
    206, 219, 232, 245, 259, 273, 287, 301, 316, 331,
    345, 361, 376, 391, 406, 422, 437, 453, 469, 484,
    500, 516, 531, 547, 563, 578, 594, 609, 624, 639,
    655, 669, 684, 699, 713, 727, 741, 755, 768, 781,
    794, 806, 819, 831, 842, 854, 864, 875, 885, 895,
    905, 914, 922, 930, 938, 946, 952, 959, 965, 970,
    976, 980, 984, 988, 991, 994, 996, 998, 999, 1000,
    1000
};

/* 当前已下达的左右速度（Hz，有符号）。Move 正常更新，StopImmediate 清零。 */
static s32 s_speed_left  = 0;
static s32 s_speed_right = 0;
static s32 s_target_left = 0;
static s32 s_target_right = 0;
static u32 s_tick_ms;
static u32 s_step_fraction;

/* 当前加速度（Hz/s）。S 曲线变速时长 = |速度差| / s_accel_hz_per_s。
 * 默认 2000 == "1000Hz 用 500ms"；运行时可调，见 Motion_SetAccel()。 */
static u32 s_accel_hz_per_s = CFG_SCURVE_ACCEL_HZ_PER_S;

void Motion_Init(void)
{
    /* 时钟、GPIO、AF 复用、时基、PWM 通道全部由 CubeMX 生成代码完成，
     * Bsp 保持 4 个通道及两路计数器停止，首次非零目标才启动。 */
    Bsp_MotorPulse_Init();

    s_speed_left  = 0;
    s_speed_right = 0;
    s_target_left = 0;
    s_target_right = 0;
    s_tick_ms = Osal_GetTickMs();
    s_step_fraction = 0;
}

/* ---------------------------------------------------------------------------
 * 底层：立即把左右速度写进硬件（不平滑）
 *   v > 0 前进，v < 0 后退，v == 0 该侧停止
 * ------------------------------------------------------------------------ */
void Motion_Move(s32 vLeft, s32 vRight)
{
    u32 fl = (vLeft  < 0) ? (u32)(-vLeft)  : (u32)vLeft;
    u32 fr = (vRight < 0) ? (u32)(-vRight) : (u32)vRight;

    /* Stop pulses before changing direction or stopping a side. */
    if (s_speed_left != 0 && (vLeft == 0 || ((s_speed_left < 0) != (vLeft < 0))))
    {
        Bsp_MotorPulse_SetFreq(MOTION_CH_FL, 0);
    }
    if (s_speed_right != 0 && (vRight == 0 || ((s_speed_right < 0) != (vRight < 0))))
    {
        Bsp_MotorPulse_SetFreq(MOTION_CH_FR, 0);
    }

    /* 先定方向，再加载频率并启动对应一侧。 */
    Bsp_MotorPulse_SetDir(MOTION_CH_FL, (vLeft  < 0) ? MOTION_DIR_LEFT_BWD  : MOTION_DIR_LEFT_FWD);
    Bsp_MotorPulse_SetDir(MOTION_CH_RL, (vLeft  < 0) ? MOTION_DIR_LEFT_BWD  : MOTION_DIR_LEFT_FWD);
    Bsp_MotorPulse_SetDir(MOTION_CH_FR, (vRight < 0) ? MOTION_DIR_RIGHT_BWD : MOTION_DIR_RIGHT_FWD);
    Bsp_MotorPulse_SetDir(MOTION_CH_RR, (vRight < 0) ? MOTION_DIR_RIGHT_BWD : MOTION_DIR_RIGHT_FWD);

    /* SetFreq starts only its own nonzero side; a zero side remains stopped. */
    Bsp_MotorPulse_SetFreq(MOTION_CH_FL, fl);          /* 左侧（FL/RL 共用 TIM1） */
    Bsp_MotorPulse_SetFreq(MOTION_CH_FR, fr);          /* 右侧（FR/RR 共用 TIM2） */

    s_speed_left  = vLeft;
    s_speed_right = vRight;
}

/* ---------------------------------------------------------------------------
 * 内部：按 S 曲线从"当前速度"过渡到目标速度（阻塞）
 *
 *   变速时长由加速度推出：durationMs = max(|Δ左|, |Δ右|) × 1000 / 加速度
 *   取两侧中变化更大的一侧，保证哪一轮都不超过设定的加速度上限；
 *   两侧共用同一条进度曲线，所以差速转向时左右轮同时到位、轨迹不会歪。
 *
 *   进度按"真实经过时间"算（Osal_GetTickMs），不是数循环次数，
 *   所以 DelayMs 的抖动不会累积成时长误差。
 * ------------------------------------------------------------------------ */
static void Motion_RampTo(s32 vLeftTo, s32 vRightTo)
{
    /* 必须先快照：循环里 Motion_Move() 会改写 s_speed_*，不能拿它当起点 */
    const s32 leftFrom  = s_speed_left;
    const s32 rightFrom = s_speed_right;
    const s32 dLeft     = vLeftTo  - leftFrom;
    const s32 dRight    = vRightTo - rightFrom;
    u32 span, durationMs, t0, elapsed, idx;

    if ((dLeft == 0) && (dRight == 0))
    {
        return;                                        /* 已在目标速度，不空转 */
    }

    span = (u32)((dLeft  < 0) ? -dLeft  : dLeft);
    if ((u32)((dRight < 0) ? -dRight : dRight) > span)
    {
        span = (u32)((dRight < 0) ? -dRight : dRight);
    }

    durationMs = (span * 1000UL) / s_accel_hz_per_s;
    if (durationMs < CFG_SCURVE_MIN_MS)
    {
        durationMs = CFG_SCURVE_MIN_MS;
    }

    t0 = Osal_GetTickMs();
    for (;;)
    {
        elapsed = Osal_GetTickMs() - t0;
        if (elapsed >= durationMs)
        {
            break;
        }

        idx = (elapsed * (CFG_SCURVE_POINTS - 1)) / durationMs;
        Motion_Move(leftFrom  + ((vLeftTo  - leftFrom)  * (s32)s_scurve_table[idx]) / 1000,
                    rightFrom + ((vRightTo - rightFrom) * (s32)s_scurve_table[idx]) / 1000);

        Osal_DelayMs(CFG_SCURVE_SAMPLE_MS);
    }

    Motion_Move(vLeftTo, vRightTo);                    /* 精确落到目标值 */
}

/* ---------------------------------------------------------------------------
 * ① 转弯 / 差速：左轮 vLeft、右轮 vRight（Hz，有符号）
 *   vLeft > vRight → 右转     vRight > vLeft → 左转     相等 → 直行
 *   负数表示该轮反转，例如 Motion_Steer(-2500, 2500) 就是原地左转。
 * ------------------------------------------------------------------------ */
void Motion_Steer(s32 vLeft, s32 vRight, u32 holdMs)
{
    Motion_RampTo(vLeft, vRight);

    if (holdMs > 0)
    {
        Osal_DelayMs(holdMs);                          /* 到达目标后保持 */
    }
}

/* ---------------------------------------------------------------------------
 * ① 进退：speed > 0 前进、< 0 后退、== 0 停止，左右同速（直线）
 * ------------------------------------------------------------------------ */
void Motion_Drive(s32 speed, u32 holdMs)
{
    Motion_Steer(speed, speed, holdMs);
}

/* ---------------------------------------------------------------------------
 * ① 停车：两侧 S 曲线降到 0
 * ------------------------------------------------------------------------ */
void Motion_Brake(void)
{
    Motion_Steer(0, 0, 0);
}

/* ---------------------------------------------------------------------------
 * ② 底层：读回当前已下达的速度（Hz，有符号），便于调试时观察
 * ------------------------------------------------------------------------ */
void Motion_GetSpeed(s32 *vLeft, s32 *vRight)
{
    if (vLeft  != 0) { *vLeft  = s_speed_left;  }
    if (vRight != 0) { *vRight = s_speed_right; }
}

/* ---------------------------------------------------------------------------
 * ③ 加速度：修改"多快达到目标速度"，对所有后续 S 曲线变速立即生效
 *   hzPerSec = 2000  → 1000Hz 用 500ms（默认值）
 *   hzPerSec = 4000  → 变速快一倍（0 → 2500Hz 用 625ms）
 *   hzPerSec 传 0 是非法值，直接忽略，保持原加速度不变。
 *   只改运行期状态，断电复位后回到 CFG_SCURVE_ACCEL_HZ_PER_S。
 * ------------------------------------------------------------------------ */
void Motion_SetAccel(u32 hzPerSec)
{
    if (hzPerSec == 0)
    {
        return;
    }

    s_accel_hz_per_s = hzPerSec;
}

u32 Motion_GetAccel(void)
{
    return s_accel_hz_per_s;
}

static s32 Motion_ClampTarget(s32 speed)
{
    if (speed > (s32)CFG_PUL_MAX_HZ) { return (s32)CFG_PUL_MAX_HZ; }
    if (speed < -(s32)CFG_PUL_MAX_HZ) { return -(s32)CFG_PUL_MAX_HZ; }
    return speed;
}

void Motion_SetTarget(s32 vLeft, s32 vRight)
{
    s_target_left = Motion_ClampTarget(vLeft);
    s_target_right = Motion_ClampTarget(vRight);
}

static s32 Motion_Step(s32 current, s32 target, s32 step)
{
    /* Never jump from positive to negative pulses in one update. */
    if ((current > 0 && target < 0) || (current < 0 && target > 0)) { target = 0; }
    if (current < target) { return (target - current <= step) ? target : current + step; }
    if (current > target) { return (current - target <= step) ? target : current - step; }
    return current;
}

void Motion_Tick(u32 nowMs)
{
    u32 elapsed = nowMs - s_tick_ms;
    uint64_t credit;
    u32 step;
    s32 left;
    s32 right;
    if (elapsed < CFG_SCURVE_SAMPLE_MS) { return; }
    s_tick_ms = nowMs;
    if (elapsed > CFG_MOTION_MAX_TICK_MS) { elapsed = CFG_MOTION_MAX_TICK_MS; }
    credit = (uint64_t)s_accel_hz_per_s * elapsed + s_step_fraction;
    step = (u32)(credit / 1000U);
    s_step_fraction = (u32)(credit % 1000U);
    if (step > 2U * CFG_PUL_MAX_HZ) { step = 2U * CFG_PUL_MAX_HZ; }
    left = Motion_Step(s_speed_left, s_target_left, (s32)step);
    right = Motion_Step(s_speed_right, s_target_right, (s32)step);
    if (left != s_speed_left || right != s_speed_right) { Motion_Move(left, right); }
}

void Motion_StopImmediate(void)
{
    Bsp_MotorPulse_SetFreq(MOTION_CH_FL, 0);
    Bsp_MotorPulse_SetFreq(MOTION_CH_FR, 0);
    s_speed_left = 0;
    s_speed_right = 0;
    s_target_left = 0;
    s_target_right = 0;
    s_step_fraction = 0;
    s_tick_ms = Osal_GetTickMs();
}
