#include "mission.h"
#include "motion.h"
#include "log.h"

/* ============================================================================
 * App 层 — 作业流程（上电演示）
 *
 * 五段演示：起步前进 → 左转 → 回正 → 减速后退 → 停车
 * 每一段就是一次 Motion_Drive / Motion_Steer / Motion_Brake 调用；
 * "加速多久、保持多久"全部由 Motion 那三个函数的 holdMs 参数决定，
 * 所以本文件不再需要自己算停留时间。
 *
 * ⚠️ 这三个动作函数是阻塞的：Mission_Tick() 一次调用会阻塞整段动作时间，
 *    跑完一段才返回主循环一次（串口回显因此会延后，但接收由中断完成不会丢）。
 * ============================================================================ */

/* 演示参数（改流程只动本文件） */
#define MISSION_DEMO_FREQ   2500                 /* 直线速度，Hz */
#define MISSION_DEMO_TURN   1000                 /* 左转时内侧轮速度，Hz */

/* 各段到达目标速度后"保持"多久 */
#define MISSION_STEP1_HOLD_MS   2000
#define MISSION_STEP2_HOLD_MS   1500
#define MISSION_STEP3_HOLD_MS   1000
#define MISSION_STEP4_HOLD_MS   2000

static MissionState s_state = MISSION_IDLE;
static u16          s_plant_count = 0;
static u8           s_step = 0;

static void Mission_DoStep(u8 step)
{
    switch (step)
    {
        case 0:  /* 1. 从零起步前进：S 曲线 0 → +2500Hz，然后保持 2s */
            Motion_Drive(MISSION_DEMO_FREQ, MISSION_STEP1_HOLD_MS);
            break;

        case 1:  /* 2. 左转：右轮快(2500)、左轮慢(1000) → 车头左偏 */
            Motion_Steer(MISSION_DEMO_TURN, MISSION_DEMO_FREQ, MISSION_STEP2_HOLD_MS);
            break;

        case 2:  /* 3. 回正：左轮提速回 2500，与右轮同速 */
            Motion_Drive(MISSION_DEMO_FREQ, MISSION_STEP3_HOLD_MS);
            break;

        case 3:  /* 4. 减速后退：+2500 → 0 → -2500，中途自动换向 */
            Motion_Drive(-MISSION_DEMO_FREQ, MISSION_STEP4_HOLD_MS);
            break;

        default: /* 5. 停车：S 曲线降到 0 */
            Motion_Brake();
            break;
    }
}

void Mission_Init(void)
{
    s_state = MISSION_IDLE;
    s_step = 0;
    s_plant_count = 0;
}

void Mission_Start(u16 plantCount)
{
    s_plant_count = plantCount;

    LOG("Move demo: drive -> turn left -> drive -> back -> brake\r\n");

    s_step = 0;
    s_state = MISSION_RUNNING;
    Mission_DoStep(0);                          /* 第 1 段在 Start 里就跑掉 */
}

void Mission_Tick(void)
{
    if (s_state != MISSION_RUNNING)
    {
        return;
    }

    s_step++;
    if (s_step > 4)                             /* 五段跑完 */
    {
        s_state = MISSION_DONE;
        return;
    }

    Mission_DoStep(s_step);
}

MissionState Mission_GetState(void)
{
    return s_state;
}

u16 Mission_GetPlantCount(void)
{
    return s_plant_count;
}
