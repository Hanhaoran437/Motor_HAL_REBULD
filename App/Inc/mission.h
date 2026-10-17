#ifndef APP_MISSION_H
#define APP_MISSION_H

#include "types.h"

/* ============================================================================
 * App 层 — 作业流程（原 Core/Src/main.c:99-118 那五段演示动作的搬迁）
 *
 * 现状：上电后跑一遍"前进 → 左转 → 回正 → 后退 → 停车"的演示序列，
 *       用来验证电机驱动；仍然是非阻塞状态机，由 App_Loop() 每轮调 Mission_Tick()。
 * 将来：这里就是"走到第 N 株 → 停车 → 触发拍照 → 记录株号"的主流程。
 *
 * ⚠️ 真正的表型检测逻辑（株号、拍照、数据回传）还没写；
 *    目前 plantCount 只是记下来备用。
 * ============================================================================ */

typedef enum
{
    MISSION_IDLE = 0,     /* 还没启动 */
    MISSION_RUNNING,      /* 演示/作业进行中 */
    MISSION_DONE          /* 跑完了 */
} MissionState;

void         Mission_Init(void);
void         Mission_Start(u16 plantCount);   /* 启动作业流程 */
void         Mission_Tick(void);              /* 主循环每轮调用（非阻塞） */
MissionState Mission_GetState(void);
u16          Mission_GetPlantCount(void);

#endif /* APP_MISSION_H */
