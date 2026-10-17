#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "types.h"

/* ============================================================================
 * App 层 — 运动控制
 *
 * 接口分两层：
 *
 *  ① 顶层"动作"接口（业务代码只用这三个）
 *       Motion_Drive(speed, holdMs)            进退：正=前进 / 负=后退 / 0=停
 *       Motion_Steer(vLeft, vRight, holdMs)    转弯：左>右 = 右转，右>左 = 左转
 *       Motion_Brake()                         停车：当前速度 S 曲线降到 0
 *
 *     这三个都是**阻塞**的完整动作：
 *        先用 S 曲线把速度过渡到目标（固定 CFG_SCURVE_TIME_MS = 800ms），
 *        再保持 holdMs 毫秒，然后返回。
 *     起始速度不用传 —— 本层内部记住"当前已下达的速度"，自动从那里过渡。
 *
 *  ② 底层接口（调试或需要自管过渡时用）
 *       Motion_Move(vLeft, vRight)             立即生效，不平滑
 *       Motion_GetSpeed(&l, &r)                读回当前已下达的速度
 *
 * ⚠️ 速度单位是脉冲频率 Hz，不是 mm/s。
 *    左右各有一路定时器，所以左右可以不同（差速转向）；
 *    但**同侧两轮共用定时器，永远同频**。
 *
 * ⚠️ 这三个动作函数内部用 Osal_DelayMs 忙等，期间主循环不执行。
 *    串口接收靠中断不会丢，但回显会延后到动作结束。
 * ============================================================================ */

/* 初始化四路电机（内部调 Bsp_MotorPulse_Init），并把内部速度状态清零 */
void Motion_Init(void);

/* --- ① 进退：speed > 0 前进，< 0 后退，== 0 停止 ---------------------- */
void Motion_Drive(s32 speed, u32 holdMs);

/* --- ① 转弯：vLeft > vRight 右转，vRight > vLeft 左转，相等为直行 ---- */
void Motion_Steer(s32 vLeft, s32 vRight, u32 holdMs);

/* --- ① 停车：两侧 S 曲线降到 0 --------------------------------------- */
void Motion_Brake(void);

/* --- ② 底层：立即生效（不平滑），调用者自己负责过渡 -------------------- */
void Motion_Move(s32 vLeft, s32 vRight);

/* --- ② 底层：读回当前已下达的左右速度（Hz，有符号）；传 NULL 可忽略 --- */
void Motion_GetSpeed(s32 *vLeft, s32 *vRight);

#endif /* APP_MOTION_H */
