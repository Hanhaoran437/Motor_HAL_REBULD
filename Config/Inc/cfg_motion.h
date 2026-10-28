#ifndef CFG_MOTION_H
#define CFG_MOTION_H

/* S 曲线变速参数 */

/* 加速度（Hz/s）。变速时长 = |目标速度 - 当前速度| / 该值。
 * 2000 == "1000Hz 用 500ms"，也就是每毫秒加 2Hz。
 * 例：0 → 2500Hz 需 1250ms；2500 → -2500Hz（跨 5000Hz）需 2500ms。
 * 运行时可改，不用重编译：Motion_SetAccel(hzPerSec)。 */
#define CFG_SCURVE_ACCEL_HZ_PER_S  2000

#define CFG_SCURVE_POINTS   101     /* 表长固定 101，改了两边都要改 */
#define CFG_SCURVE_SAMPLE_MS  8     /* 采样间隔：每 8ms 更新一次输出速度 */
#define CFG_SCURVE_MIN_MS    20     /* 变速时长下限，防止微小速度差也走长流程 */

/* Nonblocking remote path: bound a late tick to prevent a large speed jump. */
#define CFG_MOTION_MAX_TICK_MS 20U

#endif /* CFG_MOTION_H */
