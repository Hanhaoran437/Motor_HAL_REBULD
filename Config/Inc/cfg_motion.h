#ifndef CFG_MOTION_H
#define CFG_MOTION_H

/* 线性斜坡 */
#define CFG_RAMP_START_HZ   100
#define CFG_RAMP_STEP_HZ    20
#define CFG_RAMP_STEP_MS    15

/* S 曲线 */
#define CFG_SCURVE_START_HZ 50
#define CFG_SCURVE_TIME_MS  800
#define CFG_SCURVE_POINTS   101     /* 表长固定 101，改了两边都要改 */

#endif /* CFG_MOTION_H */