#ifndef CFG_RC_H
#define CFG_RC_H

/* This transmitter measured on 2026-10-27: right vertical = CH3;
 * left horizontal = CH4. Channel numbers here are ONE based. Standard Mode 2
 * often uses CH2 for right vertical; verify again after a model/mode change. */
#define CFG_RC_THROTTLE_CH          3U
#define CFG_RC_STEERING_CH          4U
#define CFG_RC_ENABLE_CH            5U   /* Assign SWA; failsafe MUST be 1000. */
#define CFG_RC_THROTTLE_REVERSE     0U
#define CFG_RC_STEERING_REVERSE     0U

#define CFG_RC_INPUT_MIN_US         1000U
#define CFG_RC_INPUT_CENTER_US      1500U
#define CFG_RC_INPUT_MAX_US         2000U
#define CFG_RC_INPUT_VALID_MIN_US   900U
#define CFG_RC_INPUT_VALID_MAX_US   2100U
#define CFG_RC_DEADBAND_US          50U
#define CFG_RC_ENABLE_LOW_US        1200U
#define CFG_RC_ENABLE_HIGH_US       1800U

#define CFG_RC_FRAME_TIMEOUT_MS     200U
#define CFG_RC_NEUTRAL_HOLD_MS      500U
#define CFG_RC_NEUTRAL_MAX_GAP_MS   50U
#define CFG_RC_MAX_SPEED_HZ         2500
/* Full steering at zero throttle: opposite wheel speeds, limited to 600 Hz.
 * This also sets the steering authority floor at low throttle. 0 disables it. */
#define CFG_RC_PIVOT_MAX_HZ         600
#define CFG_RC_LOG_PERIOD_MS        500U

/* The SERVO i-BUS stream has no reliable radio-loss flag. A configured CH5
 * failsafe is essential: some receivers KEEP transmitting after RF loss. */
#endif /* CFG_RC_H */
