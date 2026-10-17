#ifndef SERVICE_LOG_H
#define SERVICE_LOG_H

#include <stdio.h>

/* ============================================================================
 * Service 层 — 统一日志出口
 *
 * 现在就是 printf（底层重定向见 log.c），将来要换 RTT / 落卡 / 关日志，
 * 只改这一个文件，上层不动。
 *
 * 用法：  LOG("RX: %s\r\n", frame);
 * ============================================================================ */

#define LOG(...)  printf(__VA_ARGS__)

#endif /* SERVICE_LOG_H */
