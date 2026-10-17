#ifndef APP_APP_H
#define APP_APP_H

#include "types.h"

/* ============================================================================
 * App 层 — 唯一知道"这是一台表型检测车"的地方
 *
 * 对 CubeMX 生成代码（Core/Src/main.c）暴露的全部接口就这两个函数：
 *     main() 里 USER CODE BEGIN 2  →  App_Init();
 *     while(1) 的 USER CODE BEGIN 3 →  App_Loop();
 *
 * app_init.c 是**整个工程的唯一集成点**：它把 Bsp 的字节回调接到 Service 的
 * 协议解析器上，并决定上电后跑什么流程。多人协作时这个文件只能一个人动。
 * ============================================================================ */

void App_Init(void);
void App_Loop(void);

#endif /* APP_APP_H */
