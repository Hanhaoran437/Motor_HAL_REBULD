/* Config 层是"事实表"，正常情况下这里几乎不该有代码。
 * 这个文件目前只是为了让 add_library(cfg OBJECT ...) 有源文件。 */
#include "types.h"

void Cfg_Board_Noop(void)
{
    /* 保留：将来若出现"板级常量初始化"，放这里 */
}