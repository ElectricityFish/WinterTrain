#include "zf_common_headfile.h"
#include "menu.h"

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART

    Menu_Init();
    key_init(10);
    while (1) {
        system_delay_ms(10);
        key_scanner();
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            Menu_Up();
        } else if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            Menu_Down();
        } else if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            Menu_Forward();
        } else if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            Menu_Backward();
        } else if (key_get_state(KEY_2) == KEY_LONG_PRESS) {
            Menu_SavePIDToFlash();
        }
    }
}
// **************************** 代码区域 ****************************

