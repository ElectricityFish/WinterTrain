#include "zf_common_headfile.h"
#include "menu.h"
#include "Kfilter.h"

float yaw = 0;
float pitch = 0;
KalmanFilter KF;


int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	
	Kalman_Init(&KF,0.0001f,0.003f,0.03f);
	
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

