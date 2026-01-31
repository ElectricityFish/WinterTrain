/************************************************************************************************
* @name 超级无敌可拓展菜单系统
* @note 一个高度可自定义、可拓展的、封装完好的菜单系统
* @author 小麦香又甜
*************************************************************************************************/
#include "zf_common_headfile.h"
#include "Menu.h"
#include "Inertial_Navigation.h"

extern uint16_t task_two_stop_flag;

// 注意，为了方便Flash存储，这个菜单里的值是用int存储的，但是其实际表示的值是除以10的。

/* ==============================================================================================
                                        界面定义
   ============================================================================================== */
Interface_TypeDef interface[100] = {
    [MAIN_MENU] = {             // 主菜单界面
        .ID = MAIN_MENU,
        .option_count = 5,
        .option_text = {
            "BOOT MODE",
            "PID CONTROL", 
            "SENSOR WEIGHT", 
            "P / Y", 
            "L / R",
        },
        .option_mode = {
            SUBINTERFACE, 
            SUBINTERFACE, 
            SUBINTERFACE, 
            READ_ANGLE, 
            READ_ENCODER
        },
        .super_interface = MAIN_MENU,
        .subinterface = {
            BOOT_MENU, 
            PID_CONTROL_MENU, 
            SENSOR_MENU
        },
    },    
    [BOOT_MENU] = {             // 启动界面
        .ID = BOOT_MENU,
        .option_count = 7,
        .option_text = {
            "Boot Mode 1", 
            "Boot Mode 2", 
            "Boot Mode 3", 
            "Boot Mode 4", 
            "Boot Mode 5", 
            "P / Y", 
            "stop"
        },
        .option_mode = {
            INTERACTIBLE, 
            INTERACTIBLE, 
            INTERACTIBLE, 
            SUBINTERFACE, 
            INTERACTIBLE, 
            READ_ANGLE, 
            READ_ENCODER
        },
        .option_action = {
            BOOT_MODE_1, 
            BOOT_MODE_2, 
            BOOT_MODE_3, 
            NON_EVENT, 
            BOOT_MODE_5
        },
        .super_interface = MAIN_MENU,
        .subinterface = {
            NONE_MENU, 
            NONE_MENU, 
            NONE_MENU, 
            BOOT_4_MENU
        },
    },
    [BOOT_4_MENU] = {             // 模式4界面
        .ID = BOOT_4_MENU,
        .option_count = 4,
        .option_text = {
            "Record", 
            "Replay", 
            "P / Y", 
            "L / R"
        },
        .option_mode = {
            INTERACTIBLE, 
            INTERACTIBLE, 
            READ_ANGLE, 
            READ_ENCODER
        },
        .option_action = {
            BOOT_MODE_4_RECORD, 
            BOOT_MODE_4_REPLAY
        },
        .super_interface = BOOT_MENU,
    },
    [PID_CONTROL_MENU] = {             // PID界面
        .ID = PID_CONTROL_MENU,
        .option_count = 6,
        .option_text = {
            "ANGLE PID", 
            "SPEED PID", 
            "TURNING PID", 
            "SENSOR PID",   
            "TURN PID", 
            "DISTANCE PID",
        },
        .option_mode = {
            SUBINTERFACE, 
            SUBINTERFACE, 
            SUBINTERFACE, 
            SUBINTERFACE,
            SUBINTERFACE,
            SUBINTERFACE,
        },
        .super_interface = MAIN_MENU,
        .subinterface = {
            STAND_PID_MENU, 
            SPEED_PID_MENU, 
            TURNING_PID_MENU, 
            SENSOR_PID_MENU,
            TURN_PID_MENU,
            DISTANCE_PID_MENU,
        },
    },
    [STAND_PID_MENU] = {             // Stand PID界面
        .ID = STAND_PID_MENU,
        .option_count = 7,
        .option_text = {
            "ANGLE kP", 
            "ANGLE kI", 
            "ANGLE kD", 
            "====x100====", 
            "kP save", 
            "kI save", 
            "kD save"
        },
        .option_mode = {
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            PURE_TEXT, 
            READ_FLASH, 
            READ_FLASH, 
            READ_FLASH
        },
        .option_value = {10, 10, 10, -1, 1, 1, 1},
        .value_range = {100, 100, 100},
        .super_interface = PID_CONTROL_MENU,
    },
    [SPEED_PID_MENU] = {             // Speed PID界面
        .ID = SPEED_PID_MENU,
        .option_count = 7,
        .option_text = {
            "Speed kP", 
            "Speed kI", 
            "Speed kD", 
            "=====x1=====", 
            "kP save", 
            "kI save", 
            "kD save"
        },
        .option_mode = {
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            PURE_TEXT, 
            READ_FLASH, 
            READ_FLASH, 
            READ_FLASH
        },
        .option_value = {10, 10, 10, -1, 0, 0, 0},
        .value_range = {100, 100, 100},
        .super_interface = PID_CONTROL_MENU,
    },
    [TURNING_PID_MENU] = {             // Turning PID界面
        .ID = TURNING_PID_MENU,
        .option_count = 7,
        .option_text = {
            "Turn kP", 
            "Turn kI", 
            "Turn kD", 
            "=====x1=====", 
            "kP save", 
            "kI save", 
            "kD save"
        },
        .option_mode = {
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            PURE_TEXT, 
            READ_FLASH, 
            READ_FLASH, 
            READ_FLASH
        },
        .option_value = {10, 10, 10, -1, 0, 0, 0},
        .value_range = {100, 100, 100},
        .super_interface = PID_CONTROL_MENU,
    },
    [SENSOR_PID_MENU] = {             // sensor PID界面
        .ID = SENSOR_PID_MENU,
        .option_count = 7,
        .option_text = {
            "Sensr kP", 
            "Sensr kI", 
            "Sensr kD", 
            "=====x1=====", 
            "kP save", 
            "kI save", 
            "kD save"},
        .option_mode = {
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            PURE_TEXT,
            READ_FLASH, 
            READ_FLASH, 
            READ_FLASH
        },
        .option_value = {10, 10, 10, -1, 0, 0, 0},
        .value_range = {100, 100, 100},
        .super_interface = PID_CONTROL_MENU,
    },
    [TURN_PID_MENU] = {             // 定转向 PID界面
        .ID = TURN_PID_MENU,
        .option_count = 7,
        .option_text = {
            "Turn kP", 
            "Turn kI", 
            "Turn kD", 
            "=====x1=====", 
            "kP save", 
            "kI save", 
            "kD save"},
        .option_mode = {
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            PURE_TEXT,
            READ_FLASH, 
            READ_FLASH, 
            READ_FLASH
        },
        .option_value = {10, 10, 10, -1, 0, 0, 0},
        .value_range = {100, 100, 100},
        .super_interface = PID_CONTROL_MENU,
    },
    [DISTANCE_PID_MENU] = {             // 定转向 PID界面
        .ID = DISTANCE_PID_MENU,
        .option_count = 7,
        .option_text = {
            "Dista kP", 
            "Dista kI", 
            "Dista kD", 
            "=====x1=====", 
            "kP save", 
            "kI save", 
            "kD save"},
        .option_mode = {
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            PURE_TEXT,
            READ_FLASH, 
            READ_FLASH, 
            READ_FLASH
        },
        .option_value = {10, 10, 10, -1, 0, 0, 0},
        .value_range = {100, 100, 100},
        .super_interface = PID_CONTROL_MENU,
    },
    [SENSOR_MENU] = {             // SENSOR权重界面
        .ID = SENSOR_MENU,
        .option_count = 5,
        .option_text = {
            "this wont save", 
            "level-1", 
            "level-2", 
            "level-3", 
            "level-4"
        },
        .option_mode = {
            PURE_TEXT, 
            EDITABLE, 
            EDITABLE, 
            EDITABLE, 
            EDITABLE
        },
        .option_value = {-1, 10, 30, 50, 80},
        .value_range = {-1, 100, 100, 100, 100},
        .super_interface = MAIN_MENU,
    },
};

/* ==============================================================================================
                                        全局变量定义
   ============================================================================================== */

interface_id            current_interface;              // 目前的界面ID
short                   current_option_index;           // 目前选的选项的index
short                   current_mode;                   // 目前的模式：EDIT_MODE 和 SELECT_MODE
boot_mode               running_mode;                   // 车是否在某个状态运行中

// 如果要新增PID，你需要去 NEED_FLASH_MENU)COUNT 那边改一下
interface_id NEED_FLASH_MENU_ID [NEED_FLASH_MENU_COUNT] = {
    STAND_PID_MENU,
    SPEED_PID_MENU,
    TURNING_PID_MENU,
    SENSOR_PID_MENU,
    TURN_PID_MENU,
    DISTANCE_PID_MENU,
};

extern float pitch;
extern float yaw;

/* ==============================================================================================
                                        内部函数定义
   ============================================================================================== */

/** 
 * @brief 上电后数据初始化
 * @note 由于每次上电时上面的interface都会把PID给初始化掉，所以专门搞了个函数弄数据保持的
 * @return void
 */
void Menu_ReadFlashToValue(void)
{
    flash_read_page_to_buffer(127, 0);
    for (int i = 0;i < NEED_FLASH_MENU_COUNT;i++) {
        interface[NEED_FLASH_MENU_ID[i]].option_value[0] = flash_union_buffer[i * 3 + 0].int8_type;
        interface[NEED_FLASH_MENU_ID[i]].option_value[1] = flash_union_buffer[i * 3 + 1].int8_type;
        interface[NEED_FLASH_MENU_ID[i]].option_value[2] = flash_union_buffer[i * 3 + 2].int8_type;
    }
    flash_buffer_clear();
}

// 1 为是
int Menu_CheckNeedFlash (void)
{
    for (int i = 0; i < NEED_FLASH_MENU_COUNT; i++) {
        if (current_interface == NEED_FLASH_MENU_ID[i]) {
            return i;
        }
    }
    return -1;
}

/** 
 * @brief 刷新数据
 * @note 专门为 Flash 模块设计的DLC，用于Refresh函数的过渡模块
 * @return void
 */
void Menu_RefreshValue(void)
{
    int Memory_position = Menu_CheckNeedFlash();
    if (Memory_position == -1) {
        return;
    }

    flash_read_page_to_buffer(127, 0);
    interface[current_interface].option_value[4] = flash_union_buffer[Memory_position * 3 + 0].int8_type; // P
    interface[current_interface].option_value[5] = flash_union_buffer[Memory_position * 3 + 1].int8_type; // I
    interface[current_interface].option_value[6] = flash_union_buffer[Memory_position * 3 + 2].int8_type; // D
    // 读取并存储完毕
}

/** 
 * @brief 触发事件
 * @note 对于那些启动项来说，就需要用到这个函数，一是要区分ID，二是要置标志位：`is_running` 和 `current_mode`
 * @param action_id 详见 `interface` 内部的定义，注意这里是index
 * @return void
 */
void Menu_Event(EVENT_ID action_id)
{
    switch(action_id){
        case BOOT_MODE_1:
            running_mode = MODE_1;
            current_mode = RUNNING;
            break;
        case BOOT_MODE_2:
            running_mode = MODE_2;
            current_mode = RUNNING;
            break;
        case BOOT_MODE_3:
            running_mode = MODE_3;
            current_mode = RUNNING;
            break;
        case BOOT_MODE_4_RECORD:
            running_mode = MODE_4_RECORD;
            current_mode = RUNNING;
			Inertial_Nav_StartRecord();  // 开始记录路径
            break;
        case BOOT_MODE_4_REPLAY:
            running_mode = MODE_4_REPLAY;
            current_mode = RUNNING;
			Inertial_Nav_StartReplay();  // 开始回放路径
            break;
        case BOOT_MODE_5:
            running_mode = MODE_5;
            current_mode = RUNNING;
            break;
        default:
            break;
    }
}

/* ==============================================================================================
                                        外部函数定义
   ============================================================================================== */

/** 
 * @brief 菜单的初始化函数
 * @note 直接使用 `Menu_Init();` 即可
 * @return void
 */
void Menu_Init(void)
{
	oled_init();
    oled_set_dir(OLED_CROSSWISE);
    oled_clear();
    current_interface = MAIN_MENU;
    current_option_index = 0;
    current_mode = SELECT_MODE;
    Menu_ReadFlashToValue();
    Menu_Refresh();
}

/** 
 * @brief 刷新界面
 * @note 根据当前的信息在OLED上打印信息。
 * @return void
 */
void Menu_Refresh(void)
{
    Menu_RefreshValue();
    // 显示选择箭头 `>`
    for (int i = 0; i < CURRENT_OPTION_COUNT; i++) {
        if (i == current_option_index) {
            oled_show_string(0, i, "> ");
        } else {
            oled_show_string(0, i, "  "); 
        }
    }

    // 显示文本 'TEXT'
    for (int i = 0; i < CURRENT_OPTION_COUNT; i++) {
        oled_show_string(10, i, interface[current_interface].option_text[i]);
    }

    // 显示模式 'EDT / RUN'
    if (current_mode == EDIT_MODE) {
        oled_show_string(90, 7, "EDT");
    } else if (current_mode == RUNNING) {
        oled_show_string(90, 7, "RUN");
    } else {
        oled_show_string(90, 7, "   ");
    }

    // 清除SAVED!
    oled_show_string(80, 6, "     ");

    // 显示数据
    for (int i = 0; i < CURRENT_OPTION_COUNT; i++) {
        if (interface[current_interface].option_mode[i] == EDITABLE || interface[current_interface].option_mode[i] == READ_FLASH) {
            oled_show_float(60, i, interface[current_interface].option_value[i] / 10.0f, 2, 1);
        } else if (interface[current_interface].option_mode[i] == READ_ANGLE) {
            oled_show_float(60, i, pitch, 3, 1);
            oled_show_float(100, i, yaw, 3, 1);
        } else if (interface[current_interface].option_mode[i] == READ_ENCODER) {
            oled_show_int(60, i, task_two_stop_flag, 1);
//            oled_show_int(100, i, RIGHT_ENCODER, 3);
        }
    }

}

/** 
 * @brief 我一看，哇，数据怎么都不动，原来是我屏幕没刷新
 * @note 你应该把这个放在某个中断里
 * @return void
 */
void Menu_JustRefreshValue(void)
{
    // 显示数据
    for (int i = 0; i < CURRENT_OPTION_COUNT; i++) {
        if (interface[current_interface].option_mode[i] == EDITABLE || interface[current_interface].option_mode[i] == READ_FLASH) {
            oled_show_float(60, i, interface[current_interface].option_value[i] / 10.0f, 2, 1);
        } else if (interface[current_interface].option_mode[i] == READ_ANGLE) {
            oled_show_float(60, i, pitch, 3, 1);
            oled_show_float(100, i, yaw, 3, 1);
        } else if (interface[current_interface].option_mode[i] == READ_ENCODER) {
            oled_show_int(60, i, task_two_stop_flag, 1);
//            oled_show_int(100, i, RIGHT_ENCODER, 3);
        }
    }  
}

/** 
 * @brief 存数据
 * @note 都写在名字上了
 * @return void
 */
void Menu_SavePIDToFlash(void)
{
    int Memory_position = Menu_CheckNeedFlash();
    if (Memory_position == -1) {
        return;
    }
    // 是这样的，为了压缩空间，我把6个PID全部存在一页里面了，也就是说，每一次Save都需要一个页存，一个页Save
    if (flash_check(127, 0)) {
        flash_erase_page(127, 0);
    } 
    flash_buffer_clear();
    // 清缓存，写缓存，发缓存
    for (int i = 0;i < NEED_FLASH_MENU_COUNT;i++) {
        flash_union_buffer[i * 3 + 0].int8_type = interface[NEED_FLASH_MENU_ID[i]].option_value[0];
        flash_union_buffer[i * 3 + 1].int8_type = interface[NEED_FLASH_MENU_ID[i]].option_value[1];
        flash_union_buffer[i * 3 + 2].int8_type = interface[NEED_FLASH_MENU_ID[i]].option_value[2];
    }
    flash_write_page_from_buffer(127, 0);
    // 存储完成
    oled_show_string(100, 6, "SVD");
}

/** 
 * @brief 向上操作，对应 E5
 * @note 和按钮热血沸腾的组合技
 * @return void
 */
void Menu_Up(void)
{
    if (current_mode == SELECT_MODE) {
        current_option_index = ((current_option_index - 1) + CURRENT_OPTION_COUNT) % CURRENT_OPTION_COUNT; // 合理性检查 -> 一个界面3个按钮，取值范围为 0 ~ 2，合理！
    } else if (current_mode == EDIT_MODE) {
        CURRENT_OPTION_VALUE = (CURRENT_OPTION_VALUE + 1) % CURRENT_OPTION_VALUE_RANGE;
    }

    Menu_Refresh();
}

/** 
 * @brief 向下操作，对应E4
 * @note 和按钮热血沸腾的组合技
 * @return void
 */
void Menu_Down(void)
{
    if (current_mode == SELECT_MODE) {
        current_option_index = ((current_option_index + 1) + CURRENT_OPTION_COUNT) % CURRENT_OPTION_COUNT;
    } else if (current_mode == EDIT_MODE) {
        CURRENT_OPTION_VALUE = (CURRENT_OPTION_VALUE - 1) % CURRENT_OPTION_VALUE_RANGE;
    }

    Menu_Refresh();
}

/** 
 * @brief 确定，前进操作，对应E3
 * @note 和按钮热血沸腾的组合技
 * @return void
 */
void Menu_Forward(void)
{
    if (current_mode == SELECT_MODE) {
        if (CURRENT_OPTION_MODE == SUBINTERFACE) {
            // 转到下级菜单
            current_interface = interface[current_interface].subinterface[current_option_index];
            current_option_index = 0;
            oled_clear();
        } else if (CURRENT_OPTION_MODE == EDITABLE) {
            // 切换到编辑模式
            current_mode = EDIT_MODE;
        } else if (CURRENT_OPTION_MODE == INTERACTIBLE) {
            // 这一块写启动函数
            Menu_Event(interface[current_interface].option_action[current_option_index]);
        }
    } else if (current_mode == EDIT_MODE) {
        // 切换回选择模式
        current_mode = SELECT_MODE;
    } else if (current_mode == RUNNING) {
        // 切换回选择模式
        current_mode = SELECT_MODE;
        running_mode = IDLE;
    }

    Menu_Refresh();
}

/** 
 * @brief 取消，后退操作，对应E2
 * @note 和按钮热血沸腾的组合技
 * @return void
 */
void Menu_Backward(void)
{
    if (current_mode == SELECT_MODE) {
        // 返回上级菜单
        current_interface = interface[current_interface].super_interface;
        current_option_index = 0;
        oled_clear();
    } else if (current_mode == EDIT_MODE) {
        // 切换回选择模式
        current_mode = SELECT_MODE;
    } else if (current_mode == RUNNING) {
        // 切换回选择模式
        current_mode = SELECT_MODE;
        running_mode = IDLE;
    }

    Menu_Refresh();
}

/** 
 * @brief 获得某个地方的值
 * @note Getter Method，使用例如要获取SPEED PID的P值 `Menu_GetValue(SPEED_PID_MENU, 0)`
 * @param interface_id 详见 `interface_id` 枚举体
 * @param option_index 详见 `interface` 内部的定义，注意这里是index
 * @return 返回一个 `double` 值，除以10已经自动处理
 */
double Menu_GetValue(interface_id ID, int option_index){
    return interface[ID].option_value[option_index] / 10.0f;
}

/** 
 * @brief 设定某个地方的值
 * @note Setter Method，使用例如要设置SPEED PID存储的P值 `Menu_SetValue(SPEED_PID_MENU, 4, Flash的某些操作)`，虽然说我们并不会这样操作，我们会用上面的RefreshValue函数主动刷新数据
 * @param interface_id 详见 `interface_id` 枚举体
 * @param option_index 详见 `interface` 内部的定义，注意这里是index
 * @param value 一个 `int` 值
 * @return void
 */
void Menu_SetValue(interface_id ID, int option_index, int value){
    interface[ID].option_value[option_index] = value;
}

/** 
 * @brief 获得当前的运行状态
 * @note Getter Method，使用例如 boot_mode Current_mode = Menu_GetCurMode();
 * @return void
 */
boot_mode Menu_GetCurMode(void){
    return running_mode;
}

void Menu_SetRunningMode(boot_mode Mode) {
    running_mode = Mode;
	current_mode = RUNNING;
}
