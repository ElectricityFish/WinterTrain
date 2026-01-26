#ifndef __OLEDMENU
#define __OLEDMENU
/* ==============================================================================================
                                        宏定义和枚举类型定义
   ============================================================================================== */
#define OLED_MAX_LINE                   (8 )
#define CURRENT_OPTION_COUNT            (interface[current_interface].option_count)
#define CURRENT_OPTION_MODE             (interface[current_interface].option_mode[current_option_index])
#define CURRENT_OPTION_VALUE            (interface[current_interface].option_value[current_option_index])
#define CURRENT_OPTION_VALUE_RANGE      (interface[current_interface].value_range[current_option_index])
#define NEED_FLASH_MENU_COUNT           6

extern float SpeedLeft, SpeedRight;
#define LEFT_ENCODER                    SpeedLeft//encoder_get_count(TIM4_ENCODER)
#define RIGHT_ENCODER                   SpeedRight//encoder_get_count(TIM3_ENCODER)

/**
 * @brief 界面名称枚举
 * @note 与 `current_interface` 对应，`current_interface` 中存储当前所在的界面
 */
typedef enum interface_id 
{
    NONE_MENU,
    MAIN_MENU,              // - 主菜单
    BOOT_MENU,              // - 启动菜单
        BOOT_4_MENU,        // - 为模式4单开
    SENSOR_MENU,            // - 循迹权重调整

    PID_CONTROL_MENU,       // - PID菜单
        STAND_PID_MENU,     // - PID菜单下的直立PID
        SPEED_PID_MENU,     // - PID菜单下的速度PID
        TURNING_PID_MENU,   // - PID菜单下的转向PID
        SENSOR_PID_MENU,    // - PID菜单下的循迹转向PID
        TURN_PID_MENU,      // - 定转向 PID
        DISTANCE_PID_MENU   // - 距离PID
        

} interface_id;

/**
 * @brief 选项模式枚举
 * @note 用于判断当前选项的模式，以判断交互后的操作
 */
enum option_mode 
{
    NONE_MODE,      // - 无模式
    PURE_TEXT,      // - 纯文本模式
    SUBINTERFACE,   // - 子界面模式
    EDITABLE,       // - 可编辑模式
    INTERACTIBLE,   // - 可交互模式
    READ_FLASH,     // - 数据需要从flash读取
    READ_ANGLE,     // - 数据需要从其他地方读取
    READ_ENCODER,   // - 数据需要从其他地方读取
};

/**
 * @brief 启动模式枚举
 * @note 用于判断当前选项的模式，以判断交互后的操作，有 `IDLE`, `MODE_1~5`
 */
typedef enum boot_mode 
{
    IDLE,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4_RECORD,
    MODE_4_REPLAY,
    MODE_5,
} boot_mode;

/**
 * @brief 事件枚举
 * @note 专为INTERACTIBLE选项提供的枚举体，有 `BOOT_MODE_1~5`
 */
typedef enum EVENT_ID
{
    NON_EVENT,
    BOOT_MODE_1,
    BOOT_MODE_2,
    BOOT_MODE_3,
    BOOT_MODE_4_RECORD,
    BOOT_MODE_4_REPLAY,
    BOOT_MODE_5,
} EVENT_ID;

enum interact_mode
{
    EDIT_MODE,
    SELECT_MODE,
    RUNNING,
};

/* ==============================================================================================
                                        结构体定义
   ============================================================================================== */
/**
 * @brief 界面结构体定义
 * @note 这个结构体定义了一个界面里应该有什么
 */
typedef struct 
{
    interface_id    ID;                                         // 菜单ID
    short           option_count;                               // 选项数量，范围1~8
    char            option_text[OLED_MAX_LINE][20];             // 选项文本内容，定义时自己写
    short           option_mode[OLED_MAX_LINE];                 // 选项模式，详见 option_mode

    int             option_value[OLED_MAX_LINE];                // 如果选项模式是EDITABLE，那么就会显示这个数值，都是小数
    int             option_action[OLED_MAX_LINE];               // 如果选项模式是INTERACTIBLE，那么就会触发这个里面的东西
    interface_id    subinterface[OLED_MAX_LINE];                // 如果选项模式是SUBINTERFACE，那么就会触发这个里面的东西

    int             value_range[OLED_MAX_LINE];                 // 数值范围限制
    short           super_interface;                            // 父级界面索引
} Interface_TypeDef;

/* ==============================================================================================
                                        函数声明
   ============================================================================================== */

void                    Menu_Init                       (void);
void                    Menu_Refresh                    (void);
void                    Menu_Up                         (void);
void                    Menu_Down                       (void);
void                    Menu_Forward                    (void);
void                    Menu_Backward                   (void);
void                    Menu_SavePIDToFlash             (void);
double                  Menu_GetValue                   (interface_id ID, int option_index);
void                    Menu_SetValue                   (interface_id ID, int option_index, int value);
boot_mode               Menu_GetCurMode                 (void);
void                    Menu_JustRefreshValue           (void);

#endif
