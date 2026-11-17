#ifndef __DJI_H__
#define __DJI_H__

#include "main.h"
#include "pid.h"
#include "board_cfg.h"

// 电机型号，编译期决定
#define DJI_MODEL_2006   0
#define DJI_MODEL_3508   1

// 命令，数值必须和主控一致：0停止 1速度 2位置 3回零 4标零 5电流
typedef enum
{
    DJI_STOP = 0,       //停止，电流给0
    DJI_SPEED,          //速度模式
    DJI_POSITION_MODE,  //位置模式
    DJI_TO_ZERO,        //回零，转到偏置角
    DJI_SIGN_ZERO,      //标零，当前位置记为零点
    DJI_CURRENT         //电流模式
} DJI_Mode;

// 反馈状态位，按位或进 status
typedef enum
{
    DJI_ONLINE = 0x01U,     // 收到过电机反馈
    DJI_ZEROED = 0x02U,     // 标过零
    DJI_BUS_OFF = 0x04U,    // 总线关闭过
    DJI_TIMEOUT = 0x08U     // 命令超时
} DJI_Status;

// 命令帧基址：2006 用 0x300，3508 用 0x400
#if defined(DJI_MODEL) && (DJI_MODEL == DJI_MODEL_3508)
#define DJI_COMMAND_BASE   0x400U
#else
#define DJI_COMMAND_BASE   0x300U
#endif
#define DJI_FEEDBACK_BASE  (DJI_COMMAND_BASE + 0x90U)

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t board_id;           // 小板 id
    uint8_t motor_id;           // 槽位 1~4
    DJI_Mode mode;

    float ratio;                // 减速比
    float speed_max;            // 输出轴转速上限 单位：rpm
    float current_max;          // 电流上限
    float target_speed;
    float target_position;      // 输出轴角度 单位：度
    int16_t target_current;
    int16_t output_current;

    PID speed_pid;
    PID position_pid;

    // 反馈
    uint16_t encoder;
    uint16_t last_encoder;
    int32_t total_encoder;
    int16_t speed_rpm;
    int16_t torque;

    float angle;                // 输出轴角度 单位：度
    float zero_offset;          // 偏置角 单位：度

    uint32_t last_command_tick;
    uint8_t first_feedback;
    uint8_t online;
    uint8_t to_zero_active;
    uint8_t status;
    uint16_t stead_flag;        // 到位保持计数，1kHz 一个数
} DJI_Motor;

extern DJI_Motor dji_motor[DJI_MOTOR_NUM];

void DJI_Init(DJI_Motor *motor, CAN_HandleTypeDef *hcan, uint8_t board_id, uint8_t motor_id);
void DJI_Receive(DJI_Motor *motor, CAN_RxHeaderTypeDef *header, uint8_t data[8]);
void DJI_SetCommand(DJI_Motor *motor, DJI_Mode mode, float target);
void DJI_Control(DJI_Motor *motor);
void DJI_Transmit(CAN_HandleTypeDef *hcan);
void DJI_Back_To_Master(DJI_Motor *motor, CAN_HandleTypeDef *hcan);
void DJI_SetCurrent(DJI_Motor *motor, int16_t current);
void DJI_SignZero(DJI_Motor *motor);
void DJI_Stop(DJI_Motor *motor);

#endif
