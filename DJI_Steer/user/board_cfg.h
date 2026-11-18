#ifndef __BOARD_CFG_H__
#define __BOARD_CFG_H__


#define BOARD_HELM   0
#define BOARD_DJI    1

#define HELM_1   1
#define HELM_2   2
#define HELM_3   3
#define HELM_4   4
#define DJI_1    5
#define DJI_2    6

#ifndef BOARD_SEL
#define BOARD_SEL   HELM_1

#endif
// 可选：HELM_1 HELM_2 HELM_3 HELM_4 DJI_1 DJI_2
// ==================================

// ---------- 选型表 ----------
#if   BOARD_SEL == HELM_1
#define BOARD_TYPE         BOARD_HELM
#define BOARD_ID           1U
#define HELM_OFFSET_DEG    0.972748f
#elif BOARD_SEL == HELM_2
#define BOARD_TYPE         BOARD_HELM
#define BOARD_ID           2U
#define HELM_OFFSET_DEG    90.00f
#elif BOARD_SEL == HELM_3
#define BOARD_TYPE         BOARD_HELM
#define BOARD_ID           3U
#define HELM_OFFSET_DEG    0.000000f
#elif BOARD_SEL == HELM_4
#define BOARD_TYPE         BOARD_HELM
#define BOARD_ID           4U
#define HELM_OFFSET_DEG    -90.435791f
#elif BOARD_SEL == DJI_1
#define BOARD_TYPE         BOARD_DJI
#define BOARD_ID           5U
#define DJI_MODEL          DJI_MODEL_3508    // DJI_MODEL_2006 / DJI_MODEL_3508
#define DJI_RATIO          19.2032f              // 减速比
#elif BOARD_SEL == DJI_2
#define BOARD_TYPE         BOARD_DJI
#define BOARD_ID           6U
#define DJI_MODEL          DJI_MODEL_2006
#define DJI_RATIO          36.0f
#else
#error "BOARD_SEL 没选或拼错了，只能是 HELM_1~4 或 DJI_1~2"
#endif

// 舵轮减速比，四块板一样
#if BOARD_TYPE == BOARD_HELM
#define HELM_RATIO         57.6f
#endif

// 电机数量根据板子类型来定
#if BOARD_TYPE == BOARD_DJI
#define DJI_MOTOR_NUM      2U
#else
#define DJI_MOTOR_NUM      1U
#endif

#endif
