#ifndef __VESC_H__
#define __VESC_H__
#include <stdint.h>
#include "fdcan.h"
#include "Interpolation.h"

typedef struct
{
   int32_t erpm;
   int16_t o_current;
   int16_t duty_cycle;
   int16_t o_temp_fet;
   int16_t o_temp_motor;
   int16_t o_current_in;
   int16_t pid_pos;
   
   float rpm;
   float duty;
   float current;
   float current_in;
   float temp_fet;
   float temp_motor;

   float actual_postion;
   float actual_speed; 
}VESC_FeedBack;

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan;
  uint8_t motor_id;
  VESC_FeedBack feedback;
  int num[3];
}VESC;

void Vesc_Init(VESC *Motor, FDCAN_HandleTypeDef *hcan, uint8_t motor_id);
void buffer_int16(uint8_t *buffer, int16_t data, int32_t *index);
void buffer_int32(uint8_t *buffer, int32_t data, int32_t *index);
void buffer_float16(uint8_t *buffer, float data, float scale, int32_t *index);
void buffer_float32(uint8_t *buffer, float number, float scale, int32_t *index);
void vesc_set_duty(VESC *vesc, float duty);
void vesc_set_current(VESC *vesc, float current);
void vesc_set_current_off_delay(VESC *vesc, float current, float off_delay);
void vesc_set_current_rel(VESC *vesc, float current_rel);
void vesc_set_current_rel_off_delay(VESC *vesc, float current_rel, float off_delay);
void vesc_set_current_brake(VESC *vesc, float current);
void vesc_set_current_brake_rel(VESC *vesc, float current_rel);
void vesc_set_rpm(VESC *vesc, float rpm);
void vesc_set_pos(VESC *vesc, float pos, uint8_t interpolation_enable,
                  uint32_t run_time_ms);
void vesc_set_handbrake(VESC *vesc, float current);
void vesc_set_handbrake_rel(VESC *vesc, float current_rel);
void VESC_Analysisdata(VESC *vesc,FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);
HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t type, uint32_t id, uint8_t *data, uint8_t len);

#endif
