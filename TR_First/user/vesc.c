#include "vesc.h"
#include <string.h>

#define MOTOR_POLE_PAIRS 14.0f  // rpm=erpm/pole pairs
#define MOTOR_MAX_CURRENT 60.0f // 最大电流限制为60A
#define MOTOR_MAX_BRAKE_CURRENT 60.0f
#define MOTOR_MAX_RPM 10000.0f

typedef enum
{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

void Vesc_Init(VESC *Motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    if (Motor == NULL)
    {
        return;
    }
    memset(Motor, 0, sizeof(*Motor));
    Motor->hfdcan = hfdcan;
    Motor->motor_id = motor_id;
}

// 大端序
void buffer_int16(uint8_t *buffer, int16_t data, int32_t *index)
{
    buffer[(*index)++] = (data >> 8) & 0xFF;
    buffer[(*index)++] = data & 0xFF;
}

void buffer_int32(uint8_t *buffer, int32_t data, int32_t *index)
{
    buffer[(*index)++] = (data >> 24) & 0xFF;
    buffer[(*index)++] = (data >> 16) & 0xFF;
    buffer[(*index)++] = (data >> 8) & 0xFF;
    buffer[(*index)++] = data & 0xFF;
}

void buffer_float16(uint8_t *buffer, float data, float scale, int32_t *index)
{
    buffer_int16(buffer, (int16_t)(data * scale), index);
}

void buffer_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_int32(buffer, (int32_t)(number * scale), index);
}

void vesc_set_duty(VESC *vesc, float duty)
{
    if (duty > 1.0f)
        duty = 1.0f;
    if (duty < -1.0f)
        duty = -1.0f;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_int32(buffer, (int32_t)(duty * 100000.0), &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_DUTY << 8), buffer, 4);
}

void vesc_set_current(VESC *vesc, float current)
{
    if (current > MOTOR_MAX_CURRENT)
        current = MOTOR_MAX_CURRENT;
    if (current < -MOTOR_MAX_CURRENT)
        current = -MOTOR_MAX_CURRENT;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_int32(buffer, (int32_t)(current * 1000.0), &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT << 8), buffer, 4);
}

void vesc_set_current_off_delay(VESC *vesc, float current, float off_delay)
{
    if (current > MOTOR_MAX_CURRENT)
        current = MOTOR_MAX_CURRENT;
    if (current < -MOTOR_MAX_CURRENT)
        current = -MOTOR_MAX_CURRENT;
    int32_t index = 0;
    uint8_t buffer[6];
    buffer_int32(buffer, (int32_t)(current * 1000.0), &index);
    buffer_float16(buffer, off_delay, 1e3, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT << 8), buffer, 6);
}

void vesc_set_current_brake(VESC *vesc, float current)
{
    if (current > MOTOR_MAX_BRAKE_CURRENT)
        current = MOTOR_MAX_BRAKE_CURRENT;
    if (current < -MOTOR_MAX_BRAKE_CURRENT)
        current = -MOTOR_MAX_BRAKE_CURRENT;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_int32(buffer, (int32_t)(current * 1000.0), &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, 4);
}

void vesc_set_rpm(VESC *vesc, float rpm) // erpm
{
    if (rpm > MOTOR_MAX_RPM)
        rpm = MOTOR_MAX_RPM;
    if (rpm < -MOTOR_MAX_RPM)
        rpm = -MOTOR_MAX_RPM;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_int32(buffer, (int32_t)rpm, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_RPM << 8), buffer, 4);
}

// vesc不进行五次插值处理
void vesc_set_pos(VESC *vesc, float pos, uint8_t interpolation_enable,
                  uint32_t run_time_ms)
{
    if (pos > 360.0f)
        pos = 360.0f;
    if (pos < 0.0f)
        pos = 0.0f;

    int32_t index = 0;
    uint8_t buffer[4];
    buffer_int32(buffer, (int32_t)(pos * 1000000.0), &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_POS << 8), buffer, 4);
}

void vesc_set_current_rel(VESC *vesc, float current_rel)
{
    if (current_rel > 1)
        current_rel = 1;
    if (current_rel < -1)
        current_rel = -1;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_float32(buffer, current_rel, 1e5, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT_REL << 8), buffer, 4);
}

void vesc_set_current_rel_off_delay(VESC *vesc, float current_rel, float off_delay)
{
    if (current_rel > 1.0f)
        current_rel = 1.0f;
    if (current_rel < -1.0f)
        current_rel = -1.0f;
    int32_t index = 0;
    uint8_t buffer[6];
    buffer_float32(buffer, current_rel, 1e5, &index);
    buffer_float16(buffer, off_delay, 1e3, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT_REL << 8), buffer, 6);
}

void vesc_set_current_brake_rel(VESC *vesc, float current_rel)
{
    if (current_rel > 1)
        current_rel = 1;
    if (current_rel < -1)
        current_rel = -1;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_float32(buffer, current_rel, 1e5, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, 4);
}

void vesc_set_handbrake(VESC *vesc, float current)
{
    if (current > MOTOR_MAX_BRAKE_CURRENT)
        current = MOTOR_MAX_BRAKE_CURRENT;
    if (current < -MOTOR_MAX_BRAKE_CURRENT)
        current = -MOTOR_MAX_BRAKE_CURRENT;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_float32(buffer, current, 1e3, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, 4);
}

void vesc_set_handbrake_rel(VESC *vesc, float current_rel)
{
    if (current_rel > 1)
        current_rel = 1;
    if (current_rel < -1)
        current_rel = -1;
    int32_t index = 0;
    uint8_t buffer[4];
    buffer_float32(buffer, current_rel, 1e5, &index);
    CAN_Transmit(vesc->hfdcan, 1, vesc->motor_id | (CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, 4);
}

void VESC_Analysisdata(VESC *vesc, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if ((rx_header == NULL) || (vesc == NULL) || (data == NULL) || (rx_header->DataLength != FDCAN_DLC_BYTES_8))
    {
        return;
    }

    if ((rx_header->Identifier & 0xFFU) == vesc->motor_id)
    {
        switch ((rx_header->Identifier >> 8) & 0xFF)
        {
        case 9: // CAN_PACKET_STATUS
            vesc->feedback.erpm = (int32_t)(((uint32_t)data[0] << 24) |
                                            ((uint32_t)data[1] << 16) |
                                            ((uint32_t)data[2] << 8) |
                                            (uint32_t)data[3]);
            vesc->feedback.o_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            vesc->feedback.duty_cycle = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
            vesc->feedback.current = vesc->feedback.o_current / 10.0f;
            vesc->feedback.duty = vesc->feedback.duty_cycle / 1000.0f;
            vesc->feedback.rpm = (float)vesc->feedback.erpm / MOTOR_POLE_PAIRS; // 换算成机械rpm
            vesc->feedback.actual_speed = vesc->feedback.rpm;
            if (vesc->num[0] >= 1000)
                vesc->num[0] = 0;
            else
                vesc->num[0]++;
            break;

        case 16: // CAN_PACKET_STATUS_4
            vesc->feedback.o_temp_fet = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
            vesc->feedback.o_temp_motor = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
            vesc->feedback.o_current_in = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            vesc->feedback.pid_pos = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
            vesc->feedback.temp_fet = vesc->feedback.o_temp_fet / 10.0f;
            vesc->feedback.temp_motor = vesc->feedback.o_temp_motor / 10.0f;
            vesc->feedback.current_in = vesc->feedback.o_current_in / 10.0f;
            vesc->feedback.actual_postion = vesc->feedback.pid_pos / 50.0f;
            if (vesc->num[0] >= 1000)
                vesc->num[0] = 0;
            else
                vesc->num[0]++;
            break;
        }
    }
}
