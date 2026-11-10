#include "Interpolation.h"

void Interpolation_Init(Interpolation_t *motor)
{
    if (motor == NULL)
        return;

    motor->start_pos = 0.0f;
    motor->target_pos = 0.0f;
    motor->actual_pos = 0.0f;

    motor->alltime = 0.0f;
    motor->timed = 0.0f;

    motor->last_update_ms = HAL_GetTick();
    motor->state = false;//没有在插值状态
}

float Interpolation_PositionCommand(Interpolation_t *motor, float current_pos, float target_pos, uint8_t enable, uint32_t run_time_ms)
{
    uint32_t now_ms;
    float dt;

    if (motor == NULL)
        return target_pos;

    now_ms = HAL_GetTick();

    // 关闭插值，直接使用目标位置
    if ((enable == 0U) || (run_time_ms == 0U))
    {
        motor->state = false;
        motor->actual_pos = target_pos;
        motor->last_update_ms = now_ms;
        return target_pos;
    }

    // 首次启动插值，或者当前插值已经结束
    if (!motor->state)
    {
        motor->start_pos = current_pos;
        motor->target_pos = target_pos;
        motor->alltime = (float)run_time_ms;
        motor->timed = 0.0f;
        motor->actual_pos = current_pos;
        motor->last_update_ms = now_ms;
        motor->state = true;
    }
    else if (motor->target_pos != target_pos)
    {
        // 目标改变，从当前插值位置重新规划
        motor->start_pos = motor->actual_pos;
        motor->target_pos = target_pos;
        motor->alltime = (float)run_time_ms;
        motor->timed = 0.0f;
        motor->last_update_ms = now_ms;
    }

    // 计算实际经过的时间，单位转换为秒
    dt = (float)(now_ms - motor->last_update_ms) / 1000.0f;

    motor->last_update_ms = now_ms;

    if (motor->timed + dt >= motor->alltime / 1000.0f)
    {
        motor->actual_pos = motor->target_pos;
        motor->state = false;
        return motor->actual_pos;
    }

    motor->timed += dt;
    if (motor->timed > motor->alltime / 1000.0f)
    {
        motor->timed = motor->alltime / 1000.0f;
    }

    {
        float s = motor->timed / (motor->alltime / 1000.0f);
        float s2 = s * s;
        float s3 = s2 * s;

        motor->actual_pos = motor->start_pos + (motor->target_pos - motor->start_pos) * s3 * (10.0f + s * (-15.0f + 6.0f * s));
    }

    return motor->actual_pos;
}
