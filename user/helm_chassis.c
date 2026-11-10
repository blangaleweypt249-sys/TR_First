#include "helm_chassis.h"
#include "mymain.h"

#define motor_diameter 0.090 // 轮子半径

VESC vesc_motor[4];
DJI_Motor helm_motor[4];
PID pid_CarAnale;
Angle car_angle;

calibration helm_calibration_data = {0}; // 光电门
volatile uint8_t ready_target_initialized = 0U;

float wrap_pi(float angle)
{
    if (angle > PI || angle < -PI)
    {
        angle = fmodf(angle, 2.0f * PI);
        if (angle > PI)
        {
            angle -= 2.0f * PI;
        }
        else if (angle < -PI)
        {
            angle += 2.0f * PI;
        }
    }
    return angle;
}

float wrap_angle(float angle)
{
    if (angle > 180.0f || angle < -180.0f)
    {
        angle = fmodf(angle, 360.0f);
        if (angle > 180.0f)
        {
            angle -= 360.0f;
        }
        else if (angle < -180.0f)
        {
            angle += 360.0f;
        }
    }
    return angle;
}

void Min_Angle(float *delta, float *speed)
{
    if (*delta > PI / 2.0f)
    {
        *delta -= PI;
        *speed *= -1.0f;
    }
    else if (*delta < -PI / 2.0f)
    {
        *delta += PI;
        *speed *= -1.0f;
    }
    else
    {
        *speed *= 1.0f;
    }
}

float CosKspeed(float set_angle, float actual_angle)
{
    float delta = wrap_angle(set_angle - actual_angle) * (PI / 180.0f);

    if (delta > PI / 2.0f)
    {
        delta = PI / 2.0f;
    }
    else if (delta < -PI / 2.0f)
    {
        delta = -PI / 2.0f;
    }
    else if (fabsf(delta) < 0.05f)
    {
        delta = 0.0f;
    }
    float cos_angle = cosf(delta);
    if (cos_angle < 0.0f)
    {
        cos_angle = 0.0f;
    }

    return cos_angle * cos_angle * cos_angle;
}

void helm_calculate(Speed *body_speed, Helm_chassis *helm_chassises)
{
    if (body_speed->vx == 0.0f && body_speed->vy == 0.0f && fabsf(body_speed->vw) < 0.05f)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_chassises->helm_speed[i] = 0.0f;
        }
        helm_chassises->helm_angle[0] = -atan2f(+half_dy, -half_dx);
        helm_chassises->helm_angle[1] = -atan2f(+half_dy, +half_dx);
        helm_chassises->helm_angle[2] = -atan2f(-half_dy, +half_dx);
        helm_chassises->helm_angle[3] = -atan2f(-half_dy, -half_dx);
    }
    else
    {
        helm_chassises->helm_angle[0] = atan2f(body_speed->vy + half_dy * body_speed->vw, body_speed->vx - half_dx * body_speed->vw);
        helm_chassises->helm_angle[1] = atan2f(body_speed->vy + half_dy * body_speed->vw, body_speed->vx + half_dx * body_speed->vw);
        helm_chassises->helm_angle[2] = atan2f(body_speed->vy - half_dy * body_speed->vw, body_speed->vx + half_dx * body_speed->vw);
        helm_chassises->helm_angle[3] = atan2f(body_speed->vy - half_dy * body_speed->vw, body_speed->vx - half_dx * body_speed->vw);

        helm_chassises->helm_speed[0] = -sqrtf(powf(body_speed->vy + half_dy * body_speed->vw, 2) + powf(body_speed->vx - half_dx * body_speed->vw, 2)) * 60.0f * 14.0f / ((2.0f * PI) * motor_diameter);
        helm_chassises->helm_speed[1] = sqrtf(powf(body_speed->vy + half_dy * body_speed->vw, 2) + powf(body_speed->vx + half_dx * body_speed->vw, 2)) * 60.0f * 14.0f / ((2.0f * PI) * motor_diameter);
        helm_chassises->helm_speed[2] = -sqrtf(powf(body_speed->vy - half_dy * body_speed->vw, 2) + powf(body_speed->vx + half_dx * body_speed->vw, 2)) * 60.0f * 14.0f / ((2.0f * PI) * motor_diameter);
        helm_chassises->helm_speed[3] = sqrtf(powf(body_speed->vy - half_dy * body_speed->vw, 2) + powf(body_speed->vx - half_dx * body_speed->vw, 2)) * 60.0f * 14.0f / ((2.0f * PI) * motor_diameter);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        float desired_angle = wrap_pi(helm_chassises->helm_angle[i]);
        float feedback_angle = helm_motor[i].Feedback_angle * (PI / 180.0f);
        float normalized_feedback_angle = wrap_pi(feedback_angle);

        helm_chassises->actual_helm_angle[i] = normalized_feedback_angle;

        helm_chassises->delta[i] = wrap_pi(desired_angle - normalized_feedback_angle);
        Min_Angle(&helm_chassises->delta[i], &helm_chassises->helm_speed[i]);

        helm_chassises->helm_angle[i] = feedback_angle + helm_chassises->delta[i];
        helm_chassises->set_angle[i] = (helm_chassises->helm_angle[i] / PI) * 180.0f; // 转角度
    }
}

void Helm_Calibration(void)
{
    while (!helm_calibration_data.all_helm_success_flag)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            if (helm_calibration_data.helm_calibration_flag[i])
                continue;

            GPIO_PinState pin_state;

            switch (i)
            {
                case 0: pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); break;
                case 1: pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); break;
                case 2: pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2); break;
                default: pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); break;
            }

            if (pin_state == GPIO_PIN_SET)
            {
                DJI_Speed(&helm_motor[i], 0.0f);
                DJI_SignZero(&helm_motor[i]);
                helm_motor[i].calibration_valid = true;
                helm_calibration_data.helm_calibration_flag[i] = true;
            }
            else
            {
                DJI_Speed(&helm_motor[i], 15.0f);
            }
        }

        helm_calibration_data.all_helm_success_flag =
            helm_calibration_data.helm_calibration_flag[0] &&
            helm_calibration_data.helm_calibration_flag[1] &&
            helm_calibration_data.helm_calibration_flag[2] &&
            helm_calibration_data.helm_calibration_flag[3];

        osDelay(1);
    }

    // 四个舵轮回零
    for (uint16_t count = 0; count < 200; count++)
    {
        for (uint8_t i = 0; i < 4; i++)
            DJI_ToZero(&helm_motor[i]);

        osDelay(1);
    }

    ready_target_initialized = 1U;
    osSemaphoreRelease(ChassisReadyHandle);
}

float speed_planning(float target, float *last_target, float accel_step, float decel_step)
{
    float current = *last_target;
    float delta = target - current;
    float step;

    if (current > 0.0f)
    {
        step = (target > current) ? accel_step : decel_step;
    }
    else if (current < 0.0f)
    {
        step = (target < current) ? accel_step : decel_step;
    }
    else
    {
        step = accel_step;
    }

    if (fabsf(delta) <= step)
    {
        *last_target = target;
    }
    else
    {
        *last_target += (delta > 0.0f) ? step : -step;
    }

    return *last_target;
}

void World_to_body(Speed *world_speed, Speed *body_speed, float imu_angle)
{
    float cos_angle = cosf(imu_angle);
    float sin_angle = sinf(imu_angle);

    body_speed->vx = world_speed->vx * cos_angle + world_speed->vy * sin_angle;
    body_speed->vy = -world_speed->vx * sin_angle + world_speed->vy * cos_angle;
    body_speed->vw = -world_speed->vw;
}

void car_angle_maintain(Speed *world_speed, Angle *angle)
{
    angle->angle_out = calc_pid(&pid_CarAnale, angle->actual_angle, angle->target_angle);
    world_speed->vw = angle->angle_out;
}
