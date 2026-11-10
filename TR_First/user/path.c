#include "path.h"

// 旋转中心点坐标
#define TURN_CENTER_X 3.0f
#define TURN_CENTER_Y 2.0f

extern Helm_chassis helm_chassis;

Wheel wheel[4];

void Cacl_wheel(float yaw, float lidar_x, float lidar_y)
{
    static const int8_t sign_x[4] = {1, 1, -1, -1};
    static const int8_t sign_y[4] = {1, -1, -1, 1};

    float cos_a = cosf(yaw);
    float sin_a = sinf(yaw);

    for (uint8_t i = 0; i < 4; i++)
    {
        wheel[i].b_x = sign_x[i] * half_dx;
        wheel[i].b_y = sign_y[i] * half_dy;

        wheel[i].w_x = lidar_x + wheel[i].b_x * cos_a - wheel[i].b_y * sin_a;
        wheel[i].w_y = lidar_y + wheel[i].b_x * sin_a + wheel[i].b_y * cos_a;
    }
}

bool QuarterLidarTurn(float turn_speed, float lidar_x, float lidar_y)
{
    static bool turning = false;
    static float start_angle = 0.0f;
    static float distance = 0.0f;

    if (!turning)
    {
        start_angle = Imu_angle;
        distance = hypotf(lidar_x - TURN_CENTER_X, lidar_y - TURN_CENTER_Y);
        turning = true;
    }

    float delta = wrap_pi(Imu_angle - start_angle);

    if (fabsf(delta) >= PI / 2.0f)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_chassis.helm_angle[i] = 0.0f;
            helm_chassis.helm_speed[i] = 0.0f;
        }
        turning = false;
        return true;
    }

    // 因为轮子安装的正反方向不同所以要补偿
    static const float drive_sign[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
    // 不断更新每个轮子的坐标
    Cacl_wheel(Imu_angle, lidar_x, lidar_y);
    if (distance <= 0.001f)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_chassis.helm_speed[i] = 0.0f;
        }
        turning = false;
        return true;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        // x轴距离
        float temp_x = wheel[i].w_x - TURN_CENTER_X;
        // y轴距离
        float temp_y = wheel[i].w_y - TURN_CENTER_Y;
        float desired_wheel_angle = -atan2f(temp_x, temp_y);
        float desired_angle = wrap_pi(desired_wheel_angle - Imu_angle);
        // 计算轮子线速度
        float wheel_linear_speed = turn_speed * hypotf(temp_x, temp_y) / distance;
        // 舵角归一
        float feedback_angle = helm_motor[i].Feedback_angle * (PI / 180.0f);
        float normalized_feedback_angle = wrap_pi(feedback_angle);
        // 计算x轴和y轴速度
        wheel[i].v_x = wheel_linear_speed * cosf(desired_wheel_angle);
        wheel[i].v_y = wheel_linear_speed * sinf(desired_wheel_angle);

        helm_chassis.actual_helm_angle[i] = normalized_feedback_angle;
        helm_chassis.delta[i] = wrap_pi(desired_angle - normalized_feedback_angle);
        helm_chassis.helm_speed[i] = drive_sign[i] * wheel_linear_speed * 60.0f * 14.0f / ((2.0f * PI) * 0.090f);
        // 最小舵角
        Min_Angle(&helm_chassis.delta[i], &helm_chassis.helm_speed[i]);

        helm_chassis.helm_angle[i] = feedback_angle + helm_chassis.delta[i];
        helm_chassis.set_angle[i] = helm_chassis.helm_angle[i] * 180.0f / PI;
    }
    return false;
}

//1左转 -1右转 底盘坐标系
bool QuarterTurn(float turn_speed, float R, float LorF)
{
    static bool turning = false;
    static float start_angle = 0.0f;

    if (R <= 0.001f || fabsf(LorF) <= 0.001f)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_chassis.helm_speed[i] = 0.0f;
        }
        turning = false;
        return true;
    }

    if (!turning)
    {
        start_angle = Imu_angle;
        turning = true;
    }

    if (fabsf(wrap_pi(Imu_angle - start_angle)) >= PI / 2.0f)
    {
        //完成转向
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_chassis.helm_speed[i] = 0.0f;
        }
        turning = false;
        return true;
    }

    float turn_vx = 0.0f;
    float turn_vy = 0.0f;
    float initial_yaw = wrap_pi(start_angle);

    if (fabsf(initial_yaw) < 0.05f)                         // 0°
        turn_vx = turn_speed;
    else if (fabsf(initial_yaw - PI / 2.0f) < 0.05f)       // 90°
        turn_vy = -turn_speed;
    else if (fabsf(fabsf(initial_yaw) - PI) < 0.05f)       // 180°/-180°
        turn_vx = -turn_speed;
    else if (fabsf(initial_yaw + PI / 2.0f) < 0.05f)       // -90°
        turn_vy = turn_speed;

    Speed turn_body_speed = {
        .vx = turn_vx,
        .vy = turn_vy,
        .vw = ((LorF > 0.0f) ? 1.0f : -1.0f) * turn_speed / R
    };
    helm_calculate(&turn_body_speed, &helm_chassis);

    return false;
}
