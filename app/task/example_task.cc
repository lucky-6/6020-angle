//
// Created by fish on 2026/1/24.
//

#include "utils/os.h"

#include "bsp/uart.h"
#include "controller/pid.h"
#include "dsp/fast_math_functions.h"
#include "ins/ins.h"
#include "motor/dji.h"
#include "utils/vofa.h"

motor::dji Motor("Motor", motor::dji::GM6020, {
                  .id = 4,
                  .port = E_CAN_2,
                  .mode = motor::dji::CURRENT
              }, 100);

controller::pid Motor_pid_speed(300.0, 1.0, 0, 12000, 25000);
controller::pid Motor_pid_angle(30.0, 0.0, 2, 12000, 30);

static constexpr float target_speed_angle = 1.0f; //弧度制

// 在 [-PI, PI] 内选择到目标角的最短路径
static float shortest_arc_target(float current, float target) {
    const float error = target - current;
    if (error > PI) return target - 2.0f * PI;
    if (error < -PI) return target + 2.0f * PI;
    return target;
}

[[noreturn]] void example_task(void *args) {


    Motor.init();

    for (;;) {

        vofa::send(E_UART_10,

            Motor.feedback.angle,
            Motor.feedback.speed,
            Motor.output
            //state.gyro[0],
            //state.gyro[1],
            //state.gyro[2],
            //state.yaw,
            //state.yaw_total_angle
        );

        const float angle = Motor.feedback.angle;
        const float angle_target = shortest_arc_target(angle, target_speed_angle);//优劣弧判断

        Motor.update(Motor_pid_speed.update(Motor.feedback.speed,                   //速度环
            Motor_pid_angle.update(angle, angle_target)));                 //位置环

        os::task::sleep(10);
    }
}
