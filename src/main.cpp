#include <Arduino.h>
#include <MotorDistribute.h>
#include <Kinematics.h>
#include <EncoderDistribute.h>

// 引入Microros和wifi相关的库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>              // 消息接口
#include <micro_ros_utilities/string_utilities.h> // 引入字符串内存分配初始化工具

rcl_subscription_t sub_cmd_vel; // 创建一个订阅者
// 声明一些相关的结构体对像
rcl_allocator_t allocator;             // 内存分配器，用于动态内存分配管理
rclc_support_t support;                // 用于存储时钟，内存分配器和上下文，用于提供支持
rclc_executor_t executor;              // 执行器，用于管理订阅和计时器回调的执行
rcl_node_t node;                       // 节点，用于创建节点
geometry_msgs__msg__Twist msg_cmd_vel; // 订阅到的数据存储到这里

Kinematics kinematics;

float target_linear_speed = 0.0;  // 单位 毫米每秒
float target_angular_speed = 0.0; // 单位 弧度每秒
float out_left_speed = 0.0;       // 输出的左右轮速度，不是反馈的左右轮速度
float out_right_speed = 0.0;

void twist_calllback(const void *msg_in)
{
    // 将受到的消息指针转换成 geometry_msgs__msg__Twist 类型的指针
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    target_linear_speed = msg->linear.x * 1000;
    target_angular_speed = msg->angular.z;
    // 测试下运动学逆解
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    Serial.printf("OUT:left_speed=%f,right_speed=%f\n", out_left_speed, out_right_speed);
    // MotorDistribute::motorControl(1, target_linear_speed); // debug
    // MotorDistribute::motorControl(2, target_linear_speed); // debug
}

// 单独创建一个任务运行 micro-ROS 相当于一个线程
void microros_task(void *args)
{
    // 1.设置传输协议并延迟一段时间等待设置完成
    IPAddress agent_ip;
    agent_ip.fromString("10.135.47.48");                              // 设置agent的IP地址
    set_microros_wifi_transports("Ho2", "hkh63043202", agent_ip, 8888); // 设置传输协议
    delay(2000);                                                        // 等待2秒，等待WIFI连接
    // 2.初始化内存分配器
    allocator = rcl_get_default_allocator(); // 获取默认的内存分配器
    // 3. 初始化支持
    rclc_support_init(&support, 0, NULL, &allocator); // 初始化支持
    // 4.初始化节点
    rclc_node_init_default(&node, "ic_motion_control", "", &support); // 初始化节点
    // 5.初始化执行器
    unsigned int num_handles = 2; // 订阅和计时器的数量，注意这是一个要改的参数
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    // 初始化订阅者，并将其添加到执行器中
    rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_calllback, ON_NEW_DATA);
    // 时间同步
    while (!rmw_uros_epoch_synchronized())
    {
        rmw_uros_sync_session(1000);
        delay(10);
    }
    // 循环执行器
    rclc_executor_spin(&executor); // 循环执行器
}

void setup()
{
    Serial.begin(115200);
    MotorDistribute::begin();
    MotorDistribute::addMotor(1, 25, 33, 26);
    MotorDistribute::addMotor(2, 19, 21, 18);
    // 初始化编码器
    EncoderDistribute::begin();
    EncoderDistribute::addEncoder(1,32,35);
    EncoderDistribute::addEncoder(2,22,23);
    EncoderDistribute::updateAllDeltaTicks();
    // 初始化运动学参数
    kinematics.set_wheel_distance(810); // mm
    kinematics.set_motor_param(0, 0.0071895);
    kinematics.set_motor_param(1, 0.0071895);
    Serial.printf("Total motors: %d\n", MotorDistribute::getMotorCount());
    Serial.println("System Ready! (Two Drivers Mode)");

    // 创建一个任务运行 micro-ROS
    xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 1);
}

void loop()
{
    kinematics.update_motor_speed(millis(),
                                  EncoderDistribute::getTicks(1), EncoderDistribute::getTicks(2)); // 记得调用更新电机速度函数

    MotorDistribute::motorControl(1, out_left_speed);
    MotorDistribute::motorControl(2, out_right_speed);
//         if (Serial.available() > 0) {
//         char command = Serial.read();
        
//         // Convert to lowercase for easier handling
//         if (command >= 'A' && command <= 'Z') {
//             command = command + ('a' - 'A');
//         }
        
//         switch (command) {
//             case 'w':  // Forward
//                 Serial.printf("Moving FORWARD");
//                 MotorDistribute::motorControl(1, 200);
//                 MotorDistribute::motorControl(2, -200);
//                 break;
                
//             case 's':  // Backward
//                 Serial.printf("Moving BACKWARD");
//                 MotorDistribute::motorControl(1, -200);
//                 MotorDistribute::motorControl(2, 200);
//                 break;
                
//             case 'a':  // Turn Left
//                 Serial.printf("Turning LEFT");
//                 MotorDistribute::motorControl(1, -200);
//                 MotorDistribute::motorControl(2, -200);
//                 break;
                
//             case 'd':  // Turn Right
//                 Serial.printf("Turning RIGHT");
//                 MotorDistribute::motorControl(1, 200);
//                 MotorDistribute::motorControl(2, 200);
//                 break;
                
//             case 'q':  // Stop (but not emergency)
//                 Serial.println("Stopping all motors");
//                 MotorDistribute::stopAll();
//                 break;
                
//             case 'x':  // Emergency stop
//                 Serial.println("EMERGENCY STOP! All motors halted immediately");
//                 MotorDistribute::stopAll();
//                 break;
//         }
        
//         // Clear any remaining characters in buffer
//         while (Serial.available() > 0) {
//             Serial.read();
//         }
//     }
    
//     // Small delay to prevent overwhelming the system
//     delay(50);
}