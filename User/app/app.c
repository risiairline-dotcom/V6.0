#include "app.h"
#include "mission.h"
#include "button.h"
#include "chassis.h"
#include "hwt101.h"
#include "raspberry.h"
#include "task_code.h"
#include "tjc.h"
#include "main.h"
#include "test_gripper.h"
#include "test_lift.h"
#include "test_extend.h"
#include "test_servo1.h"
#include "test_servo2.h"
#include "test_mechanism.h"
#include "test_chassis_control.h"
#include "test_chassis_position.h"
#include "test_chassis_move_distance.h"
#include "test_heading.h"
#include "heading_pid.h"

/* 测试模式按以下优先级选择；全部为 0 时运行原有正式流程。 */
#define TEST_MODE_MECHANISM_ENABLE 0
#define TEST_MODE_SERVO2_ENABLE   0
#define TEST_MODE_SERVO1_ENABLE   0
#define TEST_MODE_LIFT_ENABLE     0
#define TEST_MODE_EXTEND_ENABLE   0
#define TEST_MODE_GRIPPER_ENABLE  0
#define TEST_MODE_CHASSIS_CONTROL_ENABLE 0
#define TEST_MODE_CHASSIS_POSITION_ENABLE 0
#define TEST_MODE_CHASSIS_MOVE_DISTANCE_ENABLE 1
#define TEST_MODE_HEADING_ENABLE 0

void App_Init(void)
{
#if TEST_MODE_MECHANISM_ENABLE
  Button_Init();
  Mechanism_Test_Init();
#elif TEST_MODE_SERVO2_ENABLE
  Button_Init();
  Servo2_Test_Init();
#elif TEST_MODE_SERVO1_ENABLE
  Button_Init();
  Servo1_Test_Init();
#elif TEST_MODE_LIFT_ENABLE
  Button_Init();
  Lift_Test_Init();
#elif TEST_MODE_EXTEND_ENABLE
  Button_Init();
  Extend_Test_Init();
#elif TEST_MODE_GRIPPER_ENABLE
  Button_Init();
  Gripper_Test_Init();
#elif TEST_MODE_CHASSIS_CONTROL_ENABLE
  Button_Init();
  Chassis_Control_Test_Init();
#elif TEST_MODE_CHASSIS_POSITION_ENABLE
  Button_Init();
  Chassis_Position_Test_Init();
#elif TEST_MODE_CHASSIS_MOVE_DISTANCE_ENABLE
  Button_Init();
  Chassis_MoveDistance_Test_Init();
#elif TEST_MODE_HEADING_ENABLE
  Button_Init();
  if (HWT101_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Heading_Test_Init();
#else
  /* 先启动航向接收，保证底盘任务开始前能够建立IMU零点。 */
  if (HWT101_Init() != HAL_OK)
  {
    Error_Handler();
  }

  TaskCode_Init();
  Raspberry_Init();
  TJC_Init();

  /* 初始化并停止底盘，确保上电后的四轮状态可控。 */
  Chassis_Init();

  /* 最后初始化上层状态机和按钮事件。 */
  Mission_Init();
  Button_Init();

#endif
}

void App_Run(void)
{
#if TEST_MODE_MECHANISM_ENABLE
  Mechanism_Test_Run();
#elif TEST_MODE_SERVO2_ENABLE
  Servo2_Test_Run();
#elif TEST_MODE_SERVO1_ENABLE
  Servo1_Test_Run();
#elif TEST_MODE_LIFT_ENABLE
  Lift_Test_Run();
#elif TEST_MODE_EXTEND_ENABLE
  Extend_Test_Run();
#elif TEST_MODE_GRIPPER_ENABLE
  Gripper_Test_Run();
#elif TEST_MODE_CHASSIS_CONTROL_ENABLE
  Button_Update();
  Chassis_Control_Test_Run();
#elif TEST_MODE_CHASSIS_POSITION_ENABLE
  Button_Update();
  Chassis_Position_Test_Run();
#elif TEST_MODE_CHASSIS_MOVE_DISTANCE_ENABLE
  Button_Update();
  Chassis_MoveDistance_Test_Run();
#elif TEST_MODE_HEADING_ENABLE
  Button_Update();
  Heading_Test_Run();
#else
  uint8_t start_event;

  Raspberry_Task();

  /* 非阻塞更新PC0消抖状态，并读取一次性按下事件。 */
  Button_Update();
  start_event = Button_GetStartEvent();

  /* 同一个单次按键事件负责启动和立即停止Mission。 */
  if (start_event != 0)
  {
    if (Mission_IsRunning() != 0)
      Mission_Stop();
    else
      Mission_Start();
  }

  /* 按当前路线推进正式 Mission。 */
  Mission_Run();
#endif
}
