#include "button.h"
#include "main.h"

#define BUTTON_DEBOUNCE_MS  25U
#define BUTTON_PRESSED_LEVEL GPIO_PIN_RESET

static GPIO_PinState stable_state;
static GPIO_PinState candidate_state;
static uint32_t candidate_since;
static uint8_t start_pressed_event;
static uint8_t start_armed;

void Button_Init(void)
{
  /* PC0 已由 MX_GPIO_Init 配置为上拉输入，此处只初始化消抖状态。 */
  stable_state = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
  candidate_state = stable_state;
  candidate_since = HAL_GetTick();
  start_pressed_event = 0U;
  start_armed = (stable_state == GPIO_PIN_SET) ? 1U : 0U;
}

void Button_Update(void)
{
  GPIO_PinState raw_state;
  uint32_t now;

  now = HAL_GetTick();
  raw_state = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);

  if (raw_state != candidate_state)
  {
    candidate_state = raw_state;
    candidate_since = now;
  }
  else if ((candidate_state != stable_state) &&
           ((uint32_t)(now - candidate_since) >= BUTTON_DEBOUNCE_MS))
  {
    stable_state = candidate_state;
    if (stable_state == GPIO_PIN_SET)
    {
      /* 按钮释放后只重新解锁，不停止已经启动的任务。 */
      start_armed = 1U;
    }
    else if ((stable_state == BUTTON_PRESSED_LEVEL) && (start_armed == 1U))
    {
      start_pressed_event = 1U;
      start_armed = 0U;
    }
  }
}

uint8_t Button_GetStartEvent(void)
{
  uint8_t event = start_pressed_event;

  start_pressed_event = 0U;
  return event;
}

uint8_t Button_GetStopEvent(void)
{
  /* [DORMANT-COMPAT] 保留原接口；瞬时启动按钮不再产生停止事件。 */
  return 0U;
}

uint8_t Button_IsOn(void)
{
  /* [DORMANT] 当前App未调用，保留供按键状态诊断。 */
  return (stable_state == GPIO_PIN_RESET) ? 1U : 0U;
}
