#include "user_interface.h"
#include "referee_task.h"
#include "cmsis_os.h"
#include "protocol.h"

#define UI_TASK_DELAY 20

extern osThreadId_t UITaskHandle;
const osThreadAttr_t UITaskAttribute = {.name = "UITask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 1024 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

static communication::UserInterface* UI = nullptr;

void UITask(void* arg);
void init_ui();
void kill_ui();



extern communication::Referee* referee;
extern RefereeUART* referee_uart;