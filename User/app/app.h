#ifndef APP_H
#define APP_H

/* 根据编译模式初始化夹爪测试或原有正式流程。 */
void App_Init(void);

/* 裸机主循环周期入口，运行当前模式的任务。 */
void App_Run(void);

#endif /* APP_H */
