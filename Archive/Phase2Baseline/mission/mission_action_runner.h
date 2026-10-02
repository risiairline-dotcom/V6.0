#ifndef MISSION_ACTION_RUNNER_H
#define MISSION_ACTION_RUNNER_H

#include "mission.h"
#include "chassis.h"

void Mission_ActionRunner_Reset(void);
/* MOVE 仅执行纯 X 或纯 Y；FAST_MOVE 用 param1 查找 Fast Block。
 * GRAB/PLACE/FINISH 仍仅预留类型。 */
uint8_t Mission_ActionRunner_NeedsImu(const MissionAction_t *action);
ChassisStatus_t Mission_ActionRunner_Start(const MissionAction_t *action,
                                           float *planned_yaw);
uint8_t Mission_ActionRunner_IsDone(const MissionAction_t *action);
uint8_t Mission_ActionRunner_IsTimedOut(const MissionAction_t *action);

#endif /* MISSION_ACTION_RUNNER_H */
