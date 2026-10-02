#ifndef MISSION_ACTION_H
#define MISSION_ACTION_H

#include "mission.h"

void Mission_Action_Reset(void);
/* rotate_target_deg is the absolute target yaw calculated by Mission. */
uint8_t Mission_Action_Start(const MissionAction *action,
                             float rotate_target_deg);
uint8_t Mission_Action_IsDone(const MissionAction *action);

#endif /* MISSION_ACTION_H */

