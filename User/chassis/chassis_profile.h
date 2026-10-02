#ifndef CHASSIS_PROFILE_H
#define CHASSIS_PROFILE_H

void Chassis_ProfileInit(void);
float Chassis_ProfileUpdate(float max_speed_mm_s,
                            float remaining_distance_mm,
                            float period_s);

#endif /* CHASSIS_PROFILE_H */
