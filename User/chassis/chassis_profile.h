#ifndef CHASSIS_PROFILE_H
#define CHASSIS_PROFILE_H

void Chassis_ProfileInit(void);
float JerkLimitedBrakingDistance(float current_speed_mm_s,
                                 float current_accel_mm_s2,
                                 float max_decel_mm_s2,
                                 float max_jerk_mm_s3);
float Chassis_ProfileUpdate(float max_speed_mm_s,
                            float remaining_distance_mm,
                            float period_s);

#endif /* CHASSIS_PROFILE_H */
