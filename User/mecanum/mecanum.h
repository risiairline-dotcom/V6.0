#ifndef MECANUM_H
#define MECANUM_H

typedef struct
{
  float m1;
  float m2;
  float m3;
  float m4;
} MecanumWheels_t;

/* 逆运动学：输入车体平移量/速度和角度量/角速度，角度量使用弧度。 */
void Mecanum_Inverse(float x, float y, float rotation_rad,
                     MecanumWheels_t *wheels);

/* 将轮面移动距离转换为电机轴角度。 */
float distance_to_motor_angle(float distance_mm);

#endif /* MECANUM_H */
