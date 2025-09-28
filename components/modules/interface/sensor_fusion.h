
#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_
#include <stdbool.h>

void sensfusion6Init(void);
bool sensfusion6Test(void);

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw, float az, float* accWZ);

#endif /* SENSOR_FUSION_H_ */
