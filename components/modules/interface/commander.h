#ifndef COMMANDER_H_
#define COMMANDER_H_
#include <stdint.h>
#include <stdbool.h>

// [CORRECCIÓN] Se define la estructura en el archivo .h
struct CommanderCrtpValues
{
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
} __attribute__((packed));

#define COMMANDER_WDT_TIMEOUT_STABALIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

typedef enum
{
  RATE,
  ANGLE
} RPYType;

extern struct CommanderCrtpValues targetVal[2];
extern int side;

void commanderInit(void);
bool commanderTest(void);

uint32_t commanderGetInactivityTime(void);
void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired);
void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType);
void commanderGetTrust(uint16_t* thrust);

#endif /* COMMANDER_H_ */