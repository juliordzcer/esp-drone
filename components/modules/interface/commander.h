#ifndef COMMANDER_H_
#define COMMANDER_H_
#include <stdint.h>
#include <stdbool.h>

// [CORRECCIÓN] Solo declaración en el header, definición en el .c
struct CommanderCrtpValues
{
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
  bool hover;
} __attribute__((packed));

#define COMMANDER_WDT_TIMEOUT_STABALIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

typedef enum
{
  RATE,
  ANGLE
} RPYType;

// [CORRECCIÓN] Solo declaración extern, no definición
extern struct CommanderCrtpValues targetVal[2];
extern int side;
extern bool hoverMode;  // <-- AÑADIR esta variable externa

void commanderInit(void);
bool commanderTest(void);

uint32_t commanderGetInactivityTime(void);
void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired);
void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType);
void commanderGetThrust(uint16_t* thrust);  // <-- SOLO UNA declaración
void commanderGetHover(bool* hover, bool* set_hover, float* hover_change);

#endif /* COMMANDER_H_ */