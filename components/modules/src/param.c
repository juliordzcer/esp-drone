#include <string.h>
#include <errno.h>

/* FreeRtos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "crtp.h"
#include "param.h"
#include "crc.h"
#include "console.h"
#include "pid.h"
#include "controller.h" // Se asume que este header tiene la definición de PidObject

// **Declara las variables globales de los PID** (definidas en controller.c)
extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;
extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;

// Define la tabla completa con todos los parámetros
static const struct param_s allParams[] = {
    // Grupo: pid_attitude
    { .type = PARAM_FLOAT, .name = "pid_attitude.roll_kp", .address = &pidRoll.kp },
    { .type = PARAM_FLOAT, .name = "pid_attitude.roll_ki", .address = &pidRoll.ki },
    { .type = PARAM_FLOAT, .name = "pid_attitude.roll_kd", .address = &pidRoll.kd },
    { .type = PARAM_FLOAT, .name = "pid_attitude.pitch_kp", .address = &pidPitch.kp },
    { .type = PARAM_FLOAT, .name = "pid_attitude.pitch_ki", .address = &pidPitch.ki },
    { .type = PARAM_FLOAT, .name = "pid_attitude.pitch_kd", .address = &pidPitch.kd },
    { .type = PARAM_FLOAT, .name = "pid_attitude.yaw_kp", .address = &pidYaw.kp },
    { .type = PARAM_FLOAT, .name = "pid_attitude.yaw_ki", .address = &pidYaw.ki },
    { .type = PARAM_FLOAT, .name = "pid_attitude.yaw_kd", .address = &pidYaw.kd },

    // Grupo: pid_rate
    { .type = PARAM_FLOAT, .name = "pid_rate.roll_kp", .address = &pidRollRate.kp },
    { .type = PARAM_FLOAT, .name = "pid_rate.roll_ki", .address = &pidRollRate.ki },
    { .type = PARAM_FLOAT, .name = "pid_rate.roll_kd", .address = &pidRollRate.kd },
    { .type = PARAM_FLOAT, .name = "pid_rate.pitch_kp", .address = &pidPitchRate.kp },
    { .type = PARAM_FLOAT, .name = "pid_rate.pitch_ki", .address = &pidPitchRate.ki },
    { .type = PARAM_FLOAT, .name = "pid_rate.pitch_kd", .address = &pidPitchRate.kd },
    { .type = PARAM_FLOAT, .name = "pid_rate.yaw_kp", .address = &pidYawRate.kp },
    { .type = PARAM_FLOAT, .name = "pid_rate.yaw_ki", .address = &pidYawRate.ki },
    { .type = PARAM_FLOAT, .name = "pid_rate.yaw_kd", .address = &pidYawRate.kd },
};

#define TOC_CH 0
#define READ_CH 1
#define WRITE_CH 2

#define CMD_RESET 0
#define CMD_GET_NEXT 1
#define CMD_GET_CRC 2

#define CMD_GET_ITEM 0
#define CMD_GET_INFO 1

//Private functions
static void paramTask(void * prm);
void paramTOCProcess(int command);

//The following two function SHALL NOT be called outside paramTask!
static void paramWriteProcess(int id, void*);
static void paramReadProcess(int id);
static int variableGetIndex(int id);

//Pointer to the parameters list and length of it
static const struct param_s * params;
static int paramsLen;
static uint32_t paramsCrc;
static int paramsCount = 0;

static bool isInit = false;

void paramInit(void)
{
  int i;

  if(isInit)
    return;

  // Usa la tabla definida manualmente
  params = allParams; 
  paramsLen = sizeof(allParams) / sizeof(allParams[0]);
  paramsCrc = crcSlow(params, paramsLen);

  for (i=0; i<paramsLen; i++)
  {
    if(!(params[i].type & PARAM_GROUP)) 
      paramsCount++;
  }
  
  //Start the param task
  xTaskCreate(paramTask, "PARAM", 
            configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
  
  //TODO: Handle stored parameters!
  
  isInit = true;
}

bool paramTest(void)
{
  return isInit;
}

CRTPPacket p;

void paramTask(void * prm)
{
  crtpInitTaskQueue(CRTP_PORT_PARAM);
  
  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_PARAM, &p);
    
    if (p.channel==TOC_CH)
      paramTOCProcess(p.data[0]);
    else if (p.channel==READ_CH)
      paramReadProcess(p.data[0]);
    else if (p.channel==WRITE_CH)
      paramWriteProcess(p.data[0], &p.data[1]);
  }
}

void paramTOCProcess(int command)
{
  int ptr = 0;
  char * group = "";
  int n=0;
  
  switch (command)
  {
  case CMD_GET_INFO: //Get info packet about the param implementation
    ptr = 0;
    group = "";
    p.header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
    p.size=6;
    p.data[0]=CMD_GET_INFO;
    p.data[1]=paramsCount;
    memcpy(&p.data[2], &paramsCrc, 4);
    crtpSendPacket(&p);
    break;
  case CMD_GET_ITEM:  //Get param variable
    for (ptr=0; ptr<paramsLen; ptr++) //Ptr points a group
    {
      if (params[ptr].type & PARAM_GROUP)
      {
        if (params[ptr].type & PARAM_START)
          group = params[ptr].name;
        else
          group = "";
      }
      else                          //Ptr points a variable
      {
        if (n==p.data[1])
          break;
        n++;
      }
    }
    
    if (ptr<paramsLen)
    {
      p.header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.data[1]=n;
      p.data[2]=params[ptr].type;
      memcpy(p.data+3, group, strlen(group)+1);
      memcpy(p.data+3+strlen(group)+1, params[ptr].name, strlen(params[ptr].name)+1);
      p.size=3+2+strlen(group)+strlen(params[ptr].name);
      crtpSendPacket(&p);      
    } else {
      p.header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.size=1;
      crtpSendPacket(&p);
    }
    break;
  }
}

static void paramWriteProcess(int ident, void* valptr)
{
  int id;

  id = variableGetIndex(ident);
  
  if (id<0) {
    p.data[0] = -1;
    p.data[1] = ident;
    p.data[2] = ENOENT;
    p.size = 3;
    
    crtpSendPacket(&p);
    return;
  }

  if (params[id].type & PARAM_RONLY)
    return;

  switch (params[id].type & PARAM_BYTES_MASK)
  {
  case PARAM_1BYTE:
    *(uint8_t*)params[id].address = *(uint8_t*)valptr;
    break;
    case PARAM_2BYTES:
      *(uint16_t*)params[id].address = *(uint16_t*)valptr;
      break;
  case PARAM_4BYTES:
      *(uint32_t*)params[id].address = *(uint32_t*)valptr;
      break;
  case PARAM_8BYTES:
      *(uint64_t*)params[id].address = *(uint64_t*)valptr;
      break;
  }
  
  crtpSendPacket(&p);
}

static void paramReadProcess(int ident)
{
  int id;

  id = variableGetIndex(ident);
  
  if (id<0) {
    p.data[0] = -1;
    p.data[1] = ident;
    p.data[2] = ENOENT;
    p.size = 3;
    
    crtpSendPacket(&p);
    return;
  }

  switch (params[id].type & PARAM_BYTES_MASK)
  {
  case PARAM_1BYTE:
      memcpy(&p.data[1], params[id].address, sizeof(uint8_t));
      p.size = 1+sizeof(uint8_t);
      break;
    case PARAM_2BYTES:
      memcpy(&p.data[1], params[id].address, sizeof(uint16_t));
      p.size = 1+sizeof(uint16_t);
      break;
    case PARAM_4BYTES:
      memcpy(&p.data[1], params[id].address, sizeof(uint32_t));
      p.size = 1+sizeof(uint32_t);
      break;
    case PARAM_8BYTES:
      memcpy(&p.data[1], params[id].address, sizeof(uint64_t));
      p.size = 1+sizeof(uint64_t);
      break;
  }
  
  crtpSendPacket(&p);
}

static int variableGetIndex(int id)
{
  int i;
  int n=0;
  
  for (i=0; i<paramsLen; i++)
  {
    if(!(params[i].type & PARAM_GROUP)) 
    {
      if(n==id)
        break;
      n++;
    }
  }
  
  if (i>=paramsLen)
    return -1;
  
  return i;
}