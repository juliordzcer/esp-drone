#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "config.h"
#include "crtp.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "fp16.h"
#include "stabilizer.h"
#include "console.h"

#if 0
#define DEBUG(fmt, ...) consolePrintf("D/log " fmt, ## __VA_ARGS__)
#define ERROR(fmt, ...) consolePrintf("E/log " fmt, ## __VA_ARGS__)
#else
#define DEBUG(...)
#define ERROR(...)
#endif

// Define la estructura directamente en este archivo
typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_s;

// Define la variable global (no extern)
static attitude_s attitude = {0};

// Redefine LOG_ADD para que coincida con el formato esperado (sin coma extra)
#undef LOG_ADD
#define LOG_ADD(TYPE, NAME, ADDRESS) {TYPE, #NAME, ADDRESS}

const struct log_s allLogs[] = {
    // Agrega todas las variables que necesites loguear
    LOG_ADD(LOG_FLOAT, roll, &attitude.roll),
    LOG_ADD(LOG_FLOAT, pitch, &attitude.pitch),
    LOG_ADD(LOG_FLOAT, yaw, &attitude.yaw)
    // Agrega más entradas aquí si es necesario
};
static const int allLogsCount = sizeof(allLogs) / sizeof(allLogs[0]);

static const uint8_t typeLength[] = {
  [LOG_UINT8]  = 1,
  [LOG_UINT16] = 2,
  [LOG_UINT32] = 4,
  [LOG_INT8]   = 1,
  [LOG_INT16]  = 2,
  [LOG_INT32]  = 4,
  [LOG_FLOAT]  = 4,
  [LOG_FP16]   = 2,
};

// Maximum log payload length
#define LOG_MAX_LEN 30

/* Log packet parameters storage */
#define LOG_MAX_OPS 64
#define LOG_MAX_BLOCKS 8
struct log_ops {
  struct log_ops * next;
  uint8_t storageType : 4;
  uint8_t logType     : 4;
  void * variable;
};

struct log_block {
  int id;
  TimerHandle_t timer;
  struct log_ops * ops;
};

static struct log_ops logOps[LOG_MAX_OPS];
static struct log_block logBlocks[LOG_MAX_BLOCKS];
static SemaphoreHandle_t logLock;

struct ops_setting {
    uint8_t logType;
    uint8_t id;
} __attribute__((packed));


#define TOC_CH      0
#define CONTROL_CH  1
#define LOG_CH      2

#define CMD_GET_ITEM 0
#define CMD_GET_INFO 1

#define CONTROL_CREATE_BLOCK 0
#define CONTROL_APPEND_BLOCK 1
#define CONTROL_DELETE_BLOCK 2
#define CONTROL_START_BLOCK  3
#define CONTROL_STOP_BLOCK   4
#define CONTROL_RESET        5

#define BLOCK_ID_FREE -1

//Private functions
static void logTask(void * prm);
static void logTOCProcess(int command);
static void logControlProcess(void);

void logRunBlock(void * arg);
void logBlockTimed(TimerHandle_t timer);

//Pointer to the logeters list and length of it
static const struct log_s * logs;
static int logsLen;
static uint32_t logsCrc;
static int logsCount = 0;

static bool isInit = false;

/* Log management functions */
static int logAppendBlock(int id, struct ops_setting * settings, int len);
static int logCreateBlock(unsigned char id, struct ops_setting * settings, int len);
static int logDeleteBlock(int id);
static int logStartBlock(int id, unsigned int period);
static int logStopBlock(int id);
static void logReset();

// Función wrapper para crcSlow que acepta const
static uint32_t crcSlowConst(const void *datas, int nBytes) {
    return crcSlow((void*)datas, nBytes);
}

void logInit(void)
{
  int i;
  
  if(isInit)
    return;

  logs = allLogs;
  logsLen = allLogsCount;
  logsCrc = crcSlowConst(logs, logsLen * sizeof(struct log_s));
  

  // Big lock that protects the log datastructures
  logLock = xSemaphoreCreateMutex();

  for (i=0; i<logsLen; i++)
  {
    if(!(logs[i].type & LOG_GROUP)) 
      logsCount++;
  }
  
  //Manually free all log blocks
  for(i=0; i<LOG_MAX_BLOCKS; i++)
    logBlocks[i].id = BLOCK_ID_FREE;

  //Init data structures and set the log subsystem in a known state
  logReset();
  
  //Start the log task
  xTaskCreate(logTask, "log",
    configMINIMAL_STACK_SIZE + 2048, NULL, /*priority*/1, NULL); 

  isInit = true;
  ESP_LOGI("LOG", "Log module initialized.");
}

bool logTest(void)
{
  return isInit;
}

static CRTPPacket p;

void logTask(void * prm)
{
  static CRTPPacket p; // Hacerla static para que no use stack de la tarea
  
  crtpInitTaskQueue(CRTP_PORT_LOG);
  
  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_LOG, &p);
    
    xSemaphoreTake(logLock, portMAX_DELAY);
    if (p.channel==TOC_CH)
      logTOCProcess(p.data[0]);
    else if (p.channel==CONTROL_CH)
      logControlProcess();
    xSemaphoreGive(logLock);
    
    // Pequeña pausa para dar tiempo a otras tareas
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void logTOCProcess(int command)
{
  int ptr = 0;
  const char * group = "";
  int n=0;
  
  switch (command)
  {
  case CMD_GET_INFO: //Get info packet about the log implementation
    DEBUG("Packet is TOC_GET_INFO\n");
    ptr = 0;
    group = "";
    p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
    p.size=8;
    p.data[0]=CMD_GET_INFO;
    p.data[1]=logsCount;
    memcpy(&p.data[2], &logsCrc, sizeof(uint32_t));
    p.data[6]=LOG_MAX_BLOCKS;
    p.data[7]=LOG_MAX_OPS;
    crtpSendPacket(&p);
    break;
  case CMD_GET_ITEM:  //Get log variable
    DEBUG("Packet is TOC_GET_ITEM Id: %d\n", p.data[1]);
    for (ptr=0; ptr<logsLen; ptr++) //Ptr points a group
    {
      if (logs[ptr].type & LOG_GROUP)
      {
        if (logs[ptr].type & LOG_START)
          group = logs[ptr].name;
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
    
    if (ptr<logsLen)
    {
      DEBUG("    Item is \"%s\":\"%s\"\n", group, logs[ptr].name);
      p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.data[1]=n;
      p.data[2]=logs[ptr].type;
      memcpy(p.data+3, group, strlen(group)+1);
      memcpy(p.data+3+strlen(group)+1, logs[ptr].name, strlen(logs[ptr].name)+1);
      p.size=3+2+strlen(group)+strlen(logs[ptr].name);
      crtpSendPacket(&p);      
    } else {
      DEBUG("    Index out of range!");
      p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.size=1;
      crtpSendPacket(&p);
    }
    break;
  }
}

void logControlProcess()
{
  int ret = ENOEXEC;

  switch(p.data[0])
  {
    case CONTROL_CREATE_BLOCK:
      ret = logCreateBlock( p.data[1],
                            (struct ops_setting*)&p.data[2], 
                            (p.size-2)/sizeof(struct ops_setting) );
      break;
    case CONTROL_APPEND_BLOCK:
      ret = logAppendBlock( p.data[1],
                            (struct ops_setting*)&p.data[2], 
                            (p.size-2)/sizeof(struct ops_setting) );
      break;
    case CONTROL_DELETE_BLOCK:
      ret = logDeleteBlock( p.data[1] );
      break;
    case CONTROL_START_BLOCK:
      ret = logStartBlock( p.data[1], p.data[2]*10);
      break;
    case CONTROL_STOP_BLOCK:
      ret = logStopBlock( p.data[1] );
      break;
    case CONTROL_RESET:
      logReset();
      ret = 0;
      break;
  }
  
  //Commands answer
  p.data[2] = ret;
  p.size = 3;
  crtpSendPacket(&p);
}

static int logCreateBlock(unsigned char id, struct ops_setting * settings, int len)
{
  int i;
  
  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == BLOCK_ID_FREE) break;
  
  if (i == LOG_MAX_BLOCKS)
    return ENOMEM;
  
  logBlocks[i].id = id;
  logBlocks[i].timer = xTimerCreate("logTimer", pdMS_TO_TICKS(1000), 
                                     pdTRUE, &logBlocks[i], logBlockTimed );
  logBlocks[i].ops = NULL;
  
  if (logBlocks[i].timer == NULL)
  {
    logBlocks[i].id = BLOCK_ID_FREE;
    return ENOMEM;
  }

  DEBUG("Added block ID %d\n", id);
  
  return logAppendBlock(id, settings, len);
}

static int blockCalcLength(struct log_block * block);
static struct log_ops * opsMalloc();
static void opsFree(struct log_ops * ops);
static void blockAppendOps(struct log_block * block, struct log_ops * ops);
static int variableGetIndex(int id);

static int logAppendBlock(int id, struct ops_setting * settings, int len)
{
  int i;
  struct log_block * block;
  
  DEBUG("Appending %d variable to block %d\n", len, id);
  
  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;
  
  if (i >= LOG_MAX_BLOCKS) {
    ERROR("Trying to append block id %d that doesn't exist.", id);
    return ENOENT;
  }
  
  block = &logBlocks[i];
  
  for (i=0; i<len; i++)
  {
    int currentLength = blockCalcLength(block);
    struct log_ops * ops;
    int varId;
    
    if ((currentLength + typeLength[settings[i].logType&0x0F])>LOG_MAX_LEN) {
      ERROR("Trying to append a full block. Block id %d.\n", id);
      return E2BIG;
    }
    
    ops = opsMalloc();
    
    if(!ops) {
      ERROR("No more ops memory free!\n");
      return ENOMEM;
    }
    
    if (settings[i].id != 255)  //TOC variable
    {
      varId = variableGetIndex(settings[i].id);
      
      if (varId<0) {
        ERROR("Trying to add variable Id %d that does not exists.", settings[i].id);
        return ENOENT;
      }
      
      ops->variable    = logs[varId].address;
      ops->storageType = logs[varId].type;
      ops->logType     = settings[i].logType&0x0F;
      
      DEBUG("Appended variable %d to block %d\n", settings[i].id, id);
    } else {                     //Memory variable
      //TODO: Check that the address is in ram
      ops->variable    = (void*)(&settings[i]+1);
      ops->storageType = (settings[i].logType>>4)&0x0F;
      ops->logType     = settings[i].logType&0x0F;
      i += 2;
      
      DEBUG("Appended var addr 0x%x to block %d\n", (int)ops->variable, id);
    }
    blockAppendOps(block, ops);
    
    DEBUG("   Now lenght %d\n", blockCalcLength(block));
  }
  
  return 0;
}

static int logDeleteBlock(int id)
{
  int i;
  struct log_ops * ops;
  struct log_ops * opsNext;
  
  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;
  
  if (i >= LOG_MAX_BLOCKS) {
    ERROR("trying to delete block id %d that doesn't exist.", id);
    return ENOENT;
  }
  
  ops = logBlocks[i].ops;
  while (ops)
  {
    opsNext = ops->next;
    opsFree(ops);
    ops = opsNext;
  }
  
  if (logBlocks[i].timer != NULL) {
    xTimerStop(logBlocks[i].timer, portMAX_DELAY);
    xTimerDelete(logBlocks[i].timer, portMAX_DELAY);
    logBlocks[i].timer = NULL;
  }
  
  logBlocks[i].id = BLOCK_ID_FREE;
  return 0;
}

static int logStartBlock(int id, unsigned int period)
{
  int i;
  
  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;
  
  if (i >= LOG_MAX_BLOCKS) {
    ERROR("Trying to start block id %d that doesn't exist.", id);
    return ENOENT;
  }
  
  DEBUG("Starting block %d with period %dms\n", id, period);
  
  if (period>0)
  {
    xTimerChangePeriod(logBlocks[i].timer, pdMS_TO_TICKS(period), portMAX_DELAY);
    xTimerStart(logBlocks[i].timer, portMAX_DELAY);
  } else {
    // single-shoot run
    workerSchedule(logRunBlock, &logBlocks[i]);
  }
  
  return 0;
}

static int logStopBlock(int id)
{
  int i;
  
  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;
  
  if (i >= LOG_MAX_BLOCKS) {
    ERROR("Trying to stop block id %d that doesn't exist.\n", id);
    return ENOENT;
  }
  
  xTimerStop(logBlocks[i].timer, portMAX_DELAY);
  
  return 0;
}

/* This function is called by the timer subsystem */
void logBlockTimed(TimerHandle_t timer)
{
  workerSchedule(logRunBlock, pvTimerGetTimerID(timer));
}

/* This function is usually called by the worker subsystem */
void logRunBlock(void * arg)
{
  struct log_block *blk = arg;
  struct log_ops *ops = blk->ops;
  CRTPPacket pk = {0};
  unsigned int timestamp;
  
  xSemaphoreTake(logLock, portMAX_DELAY);

  timestamp = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;
  
  pk.header = CRTP_HEADER(CRTP_PORT_LOG, LOG_CH);
  pk.size = 1;
  pk.data[0] = blk->id;
  
  memcpy(&pk.data[1], &timestamp, sizeof(timestamp));
  pk.size += sizeof(timestamp);
  
  while (ops)
  {
    int valuei = 0;
    float valuef = 0;
     
    switch(ops->storageType)
    {
      case LOG_UINT8:
        valuei = *(uint8_t *)ops->variable;
        break;
      case LOG_INT8:
        valuei = *(int8_t *)ops->variable;
        break;
      case LOG_UINT16:
        valuei = *(uint16_t *)ops->variable;
        break;
      case LOG_INT16:
        valuei = *(int16_t *)ops->variable;
        break;
      case LOG_UINT32:
        valuei = *(uint32_t *)ops->variable;
        break;
      case LOG_INT32:
        valuei = *(int32_t *)ops->variable;
        break;
      case LOG_FLOAT:
        valuei = *(float *)ops->variable;
        break;
    }
    
    if (ops->logType == LOG_FLOAT || ops->logType == LOG_FP16)
    {
      if (ops->storageType == LOG_FLOAT)
        valuef = *(float *)ops->variable;
      else
        valuef = valuei;
      
      if (ops->logType == LOG_FLOAT)
      {
        memcpy(&pk.data[pk.size], &valuef, 4);
        pk.size += 4;
      }
      else
      {
        valuei = single2half(valuef);
        memcpy(&pk.data[pk.size], &valuei, 2);
        pk.size += 2;
      }
    }
    else  //logType is an integer
    {
      memcpy(&pk.data[pk.size], &valuei, typeLength[ops->logType]);
      pk.size += typeLength[ops->logType];
    }
    
    ops = ops->next;
  }
  
  xSemaphoreGive(logLock);

  crtpSendPacket(&pk);
}

static int variableGetIndex(int id)
{
  int i;
  int n=0;
  
  for (i=0; i<logsLen; i++)
  {
    if(!(logs[i].type & LOG_GROUP)) 
    {
      if(n==id)
        break;
      n++;
    }
  }
  
  if (i>=logsLen)
    return -1;
  
  return i;
}

static struct log_ops * opsMalloc()
{
  int i;

  for (i=0;i<LOG_MAX_OPS; i++)
      if (logOps[i].variable == NULL) break;

  if (i >= LOG_MAX_OPS)
      return NULL;

  return &logOps[i];
}

static void opsFree(struct log_ops * ops)
{
  ops->variable = NULL;
}

static int blockCalcLength(struct log_block * block)
{
  struct log_ops * ops;
  int len = 0;
  
  for (ops = block->ops; ops; ops = ops->next)
    len += typeLength[ops->logType];

  return len;
}

void blockAppendOps(struct log_block * block, struct log_ops * ops)
{
  struct log_ops * o;
  
  ops->next = NULL;
  
  if (block->ops == NULL)
    block->ops = ops;
  else
  {
    for (o = block->ops; o->next; o = o->next);
    
    o->next = ops;
  }
}

static void logReset(void)
{
  int i;
  
  if (isInit)
  {
    //Stop and delete all started log blocks
    for(i=0; i<LOG_MAX_BLOCKS; i++)
      if (logBlocks[i].id != -1)
      {
        logStopBlock(logBlocks[i].id);
        logDeleteBlock(logBlocks[i].id);
      }
  }
  
  //Force free all the log block objects
  for(i=0; i<LOG_MAX_BLOCKS; i++)
    logBlocks[i].id = BLOCK_ID_FREE;
  
  //Force free the log ops
  for (i=0; i<LOG_MAX_OPS; i++)
    logOps[i].variable = NULL;
}