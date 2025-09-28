
#ifndef __LOG_H__
#define __LOG_H__

#include <stdbool.h>
#include <stdint.h>

/* Public functions */
void logInit(void);
bool logTest(void);

/* Basic log structure */
struct log_s {
  uint8_t type;
  char * name;
  void * address;
};

/* Possible variable types */
#define LOG_UINT8  1
#define LOG_UINT16 2
#define LOG_UINT32 3
#define LOG_INT8   4
#define LOG_INT16  5
#define LOG_INT32  6
#define LOG_FLOAT  7
#define LOG_FP16   8

/* Internal defines */
#define LOG_GROUP 0x80
#define LOG_START 1
#define LOG_STOP  0

/* Macros */
#define LOG_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_GROUP_START(NAME)  \
  static const struct log_s __logs_##NAME[] __attribute__((section(".log." #NAME), used)) = { \
  LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, 0x0)

//#define LOG_GROUP_START_SYNC(NAME, LOCK) LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, LOCK);

#define LOG_GROUP_STOP(NAME) \
  LOG_ADD_GROUP(LOG_GROUP | LOG_STOP, stop_##NAME, 0x0) \
  };

#endif /* __LOG_H__ */

