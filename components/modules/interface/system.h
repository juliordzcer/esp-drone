#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdbool.h>

void systemInit(void);
bool systemTest(void);

void systemLaunch(void);


void systemStart();
void systemWaitStart(void);
void systemSetCanFly(bool val);
bool systemCanFly(void);

#endif //__SYSTEM_H__
