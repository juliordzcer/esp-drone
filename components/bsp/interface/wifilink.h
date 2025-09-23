/**
 * ESP32 Wi-Fi implementation of the CRTP link.
 */

#ifndef WIFILINK_H_
#define WIFILINK_H_

#include <stdint.h>
#include <stdbool.h>
#include "crtp.h"

void wifilinkInit(void);
bool wifilinkTest(void);
struct crtpLinkOperations* wifilinkGetLink(void);
void wifilinkReInit(void);

#endif /* WIFILINK_H_ */