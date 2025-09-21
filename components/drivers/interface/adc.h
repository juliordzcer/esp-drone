#ifndef ADC_H_
#define ADC_H_

#include <stdbool.h>
#include <stdint.h>

/*** Public interface ***/
void adcInit(void);
bool adcTest(void);
float adcGetBatteryVoltage(void);

#endif /* ADC_H_ */