#ifndef CONFIG_H_
#define CONFIG_H_
#include "nrf24l01.h"

#define CONFIG_BLOCK_ADDRESS 0x1FC00

#define P_NAME "Crazyflie Rev.F"

#define H_INIT_EXTI
#define T_LAUNCH_ADC
#define T_LAUNCH_RADIO
#define T_LAUNCH_POWERMANAGMENT

#define T_LAUNCH_CONTROL
#define T_LAUNCH_MULTILOG_1
#define T_LAUNCH_MULTICONTROL

//The radio channel. From 0 to 125
#define RADIO_CHANEL 10
#define RADIO_DATARATE RADIO_RATE_250K

#define ACTIVATE_AUTO_SHUTDOWN

#if defined(UART_OUTPUT_TRACE_DATA) && defined(ADC_OUTPUT_RAW_DATA)
#  error "Can't define UART_OUTPUT_TRACE_DATA and ADC_OUTPUT_RAW_DATA at the same time"
#endif

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
#define UART_OUTPUT_RAW_DATA_ONLY
#endif

#if defined(UART_OUTPUT_TRACE_DATA) && defined(T_LAUNCH_ACC)
#  error "UART_OUTPUT_TRACE_DATA and T_LAUNCH_ACC doesn't work at the same time yet due to dma sharing..."
#endif

#endif /* CONFIG_H_ */
