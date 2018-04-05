#ifndef GZY_LRF_H
#define GZY_LRF_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

#define GZY_COMMAND_BUFFER_SIZE 10
#define GZY_DATA_BUFFER_SIZE 80
#define GZY_MESSAGE_SIZE 35
#define INVALID 'R'
#define VALID 'V'
#define ERROR 'A'

typedef struct gzyDistanceStruct{
    uint16_t maximumDistance;
    uint16_t lastResponse;
    uint16_t firstResponse;
}gzyDistanceStruct;

extern UART_HandleTypeDef * gzy_uart;
extern osSemaphoreId lrfDataReceivedHandle;


void initGZY(UART_HandleTypeDef * huart);
void gzyMeasureCommand();
gzyDistanceStruct parseGZYMessage();

#endif
