#include "stm32f7xx_hal.h"
#include "GZY_LRF.h"


UART_HandleTypeDef * gzy_uart;
uint8_t gzyCommandBuffer[GZY_COMMAND_BUFFER_SIZE];
uint8_t gzyDataBuffer[GZY_DATA_BUFFER_SIZE];
uint8_t gzyMessage[GZY_MESSAGE_SIZE];

uint8_t gzyRxIdx = 0;
uint8_t gzyTxIdx = 0;

gzyDistanceStruct gzyDistance;
uint8_t wait = 0;

uint8_t crc8(uint8_t *buffer, uint8_t len)
{
    uint8_t crc = 0x00;
    uint8_t i;

    while (len--)
    {
        crc += *buffer++;


    }

    return crc & 0xFF;
}

void initGZY(UART_HandleTypeDef * huart){
    gzy_uart = huart;
    __HAL_UART_ENABLE_IT(gzy_uart, UART_IT_RXNE);
}

void gzyMeasureCommand(){
    sprintf(&gzyCommandBuffer,">Md1\r");
    memset(&gzyDataBuffer, 0, GZY_DATA_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(gzy_uart, UART_IT_TXE);
}

gzyDistanceStruct parseGZYMessage(){
    uint8_t i = 0;
    uint8_t * p = &gzyMessage;
    for(i = 0; i < 3; ++i){
    	if(p[1] == 'v'){
			uint8_t crc1 = crc8(&p[1],8);

			uint8_t crc_h = p[9];
			if(crc_h >= 65) crc_h -= 55;
			else crc_h -= 48;

			uint8_t crc_l = p[10];
			if(crc_l >= 65) crc_l -= 55;
			else crc_l -= 48;
            
			uint8_t crc2 = crc_h << 4 | crc_l;

			if(crc2 == crc1){
				uint32_t mn_10 = 1;
				uint16_t distance = 0;

				uint8_t j = 0;
				for(j = 2; j < 7; ++j){
					distance += (p[8 - j] - 48)*mn_10;
					mn_10 *= 10;
				}

//				*(((uint8_t *)&gzyDistance) +4*i) = (float) distance / 100.0f;
				if(i == 0){
					gzyDistance.maximumDistance = distance;
				}else if(i == 1){
					gzyDistance.lastResponse = distance;
				}else{
					gzyDistance.firstResponse = distance;
				}

			}
			p += 11;
    	}

    }

    return gzyDistance;
    // xQueueA


}

void LRF_UART_IRQ_Handler(UART_HandleTypeDef * huart) {
  if (huart->Instance == gzy_uart->Instance) {

    // RECEPTION
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
        && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET)) {

      if(gzyRxIdx < GZY_DATA_BUFFER_SIZE){
        if(huart->Instance->RDR != 0)
          gzyDataBuffer[gzyRxIdx++] = huart->Instance->RDR;
        if(gzyRxIdx > 1 && gzyDataBuffer[gzyRxIdx - 2] == '<' && (gzyDataBuffer[gzyRxIdx - 1] == 'v' ||
                                gzyDataBuffer[gzyRxIdx - 1] == 'R' || gzyDataBuffer[gzyRxIdx - 1] == 'a')){
        	wait = 1;
        }else if(wait == 1 && gzyDataBuffer[gzyRxIdx - 1] == '<'){
        	wait = 0;
            memcpy(&gzyMessage, &gzyDataBuffer[gzyRxIdx  - GZY_MESSAGE_SIZE], GZY_MESSAGE_SIZE);
            gzyRxIdx = 0;
            // parseGZYMessage();
            static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(lrfDataReceivedHandle, xHigherPriorityTaskWoken);
            if(xHigherPriorityTaskWoken == pdTRUE){
              taskYIELD();
            }
        }

      }else{
        gzyRxIdx = 0;
      }

      __HAL_UART_FLUSH_DRREGISTER(huart);
      return;
    }

    // TRANSMISSION
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) != RESET)) {
      huart->Instance->TDR = gzyCommandBuffer[gzyTxIdx++];

      if(gzyCommandBuffer[gzyTxIdx - 1] == '\r'){
        gzyTxIdx = 0;
        while(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

      }

      return;
    }
    __HAL_UART_FLUSH_DRREGISTER(huart);
  }

}
