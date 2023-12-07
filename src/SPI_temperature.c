#include "main.h"
#include "SPI_temperature.h"


// ----------------------------------------------
// this function receive on SPI the temperature
// ----------------------------------------------

void temperatureReceive(SPI_HandleTypeDef hspi1,uint8_t* tempReceivedSPI1) {

   Print_DEBUG("temp receive");

    //if( HAL_SPI_Receive(&hspi1,(uint8_t*) (tempReceivedSPI1),1,1000  ) != HAL_OK) {
      if( HAL_SPI_Receive(&hspi1,(uint8_t*) (tempReceivedSPI1),1,1000) != HAL_OK) {
            Print_DEBUG("Error in data reception SPI1");
          Error_Handler();
      }

}