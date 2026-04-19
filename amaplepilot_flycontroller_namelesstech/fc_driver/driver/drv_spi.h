#ifndef __DRV_SPI_H
#define __DRV_SPI_H



void spi1_init(uint16_t speed);
uint8_t spi1_sendreceive(uint8_t data);	

void spi2_init(uint16_t speed);
void spi2_setspeed(uint16_t SPI_BaudRatePrescaler);
uint8_t spi2_sendreceive(uint8_t data);	


void spi3_init(uint16_t speed);
void spi3_setspeed(uint16_t SPI_BaudRatePrescaler);
uint8_t spi3_sendreceive(uint8_t data);


#endif

