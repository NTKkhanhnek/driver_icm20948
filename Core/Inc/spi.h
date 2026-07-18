#ifndef INC_SPI_H_
#define INC_SPI_H_

#include <stdint.h>

typedef enum
{
    SPI_OK = 0,
    SPI_ERROR,
    SPI_TIMEOUT
} SPI_Status_t;

typedef enum
{
    SPI_CS_LOW = 0,
    SPI_CS_HIGH
} SPI_CS_State_t;

void SPI_init(void);
SPI_Status_t SPI_transmit(const uint8_t *data, uint16_t len);
SPI_Status_t SPI_receive(uint8_t *data, uint16_t len);
SPI_Status_t SPI_transmit_receive(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
SPI_Status_t SPI_mem_write(uint8_t reg_addr, const uint8_t *data, uint16_t len);
SPI_Status_t SPI_mem_read(uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif /* INC_SPI_H_ */