#ifndef _SPI_FIFO_H_
#define _SPI_FIFO_H_

void spi_fifo_interface_bus_init(uint8_t baudmhz);
void spi_ctrl_cmd_read_gw_version(void);
void spi_ctrl_cmd_read_chip_id(void);
uint16_t spi_ctrl_read_tx_fifo_free_bytes(void);
uint16_t spi_fifo_get_tx_data_bytes(void);
bool spi_fifo_write_with_check(uint8_t *data, uint16_t len);
void spi_fifo_write_data(uint8_t *data, uint16_t len);
void spi_fifo_write_prepare(uint16_t len);
void spi_fifo_write_execute(uint16_t len);
uint16_t spi_fifo_read(void);
uint8_t rxdata_valid(void);

#endif /* _SPI_FIFO_H_ */