#ifndef _SPI_FIFO_H_
#define _SPI_FIFO_H_

void spi_fifo_interface_bus_init(uint8_t baudmhz);
void spi_ctrl_cmd_read_gw_version(void);
void spi_ctrl_cmd_read_chip_id(void);
uint16_t spi_ctrl_read_tx_fifo_free_space(void);
bool spi_fifo_write_with_check(uint8_t *data, uint16_t len);
void spi_fifo_write(uint8_t *data, uint16_t len);
uint16_t spi_fifo_find_max_data_len(void);
uint16_t spi_fifo_read(void);
uint8_t rxdata_valid(void);

#endif /* _SPI_FIFO_H_ */