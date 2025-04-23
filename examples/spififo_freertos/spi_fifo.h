#ifndef _SPI_FIFO_H_
#define _SPI_FIFO_H_

void spi_fifo_interface_bus_init(uint8_t baudmhz);
void spi_fifo_task_init(void);
void spi_ctrl_send_byte(uint8_t byte);
void spi_ctrl_cmd_read_gw_version(void);
void spi_ctrl_cmd_read_chip_id(void);
void spi_ctrl_cmd_reset_fifo(void);
void spi_ctrl_cmd_write_data_len(uint16_t len);
void spi_ctrl_cmd_read_data_len(void);
void spi_ctrl_cmd_write_data(void);
void spi_ctrl_cmd_write_data_with_given_data_len(void);
void spi_ctrl_cmd_read_data(void);
void spi_ctrl_cmd_read_tx_fifo_level(void);
void spi_ctrl_cmd_read_rx_fifo_level(void);

#endif /* _SPI_FIFO_H_ */