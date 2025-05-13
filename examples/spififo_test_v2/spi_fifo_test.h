#ifndef _SPI_FIFO_TEST_H_
#define _SPI_FIFO_TEST_H_

void spi_fifo_interface_bus_init(uint8_t baudmhz);
void spi_ctrl_cmd_read_gw_version(void);
void spi_ctrl_cmd_read_chip_id(void);
void spi_ctrl_cmd_reset_fifo(void);
void send_msg1(void);
void send_msg2(void);
void spi_ctrl_cmd_read_data(void);
uint16_t spi_ctrl_cmd_read_tx_fifo_level(void);
uint16_t spi_ctrl_cmd_read_rx_fifo_level(void);
uint16_t spi_ctrl_read_tx_fifo_free_space(void);
void spi_ctrl_data_receive_loop(void);

#endif /* _SPI_FIFO_TEST_H_ */