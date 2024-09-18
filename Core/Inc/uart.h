/*
 * uart.h
 *
 *  Created on: Sep 16, 2024
 *      Author: User
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

//void init_uart_1(void);
void receive_data_from_uart(char *rx_buff);
void receive_data_from_uart_dma(char *rx_buff, uint16_t rx_buff_len,
		char *final_buff, uint16_t final_buff_len);
void transmit_data_to_uart(char *tx_buff);
void transmit_data_to_uart_dma(char *start_buff, uint16_t start_buff_len,
		char *tx_buff, uint16_t tx_buff_len);

#endif /* INC_UART_H_ */
